import streamlit as st
import asyncio
import os
import cv2
import numpy as np
import time
import subprocess
from PIL import Image as PILImage
import google.generativeai as genai
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

# ==========================================
# CONFIGURATION
# ==========================================
GOOGLE_API_KEY = "paste your own api key"
SERVER_SCRIPT = "/home/yahboom/my_robot_project/ros-mcp-server/server.py"

# YOUR MAP COORDINATES
LOCATIONS = {
    "kitchen": [-0.47, -0.52, 0.0, 1.0],           # Facing East
    "living_room": [0.38, 1.29, 0.383, 0.924],     # Facing North-East
    "bedroom": [1.95, 1.40, -0.924, 0.383],        # Facing South-West
    "home": [0.03, -0.7, 0.7076, 0.7065]           # Facing North
}
# ==========================================

st.set_page_config(page_title=" WALL-X", page_icon="🤖️", layout="wide")

# --- SIDEBAR ---
with st.sidebar:
    st.title("Control Panel")
    if st.button("🗑️ Clear Chat History", type="primary"):
        st.session_state.messages = []
        st.session_state.messages.append({"role": "assistant", "content": "Memory cleared. Systems ready."})
        st.rerun()

st.title("🤖️ WALL-X")

# Initialize Chat
if "messages" not in st.session_state:
    st.session_state.messages = []
    st.session_state.messages.append({"role": "assistant", "content": "System Online. Ask💬️: 'What do you see?' or 'Go to kitchen'."})

# Display Chat History
for message in st.session_state.messages:
    with st.chat_message(message["role"]):
        st.markdown(message["content"])
        if "image" in message:
            st.image(message["image"], width=400)

# --- MAIN AI LOGIC ---
async def process_command(user_text):
    genai.configure(api_key=GOOGLE_API_KEY)
    
    server_params = StdioServerParameters(command="python3", args=[SERVER_SCRIPT], env=dict(os.environ))

    async with stdio_client(server_params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            
            # --- DEFINE TOOLS ---
            def move_smart_wrapper(room_name: str): return {"action": "move", "room": room_name}
            def battery_smart_wrapper(): return {"action": "battery"}
            def beep_wrapper(): return {"action": "beep"}
            def drive_wrapper(meters: float): return {"action": "drive", "dist": meters}
            def turn_wrapper(direction: str, degrees: float = 90.0): return {"action": "turn", "dir": direction, "deg": degrees}
            
            def recalibrate_wrapper():
                """
                Spins the robot 360 degrees to fix laser/map alignment.
                Use this if the user says 'Recalibrate', 'Fix map', or 'I am lost'.
                """
                return {"action": "recalibrate"} 
                
            # VISION TOOL (Passive - Assumes camera is already running)
            def vision_wrapper(mode: str):
                """
                Takes a snapshot from the running camera.
                mode: 'describe' (analyze image) or 'capture' (just show it).
                """
                return {"action": "vision", "mode": mode}

            # --- CONFIGURE GEMINI ---
            model = genai.GenerativeModel(
                model_name='gemini-2.0-flash',
                # Removed camera_wrapper/close_camera_wrapper
                tools=[move_smart_wrapper, battery_smart_wrapper, beep_wrapper, drive_wrapper, turn_wrapper, vision_wrapper, recalibrate_wrapper],
                system_instruction="You are a robot assistant. The camera is always on. Use 'vision_wrapper' to see."
            )
            chat = model.start_chat(enable_automatic_function_calling=False)

            response = await chat.send_message_async(user_text)
            final_reply = ""
            
            for part in response.candidates[0].content.parts:
                if part.function_call:
                    fname = part.function_call.name
                    st.toast(f"⚙️ Running: {fname}")
                    
                    # --- RECALIBRATE (The 360 Spin) ---
                    if fname == "recalibrate_wrapper":
                        st.info("🔄 Performing 360° Recalibration Spin...")
                        
                        # 360 degrees (6.28 rad) / 0.5 speed = ~12.5 seconds
                        duration = 13.0 
                        
                        # Spin Left
                        msg_spin = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.5}}
                        # Stop
                        msg_stop = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
                        
                        try:
                            await session.call_tool("publish_for_durations", arguments={
                                "topic": "/cmd_vel",
                                "msg_type": "geometry_msgs/msg/Twist",
                                "messages": [msg_spin, msg_stop],
                                "durations": [duration, 1.0]
                            })
                            tool_out = "Recalibration spin complete."
                        except Exception as e:
                            tool_out = f"Spin Error: {e}"      
                            
                                          
                    # --- NAVIGATION ---
                    if fname == "move_smart_wrapper":
                        room = part.function_call.args['room_name']
                        if room in LOCATIONS:
                            data = LOCATIONS[room]
                            target_x = float(data[0])
                            target_y = float(data[1])
                            target_z = float(data[2]) if len(data) > 2 else 0.0
                            target_w = float(data[3]) if len(data) > 3 else 1.0
                            
                            st.info(f"📍 Moving to {room}...")
                            
                            goal = {
                                'header': {'frame_id': 'map'}, 
                                'pose': {
                                    'position': {'x': target_x, 'y': target_y, 'z': 0.0}, 
                                    'orientation': {'x': 0.0, 'y': 0.0, 'z': target_z, 'w': target_w}
                                }
                            }
                            await session.call_tool("publish_once", arguments={"topic": "/goal_pose", "msg_type": "geometry_msgs/msg/PoseStamped", "msg": goal})
                            tool_out = f"Moving to {room}."
                        else: tool_out = "Unknown room."

                    # --- BEEP ---
                    elif fname == "beep_wrapper":
                        await session.call_tool("publish_once", arguments={"topic": "/beep", "msg_type": "std_msgs/msg/UInt16", "msg": {"data": 200}})
                        tool_out = "Beeped."

                    # --- DRIVE ---
                    elif fname == "drive_wrapper":
                        meters = float(part.function_call.args['meters'])
                        speed = 0.2
                        duration = abs(meters / speed)
                        velocity = speed if meters > 0 else -speed
                        msg_go = {"linear": {"x": velocity, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
                        msg_stop = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
                        await session.call_tool("publish_for_durations", arguments={"topic": "/cmd_vel", "msg_type": "geometry_msgs/msg/Twist", "messages": [msg_go, msg_stop], "durations": [duration, 1.0]})
                        tool_out = "Driven."

                    # --- TURN ---
                    elif fname == "turn_wrapper":
                        direction = part.function_call.args['direction']
                        degrees = float(part.function_call.args.get('degrees', 90.0))
                        speed = 0.5
                        radians = degrees * (3.14159 / 180.0)
                        duration = radians / speed
                        z_speed = speed if direction.lower() == 'left' else -speed
                        msg_spin = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": z_speed}}
                        msg_stop = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
                        await session.call_tool("publish_for_durations", arguments={"topic": "/cmd_vel", "msg_type": "geometry_msgs/msg/Twist", "messages": [msg_spin, msg_stop], "durations": [duration, 1.0]})
                        tool_out = f"Turned {degrees} degrees."

                    # --- BATTERY ---
                    elif fname == "battery_smart_wrapper":
                        try:
                            result = await session.call_tool("subscribe_for_duration", arguments={"topic": "/battery", "msg_type": "std_msgs/msg/UInt16", "duration": 3.0})
                            tool_out = f"Battery Level: {result.content[0].text}"
                        except Exception as e:
                            tool_out = f"Error: {e}"

                    # --- VISION (Snapshot & Analyze) ---
                    elif fname == "vision_wrapper":
                        mode = part.function_call.args.get('mode', 'capture')
                        
                        st.info("📸 Taking the image...")
                        try:
                            # Simply run the snapshot script. 
                            # It assumes the camera is ALREADY running manually.
                            result = subprocess.run(["python3", "utils/take_photo.py"], capture_output=True, text=True, timeout=8)
                            output_text = result.stdout.strip()
                            
                            if "SUCCESS:" in output_text:
                                file_path = output_text.split("SUCCESS:")[1].strip()
                                if os.path.exists(file_path):
                                    img = PILImage.open(file_path)
                                    analysis_text = ""
                                    if mode == "describe":
                                        st.info("🧠 Analyzing image...")
                                        vision_model = genai.GenerativeModel('gemini-2.0-flash')
                                        vision_response = vision_model.generate_content(["Describe this image briefly.", img])
                                        analysis_text = f" I see: {vision_response.text}"
                                    
                                    response_text = f"Snapshot captured.{analysis_text}"
                                    tool_out = response_text
                                    
                                    st.session_state.messages.append({
                                        "role": "assistant", 
                                        "content": response_text,
                                        "image": img
                                    })
                                    return "" 
                                else:
                                    tool_out = "Error: File not found."
                            else:
                                tool_out = f"Snapshot failed: {output_text} (Is the camera window open manually?)"
                        except Exception as e:
                            tool_out = f"Vision Error: {e}"

                    # Send result back
                    func_res = await chat.send_message_async(
                        {"role": "function", "parts": [{"function_response": {"name": fname, "response": {"result": tool_out}}}]}
                    )
                    final_reply = func_res.text
            
            if not final_reply: final_reply = response.text
            return final_reply

# --- INPUT HANDLING ---
if prompt := st.chat_input("Command..."):
    with st.chat_message("user"):
        st.markdown(prompt)
    st.session_state.messages.append({"role": "user", "content": prompt})

    with st.chat_message("assistant"):
        with st.spinner("Processing..."):
            try:
                reply = asyncio.run(process_command(prompt))
                if reply:
                    st.markdown(reply)
                    st.session_state.messages.append({"role": "assistant", "content": reply})
                
                time.sleep(0.5)
                st.rerun()
                
            except Exception as e:
                if "TaskGroup" not in str(e): 
                    st.error(f"Error: {e}")
