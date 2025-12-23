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

LOCATIONS = {
    "kitchen": [1.5, 0.5],
    "living_room": [0.0, 0.0],
    "bedroom": [2.0, -1.0]
}
# ==========================================

st.set_page_config(page_title="YahBoom AI Vision", page_icon="👁️", layout="wide")

# --- SIDEBAR (HISTORY CONTROL) ---
with st.sidebar:
    st.title("Control Panel")
    if st.button("🗑️ Clear Chat History", type="primary"):
        st.session_state.messages = []
        st.session_state.messages.append({"role": "assistant", "content": "Memory cleared. Systems ready."})
        st.rerun()

st.title("👁️ YahBoom AI Vision Commander")

# Initialize Chat
if "messages" not in st.session_state:
    st.session_state.messages = []
    st.session_state.messages.append({"role": "assistant", "content": "Visual Systems Online. Ask: 'What do you see?', 'Take a picture', or 'Open Camera'."})

# Display Chat History (This runs every time the script refreshes)
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
            def turn_wrapper(direction: str): return {"action": "turn", "dir": direction}
            
            def camera_wrapper(): 
                """Opens the live video window."""
                return {"action": "launch_camera"}
            
            def close_camera_wrapper():
                """Closes the live video window."""
                return {"action": "kill_camera"}

            def vision_wrapper(mode: str):
                """
                Used when user asks to 'see', 'look', 'take picture', or 'describe'.
                mode: 'describe' (analyze image) or 'capture' (just show it).
                """
                return {"action": "vision", "mode": mode}

            # --- CONFIGURE GEMINI ---
            model = genai.GenerativeModel(
                model_name='gemini-2.0-flash',
                tools=[move_smart_wrapper, battery_smart_wrapper, beep_wrapper, drive_wrapper, turn_wrapper, camera_wrapper, close_camera_wrapper, vision_wrapper],
                system_instruction="You are a robot with eyes. Use vision_wrapper to see."
            )
            chat = model.start_chat(enable_automatic_function_calling=False)

            response = await chat.send_message_async(user_text)
            final_reply = ""
            
            for part in response.candidates[0].content.parts:
                if part.function_call:
                    fname = part.function_call.name
                    st.toast(f"⚙️ Running: {fname}")
                    
                    # --- NAVIGATION ---
                    if fname == "move_smart_wrapper":
                        room = part.function_call.args['room_name']
                        if room in LOCATIONS:
                            coords = LOCATIONS[room]
                            goal = {'pose': {'header': {'frame_id': 'map'}, 'pose': {'position': {'x': float(coords[0]), 'y': float(coords[1]), 'z': 0.0}, 'orientation': {'w': 1.0}}}}
                            await session.call_tool("send_action_goal", arguments={"action_name": "/navigate_to_pose", "action_type": "nav2_msgs/action/NavigateToPose", "goal": goal})
                            tool_out = f"Moving to {room}."
                        else: tool_out = "Unknown room."
                    
                    elif fname == "beep_wrapper":
                        await session.call_tool("publish_once", arguments={"topic": "/beep", "msg_type": "std_msgs/msg/UInt16", "msg": {"data": 200}})
                        tool_out = "Beeped."

                    elif fname == "drive_wrapper":
                        meters = float(part.function_call.args['meters'])
                        speed = 0.2
                        duration = abs(meters / speed)
                        velocity = speed if meters > 0 else -speed
                        msg_go = {"linear": {"x": velocity, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
                        msg_stop = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
                        await session.call_tool("publish_for_durations", arguments={"topic": "/cmd_vel", "msg_type": "geometry_msgs/msg/Twist", "messages": [msg_go, msg_stop], "durations": [duration, 1.0]})
                        tool_out = "Driven."

                    elif fname == "turn_wrapper":
                        direction = part.function_call.args['direction']
                        speed = 0.5
                        msg_spin = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": speed if direction == 'left' else -speed}}
                        msg_stop = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
                        await session.call_tool("publish_for_durations", arguments={"topic": "/cmd_vel", "msg_type": "geometry_msgs/msg/Twist", "messages": [msg_spin, msg_stop], "durations": [3.0, 1.0]})
                        tool_out = "Turned."

                    # --- CAMERA CONTROL ---
                    elif fname == "camera_wrapper":
                        env_vars = os.environ.copy()
                        env_vars["DISPLAY"] = ":0"
                        subprocess.Popen(["python3", "/home/yahboom/my_robot_project/smart_camera.py"], env=env_vars, start_new_session=True)
                        tool_out = "Live Video Opened."

                    elif fname == "close_camera_wrapper":
                        os.system("pkill -f smart_camera.py")
                        tool_out = "Live Video Closed."

                    # --- VISION (Snapshot) ---
                    elif fname == "vision_wrapper":
                        mode = part.function_call.args.get('mode', 'capture')
                        
                        # NO AUTO-CLOSE. Parallel run.
                        st.info("📸 Taking picture...")
                        
                        try:
                            result = subprocess.run(
                                ["python3", "take_photo.py"], 
                                capture_output=True, text=True, timeout=8
                            )
                            
                            output_text = result.stdout.strip()
                            
                            if "SUCCESS:" in output_text:
                                file_path = output_text.split("SUCCESS:")[1].strip()
                                
                                if os.path.exists(file_path):
                                    img = PILImage.open(file_path)
                                    # We don't use st.image here directly anymore to avoid "Ghosting"
                                    # We save it to session state immediately below
                                    
                                    analysis_text = ""
                                    if mode == "describe":
                                        st.info("🧠 Analyzing image...")
                                        vision_model = genai.GenerativeModel('gemini-2.0-flash')
                                        vision_response = vision_model.generate_content(["Describe this image briefly.", img])
                                        analysis_text = f" I see: {vision_response.text}"
                                    
                                    response_text = f"Snapshot captured.{analysis_text}"
                                    tool_out = response_text
                                    
                                    # Add to history NOW so it appears after rerun
                                    st.session_state.messages.append({
                                        "role": "assistant", 
                                        "content": response_text,
                                        "image": img
                                    })
                                    # Return empty string to prevent double-printing text in the loop below
                                    return "" 
                                else:
                                    tool_out = "Error: File not found."
                            else:
                                tool_out = f"Camera failed: {output_text}"
                                
                        except Exception as e:
                            tool_out = f"Vision Error: {e}"

                    # Send result back
                    func_res = await chat.send_message_async(
                        {"role": "function", "parts": [{"function_response": {"name": fname, "response": {"result": tool_out}}}]}
                    )
                    final_reply = func_res.text
            
            if not final_reply: final_reply = response.text
            return final_reply

# --- USER INPUT HANDLING ---
if prompt := st.chat_input("Command..."):
    # 1. Show User Message
    with st.chat_message("user"):
        st.markdown(prompt)
    st.session_state.messages.append({"role": "user", "content": prompt})

    # 2. Process & Show Assistant Message
    with st.chat_message("assistant"):
        with st.spinner("Processing..."):
            try:
                reply = asyncio.run(process_command(prompt))
                
                # Only append if reply is not empty (Vision tool handles its own append)
                if reply:
                    st.markdown(reply)
                    st.session_state.messages.append({"role": "assistant", "content": reply})
                
                # --- THE GHOST TEXT FIX ---
                # This forces the page to reload immediately.
                # The chat history will be redrawn from 'st.session_state.messages',
                # making the font solid and permanent.
                st.rerun()
                
            except Exception as e:
                # Filter cleanup errors
                if "TaskGroup" not in str(e): 
                    st.error(f"Error: {e}")
