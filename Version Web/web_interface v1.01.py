#Update the camera part, havent fix some problem
import streamlit as st
import asyncio
import os
import cv2
import numpy as np
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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

st.set_page_config(page_title="YahBoom AI Vision", page_icon="👁️")
st.title("👁️ YahBoom AI Vision Commander")

# Initialize Chat
if "messages" not in st.session_state:
    st.session_state.messages = []
    st.session_state.messages.append({"role": "assistant", "content": "Visual Systems Online. Ask: 'What do you see?' or 'Take a picture'."})

for message in st.session_state.messages:
    with st.chat_message(message["role"]):
        st.markdown(message["content"])
        # If there is an image saved in history, show it
        if "image" in message:
            st.image(message["image"])

# --- HELPER: GRAB ONE IMAGE FROM ROS ---
def get_snapshot():
    """Connects to ROS, grabs one frame, converts to PIL Image."""
    snapshot = None
    
    # Tiny temporary node
    if not rclpy.ok(): rclpy.init()
    node = Node('snapshot_taker')
    
    def callback(msg):
        nonlocal snapshot
        try:
            # Manual Decode (No cv_bridge crash)
            np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            # Convert BGR (OpenCV) to RGB (Gemini/Screen)
            rgb_frame = cv2.cvtColor(np_arr, cv2.COLOR_BGR2RGB)
            # Flip if needed (YahBoom camera is often upside down)
            rgb_frame = cv2.flip(rgb_frame, 0) 
            snapshot = PILImage.fromarray(rgb_frame)
        except Exception:
            pass

    # Subscribe
    sub = node.create_subscription(Image, '/esp32_img', callback, 10)
    
    # Spin until we get an image or timeout (2 seconds)
    start = time.time()
    while snapshot is None and time.time() - start < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    # Do not shutdown rclpy here, as it might kill other things
    
    return snapshot

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
                    st.toast(f"⚙️ {fname}...")
                    
                    # --- EXISTING NAVIGATION TOOLS ---
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
                        # ... (Your drive logic here) ...
                        tool_out = "Driven."

                    elif fname == "turn_wrapper":
                        # ... (Your turn logic here) ...
                        tool_out = "Turned."

                    # --- NEW CAMERA TOOLS ---
                    elif fname == "camera_wrapper":
                        # Launch the Python script
                        import subprocess
                        env_vars = os.environ.copy()
                        env_vars["DISPLAY"] = ":0"
                        subprocess.Popen(["python3", "/home/yahboom/my_robot_project/smart_camera.py"], env=env_vars, start_new_session=True)
                        tool_out = "Live Video Opened."

                    elif fname == "close_camera_wrapper":
                        # Kill the python script
                        os.system("pkill -f smart_camera.py")
                        tool_out = "Live Video Closed."

                    # --- THE VISION TOOL (FIXED) ---
                    elif fname == "vision_wrapper":
                        mode = part.function_call.args['mode']
                        st.info("📸 Requesting Snapshot...")
                        
                        # 1. Run the external script to save 'snapshot.jpg'
                        import subprocess
                        # We use check_output to verify it finished
                        try:
                            # Run script and wait for it to finish
                            result = subprocess.run(
                                ["python3", "take_photo.py"], 
                                capture_output=True, text=True, timeout=5
                            )
                            
                            # Check if it printed "SUCCESS"
                            if "SUCCESS" in result.stdout:
                                # 2. Load the saved image
                                if os.path.exists("snapshot.jpg"):
                                    img = PILImage.open("snapshot.jpg")
                                    
                                    # Show on Website
                                    st.image(img, caption="Robot View", width=300)
                                    
                                    if mode == "describe":
                                        st.info("🧠 Analyzing...")
                                        vision_model = genai.GenerativeModel('gemini-2.0-flash')
                                        vision_response = vision_model.generate_content(["Describe this image.", img])
                                        tool_out = f"I see: {vision_response.text}"
                                    else:
                                        tool_out = "Picture captured successfully."
                                else:
                                    tool_out = "Error: File snapshot.jpg not found."
                            else:
                                tool_out = f"Error capturing image: {result.stdout} {result.stderr}"
                                
                        except Exception as e:
                            tool_out = f"Vision System Failed: {e}"

                    # Send result back to Gemini
                    func_res = await chat.send_message_async(
                        {"role": "function", "parts": [{"function_response": {"name": fname, "response": {"result": tool_out}}}]}
                    )
                    final_reply = func_res.text
            
            if not final_reply: final_reply = response.text
            return final_reply

# UI Input
if prompt := st.chat_input("Command..."):
    st.chat_message("user").markdown(prompt)
    st.session_state.messages.append({"role": "user", "content": prompt})
    with st.chat_message("assistant"):
        with st.spinner("Processing..."):
            try:
                reply = asyncio.run(process_command(prompt))
                st.markdown(reply)
                st.session_state.messages.append({"role": "assistant", "content": reply})
            except Exception as e:
                # Filter cleanup errors
                if "TaskGroup" not in str(e): st.error(f"Error: {e}")
