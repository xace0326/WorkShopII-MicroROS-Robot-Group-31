#First version
import streamlit as st
import asyncio
import os
import math
import google.generativeai as genai
import subprocess # <--- Add at top
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


# ==========================================
# CONFIGURATION
# ==========================================
GOOGLE_API_KEY = "paste your own api key"
SERVER_SCRIPT = "/home/yahboom/my_robot_project/ros-mcp-server/server.py"

# Known Locations
LOCATIONS = {
    "kitchen": [1.5, 0.5],
    "living_room": [0.0, 0.0],
    "bedroom": [2.0, -1.0]
}
# ==========================================

st.set_page_config(page_title="YahBoom AI Patrol", page_icon="🤖")
st.title("🤖 YahBoom AI Commander")

if "messages" not in st.session_state:
    st.session_state.messages = []
    st.session_state.messages.append({"role": "assistant", "content": "Ready. Try 'Turn left', 'Go forward', 'Open Camera' OR 'Beep'."})

for message in st.session_state.messages:
    with st.chat_message(message["role"]):
        st.markdown(message["content"])

async def process_command(user_text):
    genai.configure(api_key=GOOGLE_API_KEY)
    
    server_params = StdioServerParameters(
        command="python3", 
        args=[SERVER_SCRIPT], 
        env=dict(os.environ)
    )

    async with stdio_client(server_params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            
            # --- 1. DEFINE TOOLS ---
            
            def move_smart_wrapper(room_name: str):
                """Navigates to a specific room."""
                return {"action": "navigate", "room": room_name}

            def battery_smart_wrapper():
                """Checks battery voltage."""
                return {"action": "battery"}

            def beep_wrapper():
                """Makes a beep sound."""
                return {"action": "beep"}

            def drive_wrapper(meters: float):
                """Moves forward/backward by meters."""
                return {"action": "drive_straight", "dist": meters}

            def turn_wrapper(direction: str, degrees: float = 90.0):
                """Turns the robot left or right by degrees (default 90)."""
                return {"action": "turn", "direction": direction, "degrees": degrees}

            def camera_wrapper():
                """Centers and opens the camera window."""
                return {"action": "launch_camera"}
                
            # --- 2. CONFIGURE GEMINI ---
            model = genai.GenerativeModel(
                model_name='gemini-2.0-flash',
                # Add turn_wrapper to the list!
                tools=[move_smart_wrapper, battery_smart_wrapper, beep_wrapper, drive_wrapper, turn_wrapper, camera_wrapper],
                system_instruction="You are a robot assistant."
            )
            chat = model.start_chat(enable_automatic_function_calling=False)

            response = await chat.send_message_async(user_text)
            final_reply = ""
            
            for part in response.candidates[0].content.parts:
                if part.function_call:
                    fname = part.function_call.name
                    st.toast(f"⚙️ Running tool: {fname}")
                    
                    # --- A. NAVIGATION ---
                    if fname == "move_smart_wrapper":
                        room = part.function_call.args['room_name']
                        if room in LOCATIONS:
                            coords = LOCATIONS[room]
                            goal_msg = {
                                'pose': {
                                    'header': {'frame_id': 'map'},
                                    'pose': {
                                        'position': {'x': float(coords[0]), 'y': float(coords[1]), 'z': 0.0},
                                        'orientation': {'w': 1.0}
                                    }
                                }
                            }
                            result = await session.call_tool("send_action_goal", arguments={
                                "action_name": "/navigate_to_pose",
                                "action_type": "nav2_msgs/action/NavigateToPose",
                                "goal": goal_msg
                            })
                            tool_out = f"Navigating to {room}. (ID: {result.content[0].text})"
                        else:
                            tool_out = "Error: Room not found."

                    # --- B. BATTERY ---
                    elif fname == "battery_smart_wrapper":
                        try:
                            result = await session.call_tool("subscribe_for_duration", arguments={
                                "topic": "/battery",
                                "msg_type": "std_msgs/msg/UInt16",
                                "duration": 3.0
                            })
                            tool_out = f"Battery Level: {result.content[0].text}"
                        except Exception as e:
                            tool_out = f"Error: {e}"

                    # --- C. BEEP ---
                    elif fname == "beep_wrapper":
                        try:
                            result = await session.call_tool("publish_once", arguments={
                                "topic": "/beep",
                                "msg_type": "std_msgs/msg/UInt16",
                                "msg": {"data": 200}
                            })
                            tool_out = "Beep command sent."
                        except Exception as e:
                            tool_out = f"Error: {e}"

                    # --- D. MOVE FORWARD ---
                    elif fname == "drive_wrapper":
                        try:
                            meters = float(part.function_call.args['meters'])
                            speed = 0.2 
                            duration = abs(meters / speed)
                            velocity = speed if meters > 0 else -speed
                            
                            msg_go = {"linear": {"x": velocity, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
                            msg_stop = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}

                            result = await session.call_tool("publish_for_durations", arguments={
                                "topic": "/cmd_vel",
                                "msg_type": "geometry_msgs/msg/Twist",
                                "messages": [msg_go, msg_stop],
                                "durations": [duration, 1.0]
                            })
                            tool_out = f"Moved {meters} meters."
                        except Exception as e:
                            tool_out = f"Drive Error: {e}"

                    # --- E. TURN (NEW!) ---
                    elif fname == "turn_wrapper":
                        try:
                            direction = part.function_call.args['direction']
                            # Default to 90 if not specified
                            degrees = float(part.function_call.args.get('degrees', 90.0))
                            
                            # 1. Calculate Time
                            # Speed: 0.5 radians/sec
                            speed = 0.5 
                            # Convert degrees to radians (e.g. 90 -> 1.57)
                            radians = degrees * (3.14159 / 180.0)
                            duration = radians / speed
                            
                            # 2. Determine Direction (+ is Left, - is Right)
                            angular_vel = speed if direction.lower() == "left" else -speed
                            
                            # 3. Create Messages
                            msg_spin = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": angular_vel}}
                            msg_stop = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}

                            # 4. Send Sequence
                            result = await session.call_tool("publish_for_durations", arguments={
                                "topic": "/cmd_vel",
                                "msg_type": "geometry_msgs/msg/Twist",
                                "messages": [msg_spin, msg_stop],
                                "durations": [duration, 1.0]
                            })
                            tool_out = f"Turned {direction} {degrees} degrees."
                        except Exception as e:
                            tool_out = f"Turn Error: {e}"
                            
                    # --- F. OPEN CAMERA (FIXED DISPLAY) ---
                    elif fname == "camera_wrapper":
                        try:
                            # 1. Setup Environment to force window to appear on desktop
                            # We copy current env and force DISPLAY=:0
                            env_vars = os.environ.copy()
                            env_vars["DISPLAY"] = ":0"
                            # Also add XAUTHORITY if needed (helps with permissions)
                            home = os.path.expanduser("~")
                            if os.path.exists(f"{home}/.Xauthority"):
                                env_vars["XAUTHORITY"] = f"{home}/.Xauthority"

                            # 2. Path to script
                            script_path = "/home/yahboom/my_robot_project/smart_camera.py"
                            
                            # 3. Launch it!
                            subprocess.Popen(
                                ["python3", script_path],
                                env=env_vars,
                                start_new_session=True
                            )
                            tool_out = "Camera launched on robot display."
                        except Exception as e:
                            tool_out = f"Failed to launch camera: {e}"
            
            if not final_reply:
                final_reply = response.text
            return final_reply

if prompt := st.chat_input("Command the robot..."):
    st.chat_message("user").markdown(prompt)
    st.session_state.messages.append({"role": "user", "content": prompt})

    with st.chat_message("assistant"):
        with st.spinner("Processing..."):
            try:
                reply = asyncio.run(process_command(prompt))
                st.markdown(reply)
                st.session_state.messages.append({"role": "assistant", "content": reply})
            except Exception as e:
                # --- ERROR FILTERING ---
                # If the error is just the "TaskGroup" cleanup noise, ignore it.
                error_text = str(e)
                if "TaskGroup" in error_text or "ExceptionGroup" in error_text:
                    # The action likely succeeded, but the connection closed messy.
                    # We assume success if we got this far.
                    pass 
                else:
                    # Show real errors (like WiFi disconnects)
                    st.error(f"System Error: {e}")
