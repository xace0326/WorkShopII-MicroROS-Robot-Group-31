import streamlit as st
import asyncio
import os
import cv2
import numpy as np
import time
import subprocess
import base64
import io
from PIL import Image as PILImage
import google.generativeai as genai
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

# --- AUDIO LIBRARIES ---
from streamlit_mic_recorder import mic_recorder
import speech_recognition as sr
from pydub import AudioSegment
from gtts import gTTS

# ==========================================
# CONFIGURATION
# ==========================================
GOOGLE_API_KEY = "PASTE_YOUR_API_KEY_HERE"
SERVER_SCRIPT = "/home/yahboom/my_robot_project/ros-mcp-server/server.py"
AUDIO_FILE = "temp_audio.mp3"

LOCATIONS = {
    "kitchen": [-1.086018833517747, -0.209371012817268, -0.731727801097563, 0.6815969667632957],
    "Kitchen": [-1.086018833517747, --0.209371012817268, -0.731727801097563, 0.6815969667632957],
    "living room": [-1.2645019451231838, 0.0839337039745333, 0.999554843207999, 0.029834802151724724],
    "bedroom": [-0.24549463752807524, -0.34738760127829604, -0.7072734826030969, 0.7069400404606369],
    "home": [-0.18609893770631955, 0.060271492090862654, 0.9997587036528459, 0.021966667257031048]
}
# ==========================================

st.set_page_config(page_title="MicroRos Robot", page_icon="🤖️", layout="wide")

# --- CSS: FLOATING MIC BUTTON ---
st.markdown("""
<style>
    div.stButton > button { width: 100%; border-radius: 5px; }
    div[data-testid="stColumn"] > div > div > div > div > div > button {
        border-radius: 50%; height: 3rem; width: 3rem; padding: 0; float: right;
    }
</style>
""", unsafe_allow_html=True)

# --- HELPER: PLAY AUDIO HIDDEN ---
def autoplay_audio(file_path):
    with open(file_path, "rb") as f:
        data = f.read()
    b64 = base64.b64encode(data).decode()
    md = f"""
        <audio autoplay="true" style="display:none;">
        <source src="data:audio/mp3;base64,{b64}" type="audio/mp3">
        </audio>
        """
    st.markdown(md, unsafe_allow_html=True)

# --- HELPER: GET AUDIO LENGTH ---
def get_audio_duration(file_path):
    try:
        audio = AudioSegment.from_mp3(file_path)
        return len(audio) / 1000.0
    except: return 2.0

# --- HELPER: AUDIO TO TEXT ---
def recognize_audio(audio_bytes):
    r = sr.Recognizer()
    try:
        audio_data = io.BytesIO(audio_bytes)
        audio_segment = AudioSegment.from_file(audio_data)
        wav_data = io.BytesIO()
        audio_segment.export(wav_data, format="wav")
        wav_data.seek(0)
        with sr.AudioFile(wav_data) as source:
            audio_content = r.record(source)
            text = r.recognize_google(audio_content)
            return text
    except Exception as e: return f"Error: {e}"

# --- HELPER: TEXT TO SPEECH ---
def generate_speech(text):
    try:
        # British accent (co.uk) makes it sound cooler!
        tts = gTTS(text=text, lang='en', tld='co.uk')
        tts.save(AUDIO_FILE)
        return True
    except Exception as e: return False

# --- SIDEBAR ---
with st.sidebar:
    st.title("Control Panel")
    if st.button("🗑️ Clear Chat History", type="primary", use_container_width=True):
        st.session_state.messages = []
        st.session_state.messages.append({"role": "assistant", "content": "System Ready."})
        st.rerun()

st.title("🤖️ MicroRos Robot")

if "messages" not in st.session_state:
    st.session_state.messages = [{"role": "assistant", "content": "Online."}]

for message in st.session_state.messages:
    with st.chat_message(message["role"]):
        st.markdown(message["content"])
        if "image" in message:
            st.image(message["image"], width=400)

# --- MAIN AI LOGIC ---
async def process_command(user_text, is_voice_mode):
    genai.configure(api_key=GOOGLE_API_KEY)
    server_params = StdioServerParameters(command="python3", args=[SERVER_SCRIPT], env=dict(os.environ))

    async with stdio_client(server_params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            
            # TOOLS DEFINITIONS
            def move_smart_wrapper(room_name: str): return {"action": "move", "room": room_name}
            def battery_smart_wrapper(): return {"action": "battery"}
            def beep_wrapper(): return {"action": "beep"}
            def drive_wrapper(meters: float): return {"action": "drive", "dist": meters}
            def turn_wrapper(direction: str, degrees: float = 90.0): return {"action": "turn", "dir": direction, "deg": degrees}
            def recalibrate_wrapper(): return {"action": "recalibrate"} 
            
            def vision_wrapper(mode: str): 
                """
                Takes a snapshot. 
                mode: 'describe' (analyze image) or 'capture' (just show it).
                """
                return {"action": "vision", "mode": mode}

            # --- SYSTEM INSTRUCTION (THIS IS THE FIX) ---
            # We explicitly teach Gemini how to use the vision tool
            system_prompt = """
            You are a robot assistant.
            
            VISION RULES:
            1. If user says "What do you see?" or "Describe view", USE 'vision_wrapper' with mode='describe'.
            2. If user says "Take picture" or "Snapshot", USE 'vision_wrapper' with mode='capture'.
            3. DO NOT ask the user for the mode. Just do it.
            """

            model = genai.GenerativeModel(
                model_name='gemini-2.0-flash',
                tools=[move_smart_wrapper, battery_smart_wrapper, beep_wrapper, drive_wrapper, turn_wrapper, vision_wrapper, recalibrate_wrapper],
                system_instruction=system_prompt
            )
            chat = model.start_chat(enable_automatic_function_calling=False)
            response = await chat.send_message_async(user_text)
            final_reply = ""
            
            for part in response.candidates[0].content.parts:
                if part.function_call:
                    fname = part.function_call.name
                    st.toast(f"⚙️ {fname}")
                    tool_out = "Done."
                    
                    if fname == "move_smart_wrapper":
                        room = part.function_call.args['room_name']
                        if room in LOCATIONS:
                            data = LOCATIONS[room]
                            goal = {'header': {'frame_id': 'map'}, 'pose': {'position': {'x': float(data[0]), 'y': float(data[1]), 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': float(data[2]) if len(data)>2 else 0.0, 'w': float(data[3]) if len(data)>3 else 1.0}}}
                            await session.call_tool("publish_once", arguments={"topic": "/goal_pose", "msg_type": "geometry_msgs/msg/PoseStamped", "msg": goal})
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
                        degrees = float(part.function_call.args.get('degrees', 90.0))
                        speed = 0.5
                        duration = (degrees * 3.14159 / 180.0) / speed
                        z_speed = speed if direction.lower() == 'left' else -speed
                        msg_spin = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": z_speed}}
                        msg_stop = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
                        await session.call_tool("publish_for_durations", arguments={"topic": "/cmd_vel", "msg_type": "geometry_msgs/msg/Twist", "messages": [msg_spin, msg_stop], "durations": [duration, 1.0]})
                        tool_out = f"Turned {degrees} degrees."
                    elif fname == "battery_smart_wrapper":
                        try:
                            result = await session.call_tool("subscribe_for_duration", arguments={"topic": "/battery", "msg_type": "std_msgs/msg/UInt16", "duration": 3.0})
                            tool_out = f"Battery: {result.content[0].text}"
                        except: tool_out = "Error."
                    elif fname == "recalibrate_wrapper":
                        duration = 13.0
                        msg_spin = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.5}}
                        msg_stop = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
                        try:
                            await session.call_tool("publish_for_durations", arguments={"topic": "/cmd_vel", "msg_type": "geometry_msgs/msg/Twist", "messages": [msg_spin, msg_stop], "durations": [duration, 1.0]})
                            tool_out = "Recalibrated."
                        except: tool_out = "Error."
                    
                    # --- VISION ---
                    elif fname == "vision_wrapper":
                        mode = part.function_call.args.get('mode', 'capture')
                        st.info("📸 Taking picture...")
                        try:
                            result = subprocess.run(["python3", "utils/take_photo.py"], capture_output=True, text=True, timeout=8)
                            output_text = result.stdout.strip()
                            if "SUCCESS:" in output_text:
                                file_path = output_text.split("SUCCESS:")[1].strip()
                                if os.path.exists(file_path):
                                    img = PILImage.open(file_path)
                                    analysis_text = ""
                                    
                                    if mode == "describe":
                                        st.info("🧠 Analyzing...")
                                        vision_model = genai.GenerativeModel('gemini-2.0-flash')
                                        vision_response = vision_model.generate_content(["Describe exactly what you see.", img])
                                        analysis_text = f" {vision_response.text}"
                                        
                                    tool_out = f"Snapshot captured.{analysis_text}"
                                    
                                    # Add to history
                                    st.session_state.messages.append({"role": "assistant", "content": tool_out, "image": img})
                                    return "" 
                                else: tool_out = "File not found."
                            else: tool_out = "Camera failed."
                        except Exception as e: tool_out = f"Error: {e}"

                    func_res = await chat.send_message_async({"role": "function", "parts": [{"function_response": {"name": fname, "response": {"result": tool_out}}}]})
                    final_reply = func_res.text
            
            if not final_reply: final_reply = response.text
            
            if is_voice_mode:
                generate_speech(final_reply)
            
            return final_reply

# --- INPUT HANDLING ---
col1, col2 = st.columns([9, 1])

with col2:
    audio = mic_recorder(start_prompt="🎤", stop_prompt="⏹️", just_once=True, key='recorder')

with col1:
    text_input = st.chat_input("Command...")

input_text = None
is_voice = False

if audio:
    with st.spinner("Translating voice..."):
        transcribed = recognize_audio(audio['bytes'])
        if "Error" not in transcribed:
            input_text = transcribed
            is_voice = True
        else:
            st.error("Could not understand audio. Try again.")

elif text_input:
    input_text = text_input
    is_voice = False

if input_text:
    with st.chat_message("user"):
        st.markdown(input_text)
    st.session_state.messages.append({"role": "user", "content": input_text})

    with st.chat_message("assistant"):
        with st.spinner("Thinking..."):
            try:
                reply = asyncio.run(process_command(input_text, is_voice))
                if reply:
                    st.markdown(reply)
                    st.session_state.messages.append({"role": "assistant", "content": reply})
                    
                    if is_voice and os.path.exists(AUDIO_FILE):
                        autoplay_audio(AUDIO_FILE)
                        duration = get_audio_duration(AUDIO_FILE)
                        time.sleep(duration + 1)
                
                st.rerun()
            except Exception as e:
                if "TaskGroup" not in str(e): st.error(f"Error: {e}")
