import asyncio
import os
import google.generativeai as genai
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

# ==========================================
# 1. CONFIGURATION
# ==========================================
GOOGLE_API_KEY = "Paste your own api key"

# YOUR MAP COORDINATES (Update these with your real numbers from RViz!)
LOCATIONS = {
    "kitchen": [1.5, 0.5],
    "living_room": [0.0, 0.0],
    "bedroom": [2.0, -1.0]
}

# PATH TO THE SUPERVISOR'S SERVER FILE
# Make sure this path is 100% correct
SERVER_SCRIPT = "/home/yahboom/my_robot_project/ros-mcp-server/server.py"
# ==========================================

genai.configure(api_key=GOOGLE_API_KEY)

async def run_client():
    # 1. Launch the Supervisor's Generic Server
    server_params = StdioServerParameters(
        command="python3",
        args=[SERVER_SCRIPT], 
        env=dict(os.environ) # Pass ROS environment variables
    )

    print("Connecting to SUPERVISOR MCP Server...")

    async with stdio_client(server_params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            
            # 2. Verify Connection
            tools_list = await session.list_tools()
            print(f"✅ Connected! Server offers {len(tools_list.tools)} tools.")

            # ==================================================
            # 3. DEFINE "SMART WRAPPERS" FOR GEMINI
            # ==================================================

            def move_smart_wrapper(room_name: str):
                """Moves the robot to a room."""
                return {"action": "move", "room": room_name}

            def battery_smart_wrapper():
                """Checks battery voltage."""
                return {"action": "battery"}

            # 4. Configure Gemini
            model = genai.GenerativeModel(
                model_name='gemini-2.0-flash',
                tools=[move_smart_wrapper, battery_smart_wrapper],
                system_instruction="You are a robot assistant. Use the tools provided."
            )

            chat = model.start_chat(enable_automatic_function_calling=False)
            print("\nSYSTEM READY. Ask: 'Go to kitchen' or 'Check battery'.")

            # 5. Main Loop
            while True:
                user_input = input("YOU: ")
                if user_input.lower() in ['exit', 'quit']: break
                
                print("ROBOT: Thinking...")
                response = await chat.send_message_async(user_input)
                
                final_text = ""
                function_called = False

                # Check if Gemini called a tool
                for part in response.candidates[0].content.parts:
                    if part.function_call:
                        function_called = True
                        fname = part.function_call.name
                        print(f"⚡ Gemini called '{fname}'...")
                        
                        # --- A. HANDLE MOVE COMMAND ---
                        if fname == "move_smart_wrapper":
                            room = part.function_call.args['room_name']
                            if room in LOCATIONS:
                                coords = LOCATIONS[room]
                                print(f"⚡ Translating '{room}' -> ROS 2 Action...")
                                
                                # Construct the complex JSON for /navigate_to_pose
                                goal_msg = {
                                    'pose': {
                                        'header': {'frame_id': 'map'},
                                        'pose': {
                                            'position': {'x': float(coords[0]), 'y': float(coords[1]), 'z': 0.0},
                                            'orientation': {'w': 1.0}
                                        }
                                    }
                                }
                                
                                # Call the Generic Tool
                                result = await session.call_tool("send_action_goal", arguments={
                                    "action_name": "/navigate_to_pose",
                                    "action_type": "nav2_msgs/action/NavigateToPose",
                                    "goal": goal_msg
                                })
                                tool_output = f"Navigation started. ID: {result.content[0].text}"
                            else:
                                tool_output = "Error: Unknown room name."

                            # Send result back
                            func_response = await chat.send_message_async(
                                {"role": "function", "parts": [{"function_response": {"name": fname, "response": {"result": tool_output}}}]}
                            )
                            final_text = func_response.text

                        # --- B. HANDLE BATTERY COMMAND ---
                        elif fname == "battery_smart_wrapper":
                            print(f"⚡ Fetching /battery (Listening for 3 seconds)...")
                            
                            try:
                                # USE A DIFFERENT TOOL: 'subscribe_for_duration'
                                # This forces the server to listen for X seconds
                                result = await session.call_tool("subscribe_for_duration", arguments={
                                    "topic": "/battery",
                                    "msg_type": "std_msgs/msg/UInt16",
                                    "duration": 3   # Listen for 3 seconds
                                })
                                tool_output = f"Raw Data: {result.content[0].text}"
                            except Exception as e:
                                tool_output = f"Error reading battery: {e}"
                            
                            # Send result back
                            func_response = await chat.send_message_async(
                                {"role": "function", "parts": [{"function_response": {"name": fname, "response": {"result": tool_output}}}]}
                            )
                            
                            # Logic to handle Gemini's reply
                            if func_response.candidates[0].content.parts[0].text:
                                final_text = func_response.text
                            else:
                                final_text = "Battery data received."

                if not function_called:
                    final_text = response.text

                print(f"AI: {final_text}")

if __name__ == "__main__":
    asyncio.run(run_client())
