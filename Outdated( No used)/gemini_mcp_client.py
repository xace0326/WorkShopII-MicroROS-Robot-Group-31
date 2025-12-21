import asyncio
import os
import google.generativeai as genai
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

# ==========================================
# CONFIGURATION
# ==========================================
GOOGLE_API_KEY = "paste your own api key"
# MAKE SURE THIS PATH IS CORRECT!
SERVER_PATH = "/home/yahboom/my_robot_project/robot_mcp_server.py"
# ==========================================

genai.configure(api_key=GOOGLE_API_KEY)

async def run_client():
    # 1. Launch the Server with OS Environment (Fixes ROS2 issues)
    server_params = StdioServerParameters(
        command="python3",
        args=[SERVER_PATH],
        env=dict(os.environ)
    )

    print("Connecting to Robot MCP Server...")

    # 2. Connect via Standard IO
    async with stdio_client(server_params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            
            # 3. List Tools
            tools_result = await session.list_tools()
            print(f"Connected! Server offers: {[t.name for t in tools_result.tools]}")

            # 4. Define Wrappers (The Bridge)
            def move_wrapper(room_name: str):
                """Moves robot to a room."""
                return {"mcp_action": "move_robot", "args": {"room_name": room_name}}

            def list_wrapper():
                """Lists rooms."""
                return {"mcp_action": "list_rooms", "args": {}}

            def camera_smart_wrapper():
                """Centers and opens the camera for 15 seconds."""
                return {"mcp_action": "open_camera_smart", "args": {}}

            # 5. Configure Gemini
            model = genai.GenerativeModel(
                model_name='gemini-2.0-flash',
                # IMPORTANT: All 3 tools must be here
                tools=[move_wrapper, list_wrapper, camera_smart_wrapper],
                system_instruction="You are a robot assistant. Use the tools provided to control the robot."
            )

            chat = model.start_chat(enable_automatic_function_calling=False)
            print("\nSYSTEM READY. Ask: 'Open the camera'.")

            # 6. Main Loop
            while True:
                user_input = input("YOU: ")
                if user_input.lower() in ['exit', 'quit']: break
                
                print("ROBOT: Thinking...")
                
                response = await chat.send_message_async(user_input)
                final_text = ""
                function_called = False
                
                # Check history for function calls
                for part in response.candidates[0].content.parts:
                    if part.function_call:
                        function_called = True
                        fname = part.function_call.name
                        print(f"⚡ Gemini called '{fname}'...")
                        
                        # --- LOGIC BLOCK START ---
                        if fname == "move_wrapper":
                            room = part.function_call.args['room_name']
                            print(f"⚡ Bridging to MCP: Moving to {room}...")
                            result = await session.call_tool("move_robot", arguments={"room_name": room})
                            tool_output = result.content[0].text
                            
                            # Send result back
                            func_response = await chat.send_message_async(
                                {"role": "function", "parts": [{"function_response": {"name": fname, "response": {"result": tool_output}}}]}
                            )
                            final_text = func_response.text
                            
                        elif fname == "list_wrapper":
                            result = await session.call_tool("list_rooms")
                            tool_output = result.content[0].text
                            func_response = await chat.send_message_async(
                                {"role": "function", "parts": [{"function_response": {"name": fname, "response": {"result": tool_output}}}]}
                            )
                            final_text = func_response.text

                        # --- THIS WAS MISSING BEFORE! ---
                        elif fname == "camera_smart_wrapper":
                            print(f"⚡ Bridging to MCP: Auto-Centering Camera...")
                            
                            # Call the Server
                            result = await session.call_tool("open_camera_smart")
                            tool_output = result.content[0].text
                            print(f"⚡ Server Result: {tool_output}")

                            # Send result back
                            func_response = await chat.send_message_async(
                                {"role": "function", "parts": [{"function_response": {"name": fname, "response": {"result": tool_output}}}]}
                            )
                            final_text = func_response.text
                        # --------------------------------

                if not function_called:
                    final_text = response.text

                print(f"AI: {final_text}")

if __name__ == "__main__":
    asyncio.run(run_client())
