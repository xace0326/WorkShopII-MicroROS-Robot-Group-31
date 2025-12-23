import asyncio
import os
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

# --- UPDATE YOUR COORDINATES HERE ---
TEST_X = -0.4692
TEST_Y = -0.5199
# ------------------------------------

async def test_navigation():
    print("--- TESTING MCP NAVIGATION ---")
    
    # 1. Launch Server
    server_params = StdioServerParameters(
        command="python3",
        args=["/home/yahboom/my_robot_project/ros-mcp-server/server.py"], 
        env=dict(os.environ)
    )

    try:
        async with stdio_client(server_params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                print("✅ Connected to Server.")
                
                # 2. Prepare the Goal Message (This is the tricky part!)
                print(f"🚀 Sending Robot to X={TEST_X}, Y={TEST_Y}...")
                
                goal_msg = {
                    'pose': {
                        'header': {
                            'frame_id': 'map'
                            # 'stamp': We usually let ROS handle current time if omitted
                        },
                        'pose': {
                            'position': {
                                'x': float(TEST_X), 
                                'y': float(TEST_Y), 
                                'z': 0.0
                            },
                            'orientation': {
                                'x': 0.0, 
                                'y': 0.0, 
                                'z': 0.0, 
                                'w': 1.0
                            }
                        }
                    }
                }

                # 3. Call the Tool Directly
                result = await session.call_tool("send_action_goal", arguments={
                    "action_name": "/navigate_to_pose",
                    "action_type": "nav2_msgs/action/NavigateToPose",
                    "goal": goal_msg
                })
                
                # 4. Print Result
                print("--- RESULT FROM SERVER ---")
                print(result.content[0].text)
                print("--------------------------")
                print("👀 CHECK YOUR ROBOT NOW. IS IT MOVING?")

    except Exception as e:
        print(f"\n❌ TEST FAILED.")
        print(f"Error Message: {e}")

if __name__ == "__main__":
    asyncio.run(test_navigation())
