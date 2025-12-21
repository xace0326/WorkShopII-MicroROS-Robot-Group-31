import asyncio
import os
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

async def list_tools():
    print("--- INSPECTING ROS-MCP-SERVER ---")
    
    # 1. Launch their server using the module command
    # The command is: python3 -m ros_mcp_server
# USE ABSOLUTE FILE PATH (Guaranteed to work)
    server_params = StdioServerParameters(
        command="python3",
        args=["/home/yahboom/my_robot_project/ros-mcp-server/server.py"], 
        env=dict(os.environ) 
    )

    try:
        async with stdio_client(server_params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                
                # 2. Ask for tool list
                print("Connected. Fetching tools...")
                tools_result = await session.list_tools()
                
                print(f"\n✅ FOUND {len(tools_result.tools)} TOOLS:")
                print("="*40)
                
                for t in tools_result.tools:
                    print(f"🔹 {t.name}")
                    print(f"   {t.description}")
                    print("-" * 20)
                    
    except Exception as e:
        print(f"Error: {e}")
        print("Did you source ROS? (source ~/yahboomcar_ws/install/setup.bash)")

if __name__ == "__main__":
    asyncio.run(list_tools())
