import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from mcp.server.fastmcp import FastMCP
import threading
import time

# ==========================================
# 1. CONFIGURATION (UPDATE YOUR COORDINATES!)
# ==========================================
LOCATIONS = {
    "kitchen": [4.56, 0.25],     # Update these!
    "living_room": [2.56, 2.67],
    "bedroom": [0.13, 0.44]
}

# ==========================================
# 2. SETUP ROS 2 IN BACKGROUND
# ==========================================
# We need ROS to run in a separate thread so it doesn't block the MCP server
navigator = None

def init_ros():
    global navigator
    rclpy.init()
    navigator = BasicNavigator()
    print("--- ROS 2 NAVIGATION WARMING UP ---")
    navigator.waitUntilNav2Active()
    print("--- ROS 2 READY FOR COMMANDS ---")

# Start ROS thread
threading.Thread(target=init_ros, daemon=True).start()

# ==========================================
# 3. DEFINE MCP TOOLS
# ==========================================
mcp = FastMCP("YahBoom_Robot")

@mcp.tool()
def list_rooms() -> str:
    """Returns a list of all known rooms the robot can go to."""
    return f"Known rooms: {', '.join(LOCATIONS.keys())}"

@mcp.tool()
def move_robot(room_name: str) -> str:
    """
    Moves the robot to a specific room.
    Args:
        room_name: The name of the room (e.g. 'kitchen', 'bedroom')
    """
    global navigator
    
    # 1. Safety Checks
    if navigator is None:
        return "Error: Navigation system is not ready yet."
    
    room_name = room_name.lower().strip()
    if room_name not in LOCATIONS:
        return f"Error: I don't know where '{room_name}' is."

    # 2. Get Coordinates
    coords = LOCATIONS[room_name]
    print(f"SERVER: Received command -> Go to {room_name} {coords}")

    # 3. Send Command to ROS
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = float(coords[0])
    goal.pose.position.y = float(coords[1])
    goal.pose.orientation.w = 1.0
    
    navigator.goToPose(goal)

    # 4. Wait for Arrival (Blocking the tool is okay)
    while not navigator.isTaskComplete():
        time.sleep(0.5)

    # 5. Return Result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        return f"Success: Arrived at {room_name}."
    else:
        return "Failed: I could not reach the destination (Path blocked?)."

# ==========================================
# 4. RUN SERVER
# ==========================================
if __name__ == "__main__":
    print("MCP Server running on Stdio...")
    mcp.run()
