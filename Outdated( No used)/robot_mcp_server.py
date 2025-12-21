import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image       # <--- CHANGED: Using Raw Image
from cv_bridge import CvBridge          # <--- NEW: Tool to decode Raw Image
from std_msgs.msg import Float32, UInt16
from mcp.server.fastmcp import FastMCP
import threading
import time
import subprocess
import cv2
import numpy as np

# ==========================================
# 1. CONFIGURATION
# ==========================================
LOCATIONS = {
    "kitchen": [1.5, 0.5],     # Update these with your real map numbers!
    "living_room": [0.0, 0.0],
    "bedroom": [2.0, -1.0]
}

# ==========================================
# 2. SETUP ROS 2 IN BACKGROUND
# ==========================================
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
# 3. HELPER FUNCTIONS (Camera Viewer)
# ==========================================
def run_custom_viewer(duration=10):
    """
    Subscribes to the ESP32 camera (/esp32_img), flips it, and shows it.
    """
    # Create a temporary node just for viewing
    view_node = rclpy.create_node('ai_camera_viewer_temp')
    
    # Create the Bridge to convert ROS -> OpenCV
    bridge = CvBridge()
    
    start_time = time.time()
    
    def img_callback(msg):
        try:
            # --- CONVERT RAW IMAGE ---
            # using cv_bridge to convert standard ROS Image to OpenCV
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if frame is None:
                return

            # --- AUTO INVERT ---
            # Flip Code: 0=Vertical, 1=Horizontal, -1=Both
            # Adjust this if it's upside down or mirrored!
            frame = cv2.flip(frame, 1) 
            
            # --- DRAW CENTER LINES ---
            h, w, _ = frame.shape
            cx, cy = w // 2, h // 2
            cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
            cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)
            
            cv2.imshow("AI Camera (Auto-Centered)", frame)
            cv2.waitKey(1)
        except Exception as e:
            print(f"Frame Error: {e}")

    # QoS to ensure we catch the stream
    qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
    
    # SUBSCRIBE to the correct topic: /esp32_img
    view_node.create_subscription(Image, '/esp32_img', img_callback, qos_policy)

    # Spin for 'duration' seconds
    while time.time() - start_time < duration:
        rclpy.spin_once(view_node, timeout_sec=0.1)
    
    # Cleanup
    cv2.destroyAllWindows()
    view_node.destroy_node()

# ==========================================
# 4. DEFINE MCP TOOLS
# ==========================================
mcp = FastMCP("YahBoom_Robot")

@mcp.tool()
def list_rooms() -> str:
    """Returns a list of all known rooms the robot can go to."""
    return f"Known rooms: {', '.join(LOCATIONS.keys())}"

@mcp.tool()
def move_robot(room_name: str) -> str:
    """Moves the robot to a specific room."""
    global navigator
    
    if navigator is None:
        return "Error: Navigation system is not ready yet."
    
    room_name = room_name.lower().strip()
    if room_name not in LOCATIONS:
        return f"Error: I don't know where '{room_name}' is."

    coords = LOCATIONS[room_name]
    print(f"SERVER: Received command -> Go to {room_name} {coords}")

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = float(coords[0])
    goal.pose.position.y = float(coords[1])
    goal.pose.orientation.w = 1.0
    
    navigator.goToPose(goal)

    while not navigator.isTaskComplete():
        time.sleep(0.5)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        return f"Success: Arrived at {room_name}."
    else:
        return "Failed: I could not reach the destination."

@mcp.tool()
def open_camera_smart() -> str:
    """
    1. Centers the camera servo.
    2. Opens the camera feed for 15 seconds.
    """
    print("SERVER: Centering Camera Servo...")
    
    # --- STEP 1: CENTER THE SERVO ---
    try:
        # Run the YahBoom servo control node briefly to center it
        servo_process = subprocess.Popen(
            ["ros2", "run", "yahboom_esp32_mediapipe", "control_servo"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        time.sleep(3) # Wait for movement
        servo_process.terminate() 
        print("SERVER: Servo Centered.")
    except Exception as e:
        print(f"Servo Warning: {e}")

    # --- STEP 2: OPEN VIEWER ---
    print("SERVER: Opening Viewer...")
    try:
        run_custom_viewer(duration=15)
    except Exception as e:
        return f"Error opening video: {e}"
    
    return "Success: Camera centered and displayed for 15 seconds."

# ==========================================
# 5. RUN SERVER
# ==========================================
if __name__ == "__main__":
    print("MCP Server running on Stdio...")
    mcp.run()
