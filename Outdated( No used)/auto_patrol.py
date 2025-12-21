import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import UInt16, Float32
from sensor_msgs.msg import LaserScan
from datetime import datetime
import time
import os
import sys
import threading

# =========================================
# CONFIGURATION
# =========================================
ROUTE_FILE = "my_saved_route.txt"
LOG_FILE = "patrol_log.txt"
OBSTACLE_DISTANCE = 0.45
# =========================================

class RobotSystem(Node):
    def __init__(self):
        super().__init__('robot_system')
        self.beep_pub = self.create_publisher(UInt16, '/beep', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.click_sub = self.create_subscription(PointStamped, '/clicked_point', self.record_point_callback, 10)
        
        self.recording_mode = False
        self.recorded_points = []
        self.latest_voltage = 0.0
        self.obstacle_active = False
        self.last_warn = 0

    def beep(self, ms):
        msg = UInt16()
        msg.data = int(ms)
        self.beep_pub.publish(msg)

    def scan_callback(self, msg):
        # Obstacle detection logic
        scan = msg.ranges
        front = scan[0:10] + scan[-10:]
        valid = [x for x in front if 0.05 < x < 10.0]
        if valid and min(valid) < OBSTACLE_DISTANCE:
            if time.time() - self.last_warn > 5.0:
                self.log(f"WARNING: Obstacle detected ({min(valid):.2f}m)!")
                self.beep(50)
                self.last_warn = time.time()

    def record_point_callback(self, msg):
        # Only listen if we are in Recording Mode
        if self.recording_mode:
            x = round(msg.point.x, 2)
            y = round(msg.point.y, 2)
            self.recorded_points.append([x, y])
            print(f"Captured Point #{len(self.recorded_points)}: [{x}, {y}]")
            self.beep(100) # Beep to confirm recording

    def log(self, text):
        t = datetime.now().strftime("%H:%M:%S")
        entry = f"[{t}] {text}"
        print(entry)
        with open(LOG_FILE, "a") as f:
            f.write(entry + "\n")

def record_new_route(node):
    node.recording_mode = True
    print("\n" + "="*40)
    print("   RECORDING MODE STARTED")
    print("   1. Go to RViz")
    print("   2. Click 'Publish Point' on the map")
    print("   3. Click where you want the robot to go")
    print("   4. Repeat for all points")
    print("   5. Press Ctrl+C in this terminal to SAVE")
    print("="*40 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nSaving route to file...")
        if len(node.recorded_points) > 0:
            with open(ROUTE_FILE, "w") as f:
                for pt in node.recorded_points:
                    f.write(f"{pt[0]},{pt[1]}\n")
            print(f"Saved {len(node.recorded_points)} points to {ROUTE_FILE}")
            print("Run this script again to PATROL.")
        else:
            print("No points recorded.")

def run_patrol(node, navigator):
    # 1. Load Route
    if not os.path.exists(ROUTE_FILE):
        print(f"ERROR: No route file found ({ROUTE_FILE}).")
        print("Run the script and choose Option 1 first!")
        return

    route = []
    with open(ROUTE_FILE, "r") as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) == 2:
                route.append([float(parts[0]), float(parts[1])])
    
    if not route:
        print("Route file is empty!")
        return

    print(f"Loaded {len(route)} waypoints from file.")
    
    # 2. Start Patrol
    node.log("SYSTEM ONLINE. Connecting to Nav2...")
    navigator.waitUntilNav2Active()
    node.log("PATROL STARTED.")
    node.beep(200)

    loop_count = 1
    try:
        while True:
            node.log(f"--- Round #{loop_count} ---")
            for i, pt in enumerate(route):
                node.log(f"Moving to Waypoint {i+1} ({pt[0]}, {pt[1]})...")
                
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = navigator.get_clock().now().to_msg()
                goal.pose.position.x = pt[0]
                goal.pose.position.y = pt[1]
                goal.pose.orientation.w = 1.0
                
                navigator.goToPose(goal)

                while not navigator.isTaskComplete():
                    rclpy.spin_once(node, timeout_sec=0.1)

                if navigator.getResult() == TaskResult.SUCCEEDED:
                    node.log(f"Arrived at Waypoint {i+1}.")
                    node.beep(200)
                    time.sleep(0.2)
                    node.beep(200)
                    time.sleep(2)
                else:
                    node.log(f"Failed to reach Waypoint {i+1}.")
                    node.beep(1000)

            loop_count += 1
            node.log("Round Complete. Resting...")
            time.sleep(3)
    except KeyboardInterrupt:
        node.log("Patrol stopped by user.")

def main():
    rclpy.init()
    node = RobotSystem()
    
    print("\n=== YAHBOOM PATROL SYSTEM ===")
    print("1. Record a NEW Route (Click in RViz)")
    print("2. Run Saved Patrol")
    choice = input("Enter choice (1 or 2): ")

    if choice == '1':
        record_new_route(node)
    elif choice == '2':
        navigator = BasicNavigator()
        run_patrol(node, navigator)
    else:
        print("Invalid choice.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
