import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import UInt16
import os
import time

# ==========================================
# CONFIGURATION
# ==========================================
MAP_NAME = "my_auto_map"
WAIT_TIME_SECONDS = 5  # Reduced wait time
# ==========================================

class MapWatchdog(Node):
    def __init__(self):
        super().__init__('map_watchdog')
        
        # Listen to the Explorer
        self.subscription = self.create_subscription(
            MarkerArray,
            '/explore_markers',
            self.listener_callback,
            10)
        
        self.beep_pub = self.create_publisher(UInt16, '/beep', 10)
        
        self.map_saved = False
        self.last_msg_time = time.time()
        self.zero_count_start_time = None
        self.first_msg_received = False

    def beep(self, ms):
        msg = UInt16()
        msg.data = int(ms)
        self.beep_pub.publish(msg)

    def listener_callback(self, msg):
        if self.map_saved:
            return

        self.first_msg_received = True
        self.last_msg_time = time.time()
        
        # Count frontiers (Unknown areas)
        current_count = len(msg.markers)
        print(f"Status Update: {current_count} frontiers detected.")

        if current_count > 0:
            # We are exploring
            self.zero_count_start_time = None

        elif current_count == 0:
            # We might be done
            if self.zero_count_start_time is None:
                self.zero_count_start_time = time.time()
                print("⚠️ No frontiers seen. Timer started...")
            
            # If we see 0 frontiers for X seconds, SAVE.
            elif time.time() - self.zero_count_start_time > WAIT_TIME_SECONDS:
                print("Timer expired. Exploration assumed complete.")
                self.finish_and_save()

    def finish_and_save(self):
        self.map_saved = True
        print("\n" + "="*40)
        print("✅ SAVING MAP NOW!")
        print("="*40)
        
        # Beep
        for _ in range(3):
            self.beep(200)
            time.sleep(0.3)

        # Save
        save_cmd = f"ros2 run nav2_map_server map_saver_cli -f ~/{MAP_NAME}"
        os.system(save_cmd)
        
        print("🎉 MAP SAVED. You can close everything.")
        exit(0)

def main():
    rclpy.init()
    print("--- MAP WATCHDOG V2 STARTED ---")
    print("Waiting for data from /explore_markers...")
    
    watchdog = MapWatchdog()
    
    # Spin loop
    while rclpy.ok():
        rclpy.spin_once(watchdog, timeout_sec=1.0)
        
        # Safety check: If we don't hear ANYTHING for 30 seconds, warn user
        if not watchdog.first_msg_received and (time.time() - watchdog.last_msg_time > 30):
            print("WARNING: No data received yet. Is explore_lite running?")
            watchdog.last_msg_time = time.time() # Reset warning timer

    rclpy.shutdown()

if __name__ == '__main__':
    main()
