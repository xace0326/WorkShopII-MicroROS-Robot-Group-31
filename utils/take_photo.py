import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time
import os
import datetime
import sys

# --- CONFIGURATION ---
SAVE_DIR = "/home/yahboom/my_robot_project/snapshots"
# ---------------------

class SnapshotNode(Node):
    def __init__(self):
        super().__init__('snapshot_taker')
        self.image_received = False
        
        # QoS for WiFi Camera (Best Effort is REQUIRED)
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        # Subscribe to RAW image (not compressed, since we use manual decode)
        # Note: If your robot uses /image_raw/compressed, we need a different decoder.
        # Based on your previous logs, you use /esp32_img which is RAW.
        self.create_subscription(Image, '/esp32_img', self.callback, qos_policy)

    def callback(self, msg):
        try:
            # 1. Decode Raw Image manually (Avoids cv_bridge crash)
            # ROS Image is just a byte array. We reshape it.
            np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            
            # 2. Flip if needed (Match your live window!)
            # 1 = Horizontal Flip
            frame = cv2.flip(np_arr, 1)
            
            # 3. Create Unique Filename
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"photo_{timestamp}.jpg"
            full_path = os.path.join(SAVE_DIR, filename)
            
            # 4. Save
            if not os.path.exists(SAVE_DIR):
                os.makedirs(SAVE_DIR)
                
            cv2.imwrite(full_path, frame)
            
            # 5. Output Success
            print(f"SUCCESS:{full_path}")
            sys.stdout.flush() # Force print to appear
            self.image_received = True
            
        except Exception as e:
            # Don't crash, just print error
            print(f"FAIL: {e}")
            self.image_received = True

def main():
    if not rclpy.ok():
        rclpy.init()
        
    node = SnapshotNode()
    
    start_time = time.time()
    while not node.image_received:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start_time > 5.0: # Wait 5 seconds max
            print("TIMEOUT")
            break
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
