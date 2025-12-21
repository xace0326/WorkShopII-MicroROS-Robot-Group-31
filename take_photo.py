import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time
import os
import datetime

# --- CONFIGURATION ---
SAVE_DIR = "/home/yahboom/my_robot_project/snapshots"
# ---------------------

class SnapshotNode(Node):
    def __init__(self):
        super().__init__('snapshot_taker')
        self.image_received = False
        self.saved_path = ""
        
        # QoS for WiFi Camera
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(Image, '/esp32_img', self.callback, qos_policy)

    def callback(self, msg):
        try:
            # 1. Decode
            np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            
            # 2. Flip (YahBoom cameras are often upside down)
            frame = cv2.flip(np_arr, 0)
            
            # 3. Create Unique Filename
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"photo_{timestamp}.jpg"
            full_path = os.path.join(SAVE_DIR, filename)
            
            # 4. Save
            if not os.path.exists(SAVE_DIR):
                os.makedirs(SAVE_DIR)
                
            cv2.imwrite(full_path, frame)
            
            # 5. Tell the website where it is
            self.saved_path = full_path
            print(f"SUCCESS:{full_path}") # <--- Important!
            self.image_received = True
            
        except Exception as e:
            print(f"FAIL: {e}")
            self.image_received = True

def main():
    rclpy.init()
    node = SnapshotNode()
    
    start_time = time.time()
    while not node.image_received:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start_time > 3.0:
            print("TIMEOUT")
            break
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
