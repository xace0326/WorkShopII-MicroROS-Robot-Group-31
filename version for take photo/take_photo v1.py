import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time
import sys

class SnapshotNode(Node):
    def __init__(self):
        super().__init__('snapshot_taker_temp')
        self.image_received = False
        
        # Subscribe to ESP32 Image
        self.create_subscription(Image, '/esp32_img', self.callback, 10)

    def callback(self, msg):
        try:
            # Decode Raw Image
            np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            
            # Convert Color (ROS uses BGR, we want RGB/BGR for saving)
            # OpenCV saves as BGR automatically, so raw is fine.
            
            # Flip if needed (0 = Vertical)
            frame = cv2.flip(np_arr, 0)
            
            # Save to disk
            cv2.imwrite("snapshot.jpg", frame)
            print("SUCCESS")
            self.image_received = True
        except Exception as e:
            print(f"FAIL: {e}")
            self.image_received = True

def main():
    rclpy.init()
    node = SnapshotNode()
    
    # Wait up to 3 seconds for an image
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
