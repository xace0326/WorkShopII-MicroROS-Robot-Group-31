import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import subprocess
import time
import os

def center_servo():
    # Only center if not already centered to save time
    try:
        proc = subprocess.Popen(
            ["ros2", "run", "yahboom_esp32_mediapipe", "control_servo"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        time.sleep(2)
        proc.terminate()
    except:
        pass

class CustomViewer(Node):
    def __init__(self):
        super().__init__('smart_camera_viewer')
        topic_name = '/esp32_img'
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(Image, topic_name, self.img_callback, qos)

    def img_callback(self, msg):
        try:
            # MANUAL DECODE (No Crash)
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            
            # 1. FLIP (0 = Vertical, 1 = Horizontal, -1 = Both)
            # Try changing this number if it's still upside down!
            frame = cv2.flip(frame, 0) 

            # 2. DRAW CENTER
            h, w, _ = frame.shape
            cx, cy = w // 2, h // 2
            cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
            cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)

            cv2.imshow("AI Smart Camera", frame)
            cv2.waitKey(1)
        except:
            pass

def main():
    center_servo()
    rclpy.init()
    node = CustomViewer()
    try:
        rclpy.spin(node)
    except:
        pass
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
