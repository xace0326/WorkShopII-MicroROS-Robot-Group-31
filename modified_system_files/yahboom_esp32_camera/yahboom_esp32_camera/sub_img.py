import rclpy
import time
import subprocess
import os
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from rclpy.time import Time
import datetime

class SubImg(Node):
    def __init__(self, name):
        super().__init__(name)
        self.bridge = CvBridge()
        # Subscribe to ESP32 stream
        self.sub_img = self.create_subscription(
            CompressedImage, '/espRos/esp32camera', self.handleTopic, 1)
        
        # Publish processed topics
        self.pub_img = self.create_publisher(Image, "/esp32_img", 1)
        self.pub_comimg = self.create_publisher(CompressedImage, "/usb_cam/image_raw/compressed", 1)

        self.last_stamp = None
        self.new_seconds = 0
        self.fps_seconds = 1
        
        # --- FIX: FORCE SERVO TO CENTER (ROBUST METHOD) ---
        print("📷 Attempting to center camera servo...")
        try:
            # 1. We construct the full command chain
            cmd = "source /opt/ros/humble/setup.bash && source ~/yahboomcar_ws/install/setup.bash && ros2 run yahboom_esp32_mediapipe control_servo"
            
            # 2. We execute using /bin/bash to support 'source'
            self.servo_proc = subprocess.Popen(
                cmd, 
                shell=True, 
                executable='/bin/bash', # Critical for ROS commands to work
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.DEVNULL
            )
            print("✅ Servo command sent (Running in background).")
            
        except Exception as e:
            print(f"⚠️ Servo Error: {e}")
        # --------------------------------------------------

    def handleTopic(self, msg):
        self.last_stamp = msg.header.stamp  
        if self.last_stamp:
            total_secs = Time(nanoseconds=self.last_stamp.nanosec, seconds=self.last_stamp.sec).nanoseconds
            delta = datetime.timedelta(seconds=total_secs * 1e-9)
            seconds = delta.total_seconds()*100

            if self.new_seconds != 0:
                self.fps_seconds = seconds - self.new_seconds

            self.new_seconds = seconds

        start = time.time()
      
        try:
            # Decode Image
            frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        except Exception as e:
            return # Skip if data is corrupted

        # --- FIX ROTATION ---
        # 1 = Horizontal Flip (Mirror) - Usually correct for YahBoom
        # Change to -1 if it is still sideways/upside down
        frame = cv.flip(frame, 1) 
        # --------------------

        # Publish for App (Optional)
        img_msgcom = self.bridge.cv2_to_compressed_imgmsg(frame, "jpg")
        self.pub_comimg.publish(img_msgcom) 

        # Resize for display performance
        frame = cv.resize(frame, (640, 480))
        
        # Calculate FPS
        end = time.time()
        fps = 1/((end - start)+self.fps_seconds) 
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)

        # Show Window
        cv.imshow("color_image", frame)
        cv.waitKey(10)
        
        # Publish Raw Image for AI
        img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.pub_img.publish(img_msg)

    def destroy_node(self):
        # Cleanup Servo Process when we exit
        if hasattr(self, 'servo_proc'):
            try:
                self.servo_proc.terminate()
            except:
                pass
        super().destroy_node()

def main():
    rclpy.init()
    esp_img = SubImg("sub_img")
    try:
        rclpy.spin(esp_img)
    except KeyboardInterrupt:
        pass
    finally:
        esp_img.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
