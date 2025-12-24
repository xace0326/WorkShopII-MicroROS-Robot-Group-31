import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

# ==========================================
# CONFIGURATION: YOUR HOME COORDINATES
# ==========================================
HOME_X = -0.11677692180624813      # Replace with your X
HOME_Y = -0.6994236937413175      # Replace with your Y
HOME_Z = 0.707658791935663      # Replace with Orientation Z
HOME_W = 0.7065543391673128      # Replace with Orientation W
# ==========================================

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        # This topic is what AMCL listens to (Same as the RViz button)
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        msg.pose.pose.position.x = float(HOME_X)
        msg.pose.pose.position.y = float(HOME_Y)
        msg.pose.pose.position.z = 0.0
        
        # Orientation
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = float(HOME_Z)
        msg.pose.pose.orientation.w = float(HOME_W)
        
        # Covariance (Confidence) - 0.25 is standard
        msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.068]

        print(f"📍 Setting Robot Position to starting point")
        
        # Send it multiple times to make sure AMCL catches it
        for _ in range(3):
            self.publisher_.publish(msg)
            time.sleep(0.5)

def main():
    rclpy.init()
    node = InitialPosePublisher()
    
    # Wait a moment for connections to form
    time.sleep(2)
    node.publish_pose()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
