import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import google.generativeai as genai
import time

# ==========================================
# CONFIGURATION
# ==========================================
GOOGLE_API_KEY = "paste your own api key"
MODEL_NAME = "gemini-flash-latest"  # Using the newest model
# ==========================================

genai.configure(api_key=GOOGLE_API_KEY)

# --- 1. DEFINE THE TOOLS (The Robot's Skills) ---
def move_forward(meters: float):
    """
    Moves the robot forward by a specific distance.
    Args:
        meters: Distance in meters (e.g., 0.5, 1.0, 5.0).
    """
    # This just returns the data. The Main Loop executes it.
    return {"action": "move", "dist": meters}

def turn_robot(direction: str):
    """
    Turns the robot.
    Args:
        direction: 'left', 'right', or 'around' (180 turn).
    """
    return {"action": "turn", "dir": direction}

# --- 2. SETUP GEMINI 3 ---
# We give the AI the list of tools it can use
tools_list = [move_forward, turn_robot]

model = genai.GenerativeModel(
    model_name=MODEL_NAME,
    tools=tools_list,
    system_instruction="""
    You are a physical robot.
    - If the user says 'move forward 5 meters', call move_forward(5.0).
    - If the user says 'turn around', call turn_robot('around').
    - If the user just says 'hello', reply normally.
    """
)

# --- 3. THE ROS NODE (Hardware Control) ---
class SimpleDriver(Node):
    def __init__(self):
        super().__init__('simple_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def execute_move(self, distance):
        print(f"🚗 EXECUTING: Drive Forward {distance} meters")
        
        # Physics Math: Distance = Speed * Time
        speed = 0.2  # Meters per second (Safe speed)
        duration = float(distance) / speed
        
        # Create message
        msg = Twist()
        msg.linear.x = speed
        
        # Move loop
        start_time = time.time()
        while time.time() - start_time < duration:
            self.pub.publish(msg)
            time.sleep(0.1) # Send command 10 times a second
            
        self.stop()
        print("✅ Movement Complete.")

    def execute_turn(self, direction):
        print(f"🔄 EXECUTING: Turn {direction}")
        msg = Twist()
        
        # Turn logic
        if direction == "left":
            msg.angular.z = 0.5
            duration = 3.2 # Approx 90 degrees
        elif direction == "right":
            msg.angular.z = -0.5
            duration = 3.2
        elif direction == "around":
            msg.angular.z = 0.5
            duration = 6.4 # Approx 180 degrees
        else:
            print("Unknown direction")
            return

        start_time = time.time()
        while time.time() - start_time < duration:
            self.pub.publish(msg)
            time.sleep(0.1)
            
        self.stop()
        print("✅ Turn Complete.")

    def stop(self):
        # Send zero velocity to stop motors
        self.pub.publish(Twist())

# --- 4. MAIN LOOP ---
def main():
    rclpy.init()
    robot = SimpleDriver()
    
    print(f"--- AI DRIVER ONLINE ({MODEL_NAME}) ---")
    print("Commands example: 'Move 1 meter', 'Turn left', 'Turn around'")

    # Start the Chat
    chat = model.start_chat(enable_automatic_function_calling=False)

    while True:
        try:
            # 1. Get User Input
            user_input = input("\nYOU: ")
            if user_input.lower() in ["exit", "quit"]: break

            print("ROBOT: Thinking...")
            
            # 2. Send to Gemini 3
            response = chat.send_message(user_input)

            # 3. Check if Gemini 3 called a tool
            tool_used = False
            
            # We iterate through the response parts to find function calls
            for part in response.candidates[0].content.parts:
                if part.function_call:
                    tool_used = True
                    fname = part.function_call.name
                    args = part.function_call.args
                    
                    if fname == "move_forward":
                        dist = args["meters"]
                        robot.execute_move(dist)
                        
                    elif fname == "turn_robot":
                        direction = args["direction"]
                        robot.execute_turn(direction)

            # 4. If no tool was used, print the text reply
            if not tool_used:
                print(f"AI: {response.text}")

        except Exception as e:
            print(f"Error: {e}")
            robot.stop()

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
