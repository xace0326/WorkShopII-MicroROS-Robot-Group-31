# YahBoom AI Patrol Robot (MCP Architecture)

This project integrates a YahBoom ROS 2 Robot with Google Gemini using the Model Context Protocol (MCP).

## Features
- **Natural Language Control:** "Go to kitchen", "Check battery".
- **Visual Intelligence:** "What do you see?" (Uses ESP32 Camera).
- **Autonomous Navigation:** ROS 2 Nav2.
- **Web Interface:** Built with Streamlit.

## Architecture
- **Server:** Generic `ros-mcp-server`.
- **Client:** Custom Streamlit app (`web_interface.py`).
- **Hardware:** MicroROS Agent + Camera Proxy.

## How to Run
1. Update the API Key in `web_interface.py`.
2. Run the startup script:
   sh ~/start_agent_computer.sh (MUST)
   sh ~/start_Camera_computer.sh (Optional). 
If use camera, need to open camera by input this command to new terminal:
   ros2 run yahboom_esp32_camera sub_img

Then input this command:
   cd my_robot_project
   ./startup_robot.sh

It will automatically to open a website interface.

Link for Demo video (v1) : https://youtu.be/9oV5MaUIw2U
