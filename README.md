# MicroRos Robot (MCP Architecture)

This project integrates a YahBoom ROS 2 Robot with Google Gemini using the Model Context Protocol (MCP).

## Features
- **Natural Language Control:** "Go to kitchen".
- **Visual Intelligence:** "What do you see?" (Uses ESP32 Camera).
- **Autonomous Navigation:** ROS 2 Nav2.
- **Web Interface:** Built with Streamlit.

## Architecture
- **Server:** Generic `ros-mcp-server`.
- **Client:** Custom Streamlit app (`mcp_client.py`).

## How to Run
1. Update the API Key in `mcp_client.py`.
2. Run:
	1. sh ~/start_agent_computer.sh
	2. sh ~/start_Camera_computer.sh
	3. ros2 run yahboom_esp32_camera sub_img
	4. ros2 run yahboom_esp32_mediapipe control_servo
	5. python3 SET_Camera.py  --> Input Camera Proxy Ipv4
	6. ros2 launch yahboomcar_nav display_launch.py
	7. cd my_robot_project
	8. ./startup_robot.sh

It will automatically to open a website interface.

Link for Demo video (v1) : https://youtu.be/9oV5MaUIw2U
Link for Demo video (v2) : https://youtu.be/9OhI1jlYFsU
