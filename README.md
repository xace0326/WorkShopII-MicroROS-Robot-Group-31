# 🤖 WALL-X: AI-Driven Autonomous Patrol Robot

This project integrates a **YahBoom MicroROS Robot** with **Google Gemini 2.0** using the **Model Context Protocol (MCP)**. It enables natural language control, autonomous navigation, and visual intelligence via a web interface.

---

## ⚠️ CRITICAL PREREQUISITES (READ THIS FIRST)

**Before using this code, you MUST set up the robot hardware and drivers following the official YahBoom documentation.**

This project relies on the underlying MicroROS and Camera drivers provided by YahBoom. 
**You must successfully complete the setup here:**

👉 **[YahBoom MicroROS ESP32 Tutorial](https://www.yahboom.net/study/MicroROS-ESP32)**

**Ensure the following work manually before running this project:**
1.  **MicroROS Agent:** Can connect to the robot (`sh start_agent_computer.sh`).
2.  **Camera:** Can stream video (`ros2 run yahboom_esp32_camera sub_img`).
3.  **Lidar/SLAM:** The robot can map an area.

---

## 🌟 Features
*   **Voice & Text Control:** Speak to the robot via laptop/phone.
*   **Intelligent Navigation:** "Go to the kitchen" (Resolves coordinates automatically).
*   **Visual Analysis:** "What do you see?" (Takes snapshot & analyzes with Gemini Vision).
*   **System Telemetry:** Battery monitoring and Status checks.
*   **Robust Architecture:** Uses MCP to decouple the AI (Client) from ROS 2 (Server).

## 📂 Project Structure
```text
├── maps/                  # SLAM Maps (.yaml/.pgm)
├── modified_system_files  # The modified system file from YahBoom
├── ros-mcp-server/        # Generic MCP Server (FAST-MCP-SERVER)
├── utils/                 # Helper scripts (auto_localize, take_photo)
├── mcp_client.py      	   # Main Streamlit App (MCP Client)
├── startup_robot.sh       # One-click startup script
└── README.md              # This file
```

## 🚀 Installation & Setup
1. **Clone this repository:**
```bash
git clone https://github.com/YOUR_USERNAME/YahBoom-MCP-Robot.git
cd YahBoom-MCP-Robot
```

2. **Install Dependencies:**
```bash
pip3 install streamlit mcp google-generativeai pillow opencv-python gTTS
```

3. **Install the MCP Server:**
```bash
cd ros-mcp-server
pip3 install .
```

4. **Configure API Key:**
Open "mcp_client.py" and paste your Google Gemini API Key.

## 🕹️ How to Run (The 3-Window Method)
Because this system involves hardware, we recommend running it in 3 separate terminals:

**Terminal 1: The Hardware Link**
```bash
sh ~/start_agent_computer.sh
# If stuck, press the Reset button on the robot.
```
**Terminal 2: The Camera Driver**
```bash
sh ~/start_Camera_computer.sh
# Then run the ros node in another tab if not automated
# ros2 run yahboom_esp32_camera sub_img
```

**Terminal 3: The AI System**
```bash
./startup_robot.sh
```
Wait for the URL to appear, then open it in your browser.

## 🛠️ Configuration Notes
Map: If you move the robot to a new room, you must re-map using Cartographer/Gmapping and update the coordinates in mcp_client.py and auto_localize.py.
Navigation Tuning: If you navigation has problem, you try to change the parameters at the file params/nav2_params.yaml.

## 📹️ VIDEO For the Project
👉 **[WALL-X: Watch, Analyze, Learn and Locate AI Robot PROEJECT VIDEO](https://www.youtube.com/watch?v=TFVVhTY-xK0)**

