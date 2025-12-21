#!/bin/bash

echo "🚀 STARTING YAHBOOM ROBOT SYSTEM..."

# 0. MICRO-ROS AGENT (CRITICAL!)
# This connects the wheels/battery to the computer.
gnome-terminal --tab --title="0-Agent" -- bash -c "
echo 'Starting MicroROS Agent...';
sh ~/start_agent_computer.sh;
exec bash"

echo "⏳ Waiting 5 seconds for Agent to connect..."
sleep 5

# 1. HARDWARE (Bringup)
gnome-terminal --tab --title="1-Hardware" -- bash -c "
echo 'Starting Hardware...';
source /opt/ros/humble/setup.bash;
source ~/yahboomcar_ws/install/setup.bash;
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py;
exec bash"

echo "⏳ Waiting 8 seconds for hardware to warm up..."
sleep 8

# 2. CAMERA PROXY
gnome-terminal --tab --title="2-CamProxy" -- bash -c "
echo 'Starting Camera Proxy...';
sh ~/start_Camera_computer.sh;
exec bash"

sleep 2

# 3. CAMERA NODE (ROS Topic)
gnome-terminal --tab --title="3-CamNode" -- bash -c "
echo 'Starting Camera ROS Node...';
source ~/yahboomcar_ws/install/setup.bash;
ros2 run yahboom_esp32_camera sub_img;
exec bash"

# 4. BRIDGE (For MCP)
gnome-terminal --tab --title="4-Bridge" -- bash -c "
echo 'Starting ROS Bridge...';
source /opt/ros/humble/setup.bash;
# sudo apt install ros-humble-rosbridge-server -y; # Uncomment if needed
ros2 launch rosbridge_server rosbridge_websocket_launch.xml;
exec bash"

sleep 2

# 5. NAVIGATION (Map)
gnome-terminal --tab --title="5-Nav" -- bash -c "
echo 'Starting Navigation...';
source ~/yahboomcar_ws/install/setup.bash;
# CHECK THIS PATH!
ros2 launch yahboomcar_nav navigation_dwb_launch.py map:=/home/yahboom/my_robot_project/maps/my_new_map.yaml;
exec bash"

echo "⏳ Waiting 5 seconds for Navigation..."
sleep 5

# 6. THE AI INTERFACE (Streamlit)
gnome-terminal --tab --title="6-AI-Interface" -- bash -c "
echo 'Starting AI Web Interface...';
source ~/yahboomcar_ws/install/setup.bash;
cd ~/my_robot_project;
streamlit run web_interface.py;
exec bash"

echo "✅ SYSTEM LAUNCHED!"
echo "Go to the URL shown in the 'AI-Interface' tab."
