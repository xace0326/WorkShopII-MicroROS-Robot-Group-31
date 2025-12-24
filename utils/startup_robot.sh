#!/bin/bash

# --- FIX FOR DESKTOP ICON ---
cd /home/yahboom/my_robot_project
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ws/install/setup.bash
# ---------------------------

# --- CONFIGURATION ---
LOG_DIR=~/my_robot_project/logs
PROJECT_DIR=~/my_robot_project

# --- FUNCTION: CLEANUP ---
function cleanup {
    echo ""
    echo "🛑 STOPPING SOFTWARE STACK..."
    kill 0
    # Force kill the Bridge and Streamlit to prevent zombie ports
    sudo fuser -k 9090/tcp > /dev/null 2>&1
    pkill -f rosbridge
    pkill -f streamlit
    wait
    echo "✅ Software stopped. (Manually stop Agent & Camera Proxy!)"
}

trap cleanup SIGINT EXIT

# --- PRE-FLIGHT CHECK ---
# Clear port 9090 (Bridge) to prevent "Address already in use"
sudo fuser -k 9090/tcp > /dev/null 2>&1

echo "=============================================="
echo "   🤖 YAHBOOM AI - SOFTWARE STARTUP"
echo "=============================================="
mkdir -p $LOG_DIR

# 1. HARDWARE (Bringup)
echo "1️⃣️.  Starting Hardware (Bringup)..."
bash -c "source /opt/ros/humble/setup.bash && source ~/yahboomcar_ws/install/setup.bash && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py" > $LOG_DIR/1_bringup.log 2>&1 &
sleep 3

# 3. BRIDGE
echo "2️⃣️.  Starting ROS Bridge..."
bash -c "source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml" > $LOG_DIR/2_bridge.log 2>&1 &
sleep 3

# 4. NAVIGATION
echo "3️⃣️.  Starting Navigation..."
bash -c "source ~/yahboomcar_ws/install/setup.bash && ros2 launch yahboomcar_nav navigation_dwb_launch.py map:=/home/yahboom/my_robot_project/maps/my_new_map.yaml" > $LOG_DIR/3_nav.log 2>&1 &
sleep 5


# --- NEW: AUTO LOCALIZE ---
echo "📍 Setting Initial Pose..."
bash -c "source ~/yahboomcar_ws/install/setup.bash && python3 ~/my_robot_project/utils/auto_localize.py"
# --------------------------

# 5. AI INTERFACE
echo "4️⃣️.  Starting AI Web Interface..."
bash -c "source ~/yahboomcar_ws/install/setup.bash && cd $PROJECT_DIR && streamlit run mcp_client.py" > $LOG_DIR/4_web.log 2>&1 &

echo ""
echo "✅ SYSTEM IS RUNNING!"
echo "----------------------------------------------"
echo "🌐 Open your browser to the URL below:"
echo "----------------------------------------------"

tail -f $LOG_DIR/4_web.log | grep -m 1 "Network URL" -A 2

echo ""
echo "Press CTRL+C to stop software."
wait
