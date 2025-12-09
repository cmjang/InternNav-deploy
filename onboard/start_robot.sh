#!/bin/bash

# ================= CONFIGURATION =================
# set the network interface connected to the robot default: eth0
ROBOT_IF="eth0"
# =================================================

# Function to kill all background processes when Ctrl+C is pressed
cleanup() {
    echo ""
    echo "========================================="
    echo "   Stopping all robot modules..."
    echo "========================================="
    kill 0
    exit
}

# Trap SIGINT (Ctrl+C) to run the cleanup function
trap cleanup SIGINT

echo "========================================="
echo "   Launching OpenLegged Robot System"
echo "========================================="
echo "   Target Interface: $ROBOT_IF"
echo "========================================="

# 1. Start the Unitree Driver (Bottom Layer)
# We pass the interface variable ($ROBOT_IF) to the python script
echo "[1/3] Starting Unitree Driver..."
python3 unitree_driver.py "$ROBOT_IF" &
PID_DRIVER=$!
sleep 3  # Wait for the robot to stand up

# 2. Start the ROS Bridge
echo "[2/3] Starting ROS Bridge..."
python3 ros_bridge.py &
PID_BRIDGE=$!
sleep 1

# 3. Start RealSense Publisher
# Assuming your file is named realsense_publisher.py
echo "[3/3] Starting RealSense Publisher..."
if [ -f "realsense_publisher.py" ]; then
    python3 realsense_publisher.py &
    PID_CAMERA=$!
else
    echo "[Warning] realsense_publisher.py not found, skipping."
fi

echo "========================================="
echo "   All Systems GO! Press Ctrl+C to stop."
echo "========================================="

# Keep the script running to maintain the trap
wait