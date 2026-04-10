#!/bin/bash

# This script performs preliminary checks before running the main tests for the climbing robot.

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check for required commands
required_commands=("roscore" "roslaunch" "rosnode" "rostopic" "rosparam")

for cmd in "${required_commands[@]}"; do
    if ! command_exists "$cmd"; then
        echo "Error: $cmd is not installed. Please install it before proceeding."
        exit 1
    fi
done

# Source the ROS environment
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
else
    echo "Error: ROS environment setup file not found."
    exit 1
fi

# Source the workspace
if [ -d "~/climbing_ws/devel" ]; then
    source ~/climbing_ws/devel/setup.bash
else
    echo "Error: Workspace setup file not found. Please build the workspace first."
    exit 1
fi

# Check if the necessary ROS nodes are running
echo "Checking if necessary ROS nodes are running..."
nodes=("node1" "node2" "node3") # Replace with actual node names

for node in "${nodes[@]}"; do
    if ! rosnode list | grep -q "$node"; then
        echo "Warning: $node is not running."
    else
        echo "$node is running."
    fi
done

echo "Pre-checks completed successfully."