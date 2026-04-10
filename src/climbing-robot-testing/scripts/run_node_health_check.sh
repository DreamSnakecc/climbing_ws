#!/bin/bash

# This script checks the health of the various nodes in the climbing robot's system.

# Function to check if a node is running
check_node() {
    NODE_NAME=$1
    if rosnode list | grep -q "$NODE_NAME"; then
        echo "Node $NODE_NAME is running."
    else
        echo "Node $NODE_NAME is NOT running!"
    fi
}

# List of nodes to check
NODES=(
    "/state_estimator"
    "/body_planner"
    "/swing_leg_controller"
    "/mission_supervisor"
    "/stance_force_optimizer"
    "/jetson/fan_serial_bridge"
    "/jetson/dynamixel_bridge"
)

# Check each node
for NODE in "${NODES[@]}"; do
    check_node "$NODE"
done

# Check the overall health of the system
echo "Checking overall system health..."
if [ "$(rosnode list | wc -l)" -eq "${#NODES[@]}" ]; then
    echo "All nodes are running."
else
    echo "Some nodes are not running. Please check the logs for more details."
fi