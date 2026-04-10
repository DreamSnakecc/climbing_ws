#!/bin/bash

# This script validates the messages being published and subscribed to within the climbing robot's communication framework.

# Source the ROS environment
source /opt/ros/noetic/setup.bash
source ~/climbing_ws/devel/setup.bash

# Function to validate message types
validate_message() {
    local topic=$1
    local expected_type=$2

    echo "Validating topic: $topic"
    actual_type=$(rostopic type $topic)

    if [ "$actual_type" == "$expected_type" ]; then
        echo "Validation successful: $topic is of type $expected_type"
    else
        echo "Validation failed: $topic is of type $actual_type, expected $expected_type"
    fi
}

# List of topics and their expected message types
declare -A topics
topics=(
    ["/state/estimated"]="climbing_msgs/EstimatedState"
    ["/control/body_reference"]="climbing_msgs/BodyReference"
    ["/control/swing_leg_target"]="climbing_msgs/SwingLegTarget"
    ["/control/stance_wrench/lf"]="geometry_msgs/Wrench"
    ["/jetson/dynamixel_bridge/joint_state"]="sensor_msgs/JointState"
    ["/jetson/fan_serial_bridge/fan_currents"]="std_msgs/Float32MultiArray"
)

# Validate each topic
for topic in "${!topics[@]}"; do
    validate_message $topic ${topics[$topic]}
done

echo "Message validation completed."