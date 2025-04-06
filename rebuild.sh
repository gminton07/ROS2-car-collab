#!/bin/bash

echo "Hello cruel world!"
echo ""

cd /home/gabe/ros2_ws
colcon build --packages-select sample_pubsub
source /home/gabe/ros2_ws/install/setup.bash

echo "Finished"
echo ""
echo ""
