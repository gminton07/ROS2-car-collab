#!/bin/bash
# I place this file in my home directory, so I can easily access it with
# $ ~/rebuild.sh
# Before using, you must add executable permissions through
# $ chmod +x rebuild.sh

echo "Hello cruel world!"
echo ""

cd /home/gabe/ros2_ws
colcon build --packages-select pi_car
source /home/gabe/ros2_ws/install/setup.bash
## TODO: change "gabe" to your username

echo "Finished"
echo ""
echo ""
