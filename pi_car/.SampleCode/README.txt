Jim Feher - jdfeher@wustl.edu
6/18/24

Ros2 Example of a multiple publisher single subscriber system.
Uses Humble Hawksbill

Assumes you have followed the documentation and created a workspace and
configured things accordingly.  Try to first run the simple publisher
subscribers provided in the Hawksbill docs.

Create a new package and use the following code

This sample will create 4 nodes
1 subscriber
3 publishers
- Imu publisher works with mpu6050
- Magnetomter publisher works with mmu5603
- Keyboard input upon each key entry

Launch file for subscriber/listener, IMU & magnetometer nodes
Create a launch diretory in the package and put the launch file there
Note, workspace is not defined, could cause issues with multiple nodes on the network,
but needed to talk to keyboard.

Update the setup.py accordingly, example provided here

Update the package using colcon build

Launch the keybaord in its own terminal so that it will attach to keyboard, you 
cannot launch with ros2 launch, the keyboard will not connect to the script otherwise.

NOTE: NAMSPACE
The namespace listed is samplePub, you should consider changing it in the launch 
file and in the command you run for the keyboard.  If you do not change it, you 
risk having collisions where multiple ROS nodes use the same namespace and you 
end up either controlling someone else's robot or reading their data, which is not
helpful.

$ ros2 run sample_pubsub pub_keyboard --ros-args --remap __ns:=/samplePub

Then in another window, you should be able to launch the other 3 nodes

$ ros2 launch talk_test_launch.xml

The listener is set up to output none of the sensors, one or the other or both
depending upon keyboard input.  This could be addpated to use additional sensors
or to adjust servos etc.





