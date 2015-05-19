# Simple CAN #

This is a ROS-package that performs two principal functions:
1. Low Level Protocol Communication through CANopen Protocol.
2. High Level Packet construction to interface an Elmo Controller through CANopen protocol.

### Requisites ###

* Can interface Capabilities
* Linux Distribution
* ROS

### How do I get set up? ###

* Enabling CAN:
After you have enabled can in the kernel, it is possible to set it up.

+ sudo ip link set can0 up type can bitrate 1000000
+ sudo ifconfig can0 up

**If you have the shell files init_can.sh, instead of doing the previous step, you can just use this script **

sudo su

sudo ./init_can.sh

exit

### Deployment instructions ###

Usually, under normal circumstances, you can run a test program which utilizes two nodes, can_node and can_sniffer.

In terminal 1:

cd catkin_ws/

roscore &

source devel/setup.bash

rosrun ambpros_can_driver can_node

In terminal 2:

cd catkin_ws

source devel/setup.bash

rosrun ambpros_can_driver can_sniffer

### Troubleshooting ###

Killing a program stucked:

ps aux | grep <name>

kill -15 <pid>

Testing CAN communication (Must have can-utils):

Terminal 1:

cd can/can-utils

./candump can0

Terminal 2:

cd can/can-utils

./cansend can0 000#8100

(Check that all the devices answer the frame sent)