# Everything I Know About The Crane

Last updated 7/29/22. Everything tru still seems to be.

## The Jitter Glitch
Sometimes the crane doesnt move in particular directions, or at least doesn't seem to enjoy it. Unplug the Gantry Computer and reboot the whole system. I don't know why but this seems to help. (My suspicion is the topic_to_socketcan_node is sending bad commands to the CAN and causing it to freak out, but I'm unsure if this is true; I still need to find the original source code for the CAN)

## The CAN
The CAN table in Eisuke's helpful manual is unfortunately out of date. Here is the version I have found experimentally:

## Random Computer Details

### Quinn's .bashrc nonsense
I don't like to type the same commands over and over again, so I've attempted to automate some things. In each of the Computer's ~/.bashrc files, there is a line at the end that sources a ~/catkin_ws/env.sh file. This is where I've placed the important ROS environment variables and such. If you want to change the ROS master to no longer be the Gantry Computer, this is where to do it. NOTE: you'll have to make this change on every computer in the system.