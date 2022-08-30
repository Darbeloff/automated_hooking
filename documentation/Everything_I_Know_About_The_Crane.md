# Everything I Know About The Crane

Last updated 8/30/22. Everything still seems to be true. Refer also to Eisuke's original documentation at the bottom of this document.

## The Jitter Glitch
Sometimes the crane doesnt move in particular directions, or at least doesn't seem to enjoy it. Unplug the Gantry Computer and reboot the whole system. I don't know why, but this seems to help. (My suspicion is the topic_to_socketcan_node is sending bad commands to the CAN and causing it to freak out, but I'm unsure if this is true; I still need to find the original source code for the CAN)

## The CAN
The CAN table in Eisuke's helpful manual is unfortunately out of date. Here is the version I have found experimentally:

|CAN id | Signal Name               | Units     | r/w   | Bytes | Datatype   |
| ---   | ---                       | ---       | ---   | ---   | ---        |
| 0x01  | speed control (x)         | rad/s     | w     | 8     | double     |
| 0x02  | speed control (y)         | rad/s     | w     | 8     | double     |
| 0x04  | unknown                   |           | r     | 4     |            |
| 0x10  | crane & pendant status    | n/a       | r     | 3     | bool       |
| 0x11  | actual control (x 0)      | rads/s    | r     | 8     | double     |
| 0x12  | actual control (x 1)      | rads/s    | r     | 8     | double     |
| 0x13  | actual control (y)        | rads/s    | r     | 8     | double     |
| 0x20  | sub crane status          | n/a       | r     | 1     | bool       |
| 0x21  | real control (y)          | rads/s    | r     | 8     | double     |
| 0x101 | laser distance (0)        | mm        | r     | 4     | int32      |
| 0x102 | encoder ppr (0)           | counts    | r     | 4     | int32      |
| 0x103 | encoder inc (0)           | counts    | r     | 4     | int32      |
| 0x104 | encoder abs (0)           | counts    | r     | 4     | int32      |
| 0x105 | encoder speed (0)         | counts/s  | r     | 4     | int32      |
| 0x111 | laser distance (1)        | mm        | r     | 4     | int32      |
| 0x112 | encoder ppr (1)           | counts    | r     | 4     | int32      |
| 0x113 | encoder inc (1)           | counts    | r     | 4     | int32      |
| 0x114 | encoder abs (1)           | counts    | r     | 4     | int32      |
| 0x115 | encoder speed (1)         | counts/s  | r     | 4     | int32      |
| 0x122 | encoder ppr (2)           | counts    | r     | 4     | int32      |
| 0x123 | encoder inc (2)           | counts    | r     | 4     | int32      |
| 0x124 | encoder abs (2)           | counts    | r     | 4     | int32      |
| 0x125 | encoder speed (2)         | counts/s  | r     | 4     | int32      |


Notes:
- The original documentation lists messages to reset the encoders on each of the individual motors; if those commands are still accepted, I've not yet figured out what they are. Use the pendant and hold the start button to reset encoders manually for now.
- Limit switches do not stop the crane by default! Do not rely on them. It may still be possible to receive signals from the limit switches.

## Winch System
The serials for the ODrive boards are 208637853548 and 2061377C3548

Sometimes the boards will stop driving their motors. This can happen for a number of reasons, including an emergency stop if the winches draw too much current. This can usually be fixed by restarting the ODrives (turn main power off and on again) and running the `full_init` function found in the [ODrive.py](../src/ODrive.py) file.

## Random Computer Details

### All Computers (Quinn's .bashrc nonsense)
I don't like to type the same commands over and over again, so I've attempted to automate some things. In each of the Computer's ~/.bashrc files, there is a line at the end that sources a ~/catkin_ws/env.sh file. This is where I've placed the important ROS environment variables and such. If you want to change the ROS master to no longer be the Gantry Computer, this is where to do it. NOTE: you'll have to make this change on every computer in the system.

As is, all computers will expect roscore to be running on the Gantry Computer. If you try to start code on, say, the Crane Computer, without starting roscore on the Gantry Computer first, ROS will throw an error.

### The Gantry Computer
The Gantry Computer has been configured to run `sudo ip link set can0 up type can bitrate 500000` on startup (this command is in `init.d`, a script that runs with admin privileges every time the computer starts). You shouldn't ever have to run this command yourself.

Hopefully I can get it to start roscore on launch as well, but we'll get there later.

## Original Documentation:

- [gantry_pendant_use.pdf](gantry_pendant_use.pdf) describes how to start the crane and control it with the pendant.
- [ros_and_simulink_control.pdf](ros_and_simulink_control.pdf) describes how to connect the crane to the ROS network. These instructions are included in README.md. This document also describes how to control the crane via simulink (Not included in README.md) and information about the CAN network (mostly outdated; refer to this document instead). The pdf also has information on how to connect your laptop to the crane's LAN - this is **required** to ssh into any of the Computers and start any of the software
- [crane_utility.pdf](crane_utility.pdf) describes how to use some of Eisuke's control nodes. Don't try to use these in conjunction with the gantry control nodes I have provided. Do refer to them for information about interfacing with the ps4 controllers.
- [wiring_diagram.pdf](wiring_diagram.pdf) describes the wiring of the gantry. As far as I know, this is still up to date, however with the second gantry now in place there may be additions or modifications.
I put the `sudo ip link set can0 up type can bitrate 500000` command in `init.d`. I'm still looking into getting roscore to launch on startup though.