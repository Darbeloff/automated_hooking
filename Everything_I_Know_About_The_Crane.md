# Everything I Know About The Crane

Last updated 7/29/22. Everything still seems to be true.

## The Jitter Glitch
Sometimes the crane doesnt move in particular directions, or at least doesn't seem to enjoy it. Unplug the Gantry Computer and reboot the whole system. I don't know why but this seems to help. (My suspicion is the topic_to_socketcan_node is sending bad commands to the CAN and causing it to freak out, but I'm unsure if this is true; I still need to find the original source code for the CAN)

## The CAN
The CAN table in Eisuke's helpful manual is unfortunately out of date. Here is the version I have found experimentally:

|CAN id | Signal Name           | Units     | r/w   | Bytes | Datatype   |
| ---   | ---                   | ---       | ---   | ---   | ---        |
| 0x01  | speed control (x)     | rad/s     | w     | 8     | double     |
| 0x02  | speed control (y)     | rad/s     | w     | 8     | double     |
| 0x04  | unknown               |           | r     | 4     |            |
| 0x10  | crane & pendant status| n/a       | r     | 3     | bool       |
| 0x11  | real control (x 0)    | rads/s    | r     | 8     | double     |
| 0x12  | real control (x 1)    | rads/s    | r     | 8     | double     |
| 0x13  | real control (y)      | rads/s    | r     | 8     | double     |
| 0x20  | sub crane status      | n/a       | r     | 1     | bool       |
| 0x21  | real control (y)      | rads/s    | r     | 8     | double     |
| 0x101 | laser distance (0)    | mm        | r     | 4     | int32      |
| 0x102 | encoder ppr (0)       | counts    | r     | 4     | int32      |
| 0x103 | encoder inc (0)       | counts    | r     | 4     | int32      |
| 0x104 | encoder abs (0)       | counts    | r     | 4     | int32      |
| 0x105 | encoder speed (0)     | counts/s  | r     | 4     | int32      |
| 0x111 | laser distance (1)    | mm        | r     | 4     | int32      |
| 0x112 | encoder ppr (1)       | counts    | r     | 4     | int32      |
| 0x113 | encoder inc (1)       | counts    | r     | 4     | int32      |
| 0x114 | encoder abs (1)       | counts    | r     | 4     | int32      |
| 0x115 | encoder speed (1)     | counts/s  | r     | 4     | int32      |
| 0x122 | encoder ppr (2)       | counts    | r     | 4     | int32      |
| 0x123 | encoder inc (2)       | counts    | r     | 4     | int32      |
| 0x124 | encoder abs (2)       | counts    | r     | 4     | int32      |
| 0x125 | encoder speed (2)     | counts/s  | r     | 4     | int32      |

## Random Computer Details

### All Computers (Quinn's .bashrc nonsense)
I don't like to type the same commands over and over again, so I've attempted to automate some things. In each of the Computer's ~/.bashrc files, there is a line at the end that sources a ~/catkin_ws/env.sh file. This is where I've placed the important ROS environment variables and such. If you want to change the ROS master to no longer be the Gantry Computer, this is where to do it. NOTE: you'll have to make this change on every computer in the system.

### The Gantry Computer

I put the `sudo ip link set can0 up type can bitrate 500000` command in `init.d`. I'm still looking into getting roscore to launch on startup though.