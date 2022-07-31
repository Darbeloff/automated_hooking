# Automated Hooking
Code and content for autonomously hooking horizontal pegs with a gantry crane. This project builds heavily on [previous research](https://github.com/Cormac0/Horizontal_Insertion). This code was built for ROS1 Melodic, so no promises about it working on other distros.

For other useful bits of information I refer readers to [everything I know about the crane system](documentation/Everything_I_Know_About_The_Crane.md)

## Software Setup

### 0 - Get Access to the Computers
You'll need to be able to run commands on the system Computers. Technically plugging in a monitor and keyboard is a viable option, but ssh and vnc are much nicer.

- Connect your laptop to the system LAN via any of the ethernet cables lying around. As long as it eventually makes its way back to the Console Box, you'll be connected.
- Refer to [ros_and_simulink_control.pdf](documentation/ros_and_simulink_control.pdf) "Instruction of how to connect the VNC" for how to set up your network adapter. Unfortunately these instructions only apply to Windows; you may have to do some searching for equivalents on Linux and Mac.
- Refer also to the table of ip's, usernames, and passwords in the same document. Relatedly, DO NOT publicize this repo.
- You can now connect to the Computers in any of a couple ways.
  - The easiest is to use VNC as described in the document (you can also use any other VNC app, such as [tightVNC](https://www.tightvnc.com/)). Note that this connection is nicely visual, but often slow and uncomfortable.
  - You can also ssh into the Computer: `ssh [user]@[ip address]`. This method only gives you one terminal to use, but is fast and convenient. If you don't want to type in the password every time, you can add your laptop's ssh key to the remote machine. You can also use `tmux` to effectively get more terminals per ssh connection.
  - If you use vscode, you can use remote explorer to open the remote Computer directly. This will allow you to navigate the filesystem graphically and use handy vscode editing tools. Any terminals opened in vscode (`` Ctrl+` ``, `` Ctrl+Shift+` ``) will open on the target Computer.
### 1 - Install Code on Crane Computer.
In our case this is the rockpi.
The Crane Computer must support odrive 4.(something)
The (optional for this device) realsense packages do not support raspbian, so be sure to use something beefier than a pi.

- Install this code and the `gantry_description` package

    ```
    cd ~/catkin_ws/src
    git clone https://github.com/Darbeloff/gantry_description
    git clone https://github.com/Darbeloff/automated_hooking
    cd ..
    catkin build
    ```

- Install optional dependencies. (technically these only need to be installed on the Vision Computer, see setup step 3 for justification)

    ```
    sudo apt install ros-melodic-realsense2
    sudo apt install ros-melodic-realsense2-description
    ```

    ```
    cd ~/catkin_ws/src
    git clone https://github.com/AprilRobotics/apriltag
    git clone https://github.com/AprilRobotics/apriltag_ros
    cd ..
    catkin build
    ```

### 2 - Install components on the Vision Computer
In our case this is the nvidia jetson.
The (**required** for this device) realsense packages do not support raspbian, so be sure to use something beefier than a pi.

- Install this code and the `gantry_description` package

    ```
    cd ~/catkin_ws/src
    git clone https://github.com/Darbeloff/gantry_description
    git clone https://github.com/Darbeloff/automated_hooking
    cd ..
    catkin build
    ```

- Install realsense packages

    ```
    sudo apt install ros-melodic-realsense2
    sudo apt install ros-melodic-realsense2-description
    ```

- Install apriltag packages

    ```
    cd ~/catkin_ws/src
    git clone https://github.com/AprilRobotics/apriltag
    git clone https://github.com/AprilRobotics/apriltag_ros
    cd ..
    catkin build
    ```


### 3 - Allow Crane Computer to roslaunch remotely onto the Vision Computer
This step is techinically unnecessary, but it allows running the code with fewer ssh sessions and makes me feel nice.

- Add the Crane Computer's ssh key to the Vision Computer's `authorized_hosts` file

    Find the Crane Computer's ssh key: `rockpi> cat ~/.ssh/id_*.pub`
    
    Copy it into the jetson's authorized_hosts file at: `jetson> nano ~/.ssh/authorized_hosts`

- Add the Vision Computer to the Crane Computer's known_hosts file

    `rockpi> ssh -oHostKeyAlgorithms='ssh-rsa' {user}@{vision_computer_ip}`

    This should ssh you onto the Vision Computer without prompting you for a password. the `-oHostKeyAlgorithms` is required because roslaunch only accepts certain types of keys in the known_hosts file; if roslaunching remotely is failing, try deleting known_hosts and trying this part again.

- Ensure the `~/catkin_ws/env_loader.sh` file exists on the Vision Computer. This should already exists, unless someone has destroyed it. This file must define all environment variables (ROS_MASTER_URI, ROS_IP specifically) and source any ros workspace files.

### Notes:
- As usual, all computers in this system must agree about who is the ROS master; make sure that all computers have the same device set as their ROS_MASTER_URI, and that all have their own ip set as the ROS_IP environment variable. This must also be true in any `env_loader.sh` files.

## Running

### 1 - Turn on the Crane
Flip the breaker to On. Move the gantry in all directions to ensure it behaves normally. If the crane jitters or doesnt move in a specific direction, this is often due to the Gantry Computer acting up; try unplugging the gantry computer and starting over.
### 2 - Start Nodes on the Gantry Computer
In our case this is the raspberry pi that lives next to the breaker. More generally, this is the node connected to arduino CAN that drives the gantry.

- Ensure `roscore` is running

    `pi> nohup roscore &`

    This command starts a copy of roscore in the background, that will continue running even if this terminal is closed. 

- Start the CAN bridge nodes

    Try simply running `pi> can_node`.

    If this doesnt work, run these three instead:
    ```
    pi> sudo ip link set can0 up type can bitrate 500000
    pi> rosrun socketcan_bridge topic_to_socketcan_node can0
    pi> rosrun socketcan_bridge socketcan_to_topic_node can0
    ```

### 3 - Start all other nodes (if you did the optional setup steps)
- Start all other nodes from the Crane Computer

    `rockpi> roslaunch automated_hooking all_nodes.launch`

    Tip: `rostopic list` and `rosnode list` are useful for sanity-checking that all nodes did indeed start.
### 3 - Start all other nodes (if you did NOT do the optional setup steps)
- Start camera nodes on the Camera Computer

    `jetson> roslaunch automated_hooking camera.launch`

- Start all remaining nodes on the Crane Computer

    `rockpi> roslaunch crane.launch`



## Parts and Manufacture

[Parts List](documentation/Parts_List.md)

Manufacture is TODO.