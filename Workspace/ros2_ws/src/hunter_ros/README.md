# ROS Packages for Hunter Mobile Base

## Packages

* hunter_base: a ROS wrapper around Hunter SDK to monitor and control the robot
* hunter_bringup: launch and configuration files to start ROS nodes 
* hunter_description: urdf models
* hunter_msgs: hunter related message definitions
* (hunter_ros: meta package for the Scout robot ROS packages)
* hunter_webots_sim: webot-based simulator for hunter

## Communication interface setup

Please refer to the [README](https://github.com/westonrobot/wrp_sdk#hardware-interface) of "wrp_sdk" package for setup of communication interfaces.

#### Note on CAN interface on Nvidia Jetson Platforms

Nvidia Jeston TX2/Xavier/XavierNX have CAN controller(s) integrated in the main SOC. If you're using a dev kit, you need to add a CAN transceiver for proper CAN communication. 

## Basic usage of the ROS package

1. Install dependent ROS packages

    ```
    $ sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
    $ sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher-gui
    $ sudo apt install ros-$ROS_DISTRO-ros-controllers
    $ sudo apt install ros-$ROS_DISTRO-webots-ros
    ```

2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/westonrobot/wrp_sdk.git
    $ git clone https://github.com/westonrobot/hunter_ros.git
    $ cd ..
    $ catkin_make
    ```

3. Setup Webots simulation    

* Install Webots R2020a-rev1 (download from https://cyberbotics.com/ )

* Set WEBOTS_HOME variable, add the following line to your "~/.bashrc"

    ```
    export WEBOTS_HOME=/usr/local/webots
    ```

    Adjust the path accordingly if you installed Webots to a different place.

4. Launch ROS nodes
 
* Start the base node for the real robot

    ```
    $ roslaunch hunter_bringup hunter_robot_base.launch
    ```

* Start the base node for the Webots-based simulator

    ```
    $ roslaunch hunter_bringup hunter_sim_base.launch
    ```

* Start the simulator in which basic navigation sensors are set up
    
    ```
    $ roslaunch hunter_bringup hunter_sim_nav_indoor.launch
    ```
    
**SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 
