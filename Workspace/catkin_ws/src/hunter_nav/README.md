# Navigation Packages for Hunter Mobile Base

## Packages

* hunter_cartographer: SLAM configurations for Cartographer
* hunter_control: global and local planning with trajectory tracking control

## Setup the Navigation Packages

* Install Google Cartographer standalone version: [instructions](https://google-cartographer.readthedocs.io/en/latest/)
* Download source of cartographer_ros package
* Download source of this repository

```
$ cd <catkin-ws>/src
$ git clone https://github.com/cartographer-project/cartographer_ros.git
$ git clone https://bitbucket.org/westonrobotsoftware/wra_hunter_nav.git
$ cd ..
$ catkin_make
```
Installing and using the standalone version of Cartographer, instead of the [ROS version](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html), avoids building the catkin workspace with "catkin_make_isolated", which may break the compilation of other ROS packages.

## Basic usage of the ROS package

1. Run the robot in mapping mode

    ```
    $ roslaunch hunter_bringup hunter_robot_nav.launch
    $ roslaunch hunter_navigation hunter_3d_mapping.launch
    ```

2. Run the robot in navigation mode

    ```
    $ roslaunch hunter_bringup hunter_robot_nav.launch
    $ roslaunch hunter_navigation hunter_robot_navigation.launch
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

* Start the Webots-based simulation

    ```
    $ roslaunch hunter_bringup hunter_sim_base.launch
    ```
    
**SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 
