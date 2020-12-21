/*
 * ultrasonic_ros_node.cpp
 *
 * Created on: Jul 02, 2020 19:14
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "ultrasonic_ros/ultrasonic_ros.hpp"

using namespace westonrobot;

int main(int argc, char *argv[]) {
  // setup ROS node
  ros::init(argc, argv, "rc_ctrl");
  ros::NodeHandle node(""), private_node("~");

  // fetch parameters before connecting to robot
  std::string port_name;
  std::string pub_topic_name;

  private_node.param<std::string>("port_name", port_name, std::string("can0"));
  private_node.param<std::string>("pub_topic", pub_topic_name,
                                  std::string("ultrasonic"));

  // instantiate a rc2can messenger object
  UltrasonicRos messenger(&node, port_name);
  messenger.SetPubTopicName(pub_topic_name);

  messenger.Run();

  return 0;
}