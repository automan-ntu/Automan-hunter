/*
 * scout_base_ros.hpp
 *
 * Created on: Dec 18, 2021 17:40
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef HUNTER_BASE_ROS_HPP
#define HUNTER_BASE_ROS_HPP

#include <atomic>
#include <memory>
#include <string>

//ros2 headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>

#include "hunter_base/hunter_messenger.hpp"
#include "wrp_sdk/wrp_sdk/platforms/hunter/hunter_base.hpp"

namespace westonrobot {
class HunterBaseRos : public rclcpp::Node {
 public:
  HunterBaseRos(std::string node_name);

  bool Initialize();
  void Run();
  void Stop();

 private:
  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  int control_rate_;

  HunterBase *robot_;


  std::atomic<bool> keep_running_;

  void LoadParameters();
};
}  // namespace westonrobot

#endif /* HUNTER_BASE_ROS_HPP */