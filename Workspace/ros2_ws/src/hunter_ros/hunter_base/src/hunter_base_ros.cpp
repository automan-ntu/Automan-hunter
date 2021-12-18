/*
 * hunter_base_ros.cpp
 *
 * Created on: Oct 18, 2021 17:47
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "hunter_base/hunter_base_ros.hpp"

namespace westonrobot {  
HunterBaseRos::HunterBaseRos(std::string node_name)
    : rclcpp::Node(node_name), keep_running_(false) {

  this->declare_parameter("port_name");
  this->declare_parameter("odom_frame");
  this->declare_parameter("base_frame");
  this->declare_parameter("control_rate");

  LoadParameters();
}

void HunterBaseRos::LoadParameters() {
  this->get_parameter_or<std::string>("port_name", port_name_, "can0");

  this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
  this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
  this->get_parameter_or<int>("control_rate", control_rate_, 50);

  std::cout << "Loading parameters: " << std::endl;
  std::cout << "- port name: " << port_name_ << std::endl;
  std::cout << "- odom frame name: " << odom_frame_ << std::endl;
  std::cout << "- base frame name: " << base_frame_ << std::endl;
  std::cout << "- control rate: " << control_rate_ << std::endl;
  std::cout << "----------------------------" << std::endl;
}

bool HunterBaseRos::Initialize() {
    return true;
}

void HunterBaseRos::Stop() { keep_running_ = false; }

void HunterBaseRos::Run() {
  // instantiate a ROS messenger
    westonrobot::HunterROSMessenger *messenger = new westonrobot::HunterROSMessenger(robot_, this);

    messenger->SetOdometryFrame(odom_frame_);
    messenger->SetBaseFrame(base_frame_);

    // connect to robot and setup ROS subscription
    if (port_name_.find("can") != std::string::npos) {
        robot_->Connect(port_name_);
        std::cout << "Using CAN bus to talk with the robot" << std::endl;
    } else {
        std::cout << "Please check the specified port name is a CAN port"
                << std::endl;
        return;
        }

    // publish robot state at 50Hz while listening to twist commands
    messenger->SetupSubscription();
    keep_running_ = true;
    rclcpp::Rate rate(50);
    while (keep_running_) {
        messenger->PublishStateToROS();
        rclcpp::spin_some(shared_from_this());
        rate.sleep();
        }
}
}  // namespace westonrobot