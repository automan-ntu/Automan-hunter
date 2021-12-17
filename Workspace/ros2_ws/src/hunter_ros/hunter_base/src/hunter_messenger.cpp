/*
 * hunter_messenger.cpp
 *
 * Created on: Jun 01, 2020 15:25
 * Description:
 *
 * Test commands:
 * rostopic pub /cmd_vel geometry_msgs/Twist -r 3 -- '[0.5,0.0,0.0]' '[0.0, 0.0,
 * 0.3872]'
 *
 * rostopic pub /reset_odom_integrator std_msgs/Bool true
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */



#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include "hunter_base/hunter_messenger.hpp"
#include "hunter_msgs/msg/HunterStatus.hpp"

namespace westonrobot {
HunterROSMessenger::HunterROSMessenger(rclcpp::Node::SharedPtr *node)
    : hunter_(nullptr), node_(node) {}

HunterROSMessenger::HunterROSMessenger(HunterBase *hunter, rclcpp::Node::SharedPtr *node)
    : hunter_(hunter), node_(node) {}

void HunterROSMessenger::SetupSubscription() {
  // odometry publisher
  auto odom_publisher_ = node_->create_pulisher<nav_msgs::msg::Odometry>(odom_frame_, 50);
  auto status_publisher_ =
      node_->create_publisher<hunter_msgs::msg::HunterStatus>("/hunter_status", 10);

  // cmd subscriber
  auto motion_cmd_subscriber_ = node_->create_subcription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 5, std::bind(&HunterROSMessenger::TwistCmdCallback, this, _1));
  auto integrator_reset_subscriber_ = node_->create_subcription<std_msgs::msg::Bool>(
      "/reset_odom_integrator", 5,
      std::bind(&HunterROSMessenger::ResetOdomIntegratorCallback, this, _1));
}

void HunterROSMessenger::ResetOdomIntegratorCallback(
    const std_msgs::msg::Bool::ConstPtr &msg) {
  if (msg->data) ResetOdometry();
}

void HunterROSMessenger::TwistCmdCallback(
    const geometry_msgs::msg::Twist::ConstPtr &msg) {
  double steer_cmd = msg->angular.z;
  if(steer_cmd > HunterParams::max_steer_angle_central)
    steer_cmd = HunterParams::max_steer_angle_central;
  if(steer_cmd < - HunterParams::max_steer_angle_central)
      steer_cmd = - HunterParams::max_steer_angle_central;

  // TODO add cmd limits here
  if (!simulated_robot_) {
    double phi_i = ConvertCentralAngleToInner(msg->angular.z);
    // std::cout << "set steering angle: " << phi_i << std::endl;
    hunter_->SetMotionCommand(msg->linear.x, phi_i);
  } else {
    std::lock_guard<std::mutex> guard(twist_mutex_);
    current_twist_ = *msg.get();
  }
  // ROS_INFO("cmd received:%f, %f", msg->linear.x, msg->angular.z);
}

void HunterROSMessenger::GetCurrentMotionCmdForSim(double &linear,
                                                   double &angular) {
  std::lock_guard<std::mutex> guard(twist_mutex_);
  linear = current_twist_.linear.x;
  angular = current_twist_.angular.z;
}

double HunterROSMessenger::ConvertInnerAngleToCentral(double angle) {
  double phi = 0;
  double phi_i = angle;
  if (phi_i > steer_angle_tolerance) {
    // left turn
    double r = l / std::tan(phi_i) + w / 2;
    phi = std::atan(l / r);
  } else if (phi_i < -steer_angle_tolerance) {
    // right turn
    double r = l / std::tan(-phi_i) + w / 2;
    phi = std::atan(l / r);
    phi = -phi;
  }
  return phi;
}

double HunterROSMessenger::ConvertCentralAngleToInner(double angle) {
  double phi = angle;
  double phi_i = 0;
  if (phi > steer_angle_tolerance) {
    // left turn
    phi_i = std::atan(2 * l * std::sin(phi) /
                      (2 * l * std::cos(phi) - w * std::sin(phi)));
  } else if (phi < -steer_angle_tolerance) {
    // right turn
    phi = -phi;
    phi_i = std::atan(2 * l * std::sin(phi) /
                      (2 * l * std::cos(phi) - w * std::sin(phi)));
    phi_i = -phi_i;
  }
  return phi_i;
}

void HunterROSMessenger::PublishStateToROS() {
  current_time_ = rclcpp::Time::now();
  double dt = (current_time_ - last_time_).toSec();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = hunter_->GetHunterState();

  // publish hunter state message
  hunter_msgs::msg::HunterStatus status_msg;

  status_msg.header.stamp = current_time_;

  double left_vel = -state.motor_states[1].rpm / 60.0 * 2 * M_PI /
                    HunterParams::transmission_reduction_rate *
                    HunterParams::wheel_radius;
  double right_vel = state.motor_states[2].rpm / 60.0 * 2 * M_PI /
                     HunterParams::transmission_reduction_rate *
                     HunterParams::wheel_radius;
  status_msg.linear_velocity = (left_vel + right_vel) / 2.0;

  // TODO SHOULD NOT use this correction when new Firmware with right 
  //     cmd/feedback of steering angle is updated
  double corrected_angle = state.steering_angle * 26.5 / 40.0;
  double phi = ConvertInnerAngleToCentral(corrected_angle);
  //   double phi = ConvertInnerAngleToCentral(state.steering_angle);
  status_msg.steering_angle = phi;

  status_msg.base_state = state.base_state;
  status_msg.control_mode = state.control_mode;
  status_msg.fault_code = state.fault_code;
  status_msg.battery_voltage = state.battery_voltage;

  for (int i = 0; i < 3; ++i) {
    status_msg.motor_states[i].current = state.motor_states[i].current;
    status_msg.motor_states[i].rpm = state.motor_states[i].rpm;
    status_msg.motor_states[i].temperature = state.motor_states[i].temperature;
  }

  status_publisher_->publish(status_msg);

  // publish odometry and tf
  PublishOdometryToROS(state.linear_velocity, status_msg.steering_angle, dt);

  // record time for next integration
  last_time_ = current_time_;
}

void HunterROSMessenger::PublishSimStateToROS(double linear, double angular) {
  current_time_ = rclcpp::Time::now();
  double dt = 1.0 / sim_control_rate_;

  // publish hunter state message
  hunter_msgs::HunterStatus status_msg;

  status_msg.header.stamp = current_time_;

  // TODO should receive update from simulator
  status_msg.linear_velocity = linear;
  status_msg.steering_angle = angular;

  status_msg.base_state = 0x00;
  status_msg.control_mode = 0x01;
  status_msg.fault_code = 0x00;
  status_msg.battery_voltage = 29.5;

  for (int i = 0; i < 3; ++i) {
    status_msg.motor_states[i].current = 0;
    status_msg.motor_states[i].rpm = 0;
    status_msg.motor_states[i].temperature = 0;
  }

  status_publisher_->publish(status_msg);

  // publish odometry and tf
  PublishOdometryToROS(linear, angular, dt);
}

void HunterROSMessenger::ResetOdometry() {
  position_x_ = 0.0;
  position_y_ = 0.0;
  theta_ = 0.0;
}

void HunterROSMessenger::PublishOdometryToROS(double linear, double angular,
                                              double dt) {
  // perform numerical integration to get an estimation of pose
  linear_speed_ = linear;
  steering_angle_ = angular;

  // propagate model model
  asc::state_t state =
      model_.Propagate({position_x_, position_y_, theta_},
                       {linear_speed_, steering_angle_}, 0, dt, dt / 100);
  position_x_ = state[0];
  position_y_ = state[1];
  theta_ = state[2];

  geometry_msgs::msg::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

  // publish tf transformation
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = current_time_;
  tf_msg.header.frame_id = odom_frame_;
  tf_msg.child_frame_id = base_frame_;

  tf_msg.transform.translation.x = position_x_;
  tf_msg.transform.translation.y = position_y_;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = odom_quat;

  tf_broadcaster_.sendTransform(tf_msg);

  // publish odometry and tf messages
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.twist.twist.linear.x = linear_speed_;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = steering_angle_;

//   std::cerr << "linear: " << linear_speed_ << " , angular: " << steering_angle_
//             << " , pose: (" << position_x_ << "," << position_y_ << ","
//             << theta_ << ")" << std::endl;

  odom_publisher_->publish(odom_msg);
}
}  // namespace westonrobot