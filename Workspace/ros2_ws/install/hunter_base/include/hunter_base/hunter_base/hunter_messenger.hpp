/*
 * hunter_messenger.hpp
 *
 * Created on: Jun 01, 2020 15:18
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef HUNTER_MESSENGER_HPP
#define HUNTER_MESSENGER_HPP

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>
#include <memory>
#include <functional>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//#include "/wrp_sdk/platforms/hunter/hunter_base.hpp"
#include "wrp_sdk/wrp_sdk/platforms/hunter/hunter_base.hpp"
#include "ascent/Ascent.h"
#include "ascent/Utility.h"
#include "hunter_base/bicycle_model.hpp"
#include "hunter_base/hunter_params.hpp"

#include "hunter_msgs/msg/hunter_status.hpp"

namespace westonrobot {
template <typename SystemModel>
class SystemPropagator {
 public:
  asc::state_t Propagate(asc::state_t init_state,
                         typename SystemModel::control_t u, double t0,
                         double tf, double dt) {
    double t = t0;
    asc::state_t x = init_state;

    while (t <= tf) {
      integrator_(SystemModel(u), x, t, dt);
      // Note: you may need to add additional constraints to [x]
    }

    return x;
  }

 private:
  asc::RK4 integrator_;
};

class HunterROSMessenger {
 public:
  HunterROSMessenger(HunterBase *hunter, rclcpp::Node *node);

  std::string odom_frame_;
  std::string base_frame_;

  int control_rate_ = 50;
  void SetOdometryFrame(std::string frame) { odom_frame_ = frame; }
  void SetBaseFrame(std::string frame) { base_frame_ = frame; }

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);

  void SetupSubscription();
  void ResetOdometry();

  void PublishStateToROS();

 private:
  HunterBase *hunter_;
  //std::shared_ptr<SystemModel> hunter_;
  rclcpp::Node *node_;

  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist current_twist_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<hunter_msgs::msg::HunterStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr integrator_reset_subscriber_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // control inputs
  double linear_speed_ = 0.0;
  double steering_angle_ = 0.0;

  static constexpr double l = HunterParams::wheelbase;
  static constexpr double w = HunterParams::track;
  static constexpr double steer_angle_tolerance = 0.005;  // ~+-0.287 degrees

  // state variables
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  SystemPropagator<BicycleKinematics> model_;

  rclcpp::Time last_time_;
  rclcpp::Time current_time_;

  double ConvertInnerAngleToCentral(double angle);
  double ConvertCentralAngleToInner(double angle);

  void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void ResetOdomIntegratorCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void PublishOdometryToROS(double linear, double angular, double dt);

};
}  // namespace westonrobot

#endif /* HUNTER_MESSENGER_HPP */
