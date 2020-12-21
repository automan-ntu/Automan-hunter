/*
 * ultrasonic_ros.hpp
 *
 * Created on: Jul 02, 2020 19:14
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef ULTRASONIC_ROS_HPP
#define ULTRASONIC_ROS_HPP

#include <ros/ros.h>

#include <cstdint>

#include "wrp_sdk/asyncio/async_can.hpp"
#include "wrp_sdk/asyncio/async_serial.hpp"

namespace westonrobot {
class UltrasonicRos {
 public:
  UltrasonicRos(ros::NodeHandle* nh, std::string dev_name = "can0");

  void SetPubTopicName(std::string name);

  void Run();

 private:
  ros::NodeHandle* nh_;
  ros::Publisher ultrasonic_pub_[8];

  std::shared_ptr<ASyncCAN> can_if_;
  bool can_connected_ = false;

  std::shared_ptr<ASyncSerial> serial_if_;
  bool serial_connected_ = false;

  std::string ultrasonic_topic_name_;

  void ParseCANFrame(can_frame* rx_frame);
  void ParseUARTBuffer(uint8_t* buf, const size_t bufsize,
                       size_t bytes_received);
};
}  // namespace westonrobot

#endif /* ULTRASONIC_ROS_HPP */
