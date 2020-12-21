/*
 * ultrasonic_ros.cpp
 *
 * Created on: Jul 02, 2020 19:13
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "ultrasonic_ros/ultrasonic_ros.hpp"
#include "ultrasonic_ros/w200d_ultrasonic.h"

#include <sensor_msgs/Range.h>

namespace westonrobot {
UltrasonicRos::UltrasonicRos(ros::NodeHandle* nh, std::string dev_name)
    : nh_(nh) {
  std::size_t found_can = dev_name.find("can");
  if (found_can != std::string::npos) {
    can_if_ = std::make_shared<ASyncCAN>(dev_name);
    can_if_->set_receive_callback(
        std::bind(&UltrasonicRos::ParseCANFrame, this, std::placeholders::_1));
    can_connected_ = true;
  } else {
    serial_if_ = std::make_shared<ASyncSerial>(dev_name, 115200);
    serial_if_->open();
    if (serial_if_->is_open()) serial_connected_ = true;
    serial_if_->set_receive_callback(
        std::bind(&UltrasonicRos::ParseUARTBuffer, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));
  }
}

void UltrasonicRos::SetPubTopicName(std::string name) {
  ultrasonic_topic_name_ = name;
  for (int i = 0; i < 8; ++i) {
    ultrasonic_pub_[i] = nh_->advertise<sensor_msgs::Range>(
        ultrasonic_topic_name_ + std::to_string(i), 10);
  }
}

void UltrasonicRos::ParseCANFrame(can_frame* rx_frame) {
  static uint32_t ultrasonic_frame_seq = 0;

  // ultrasonic CAN frame
  if (rx_frame->can_id == 0x601) {
    std::cout << "received can message: ";
    for (int i = 0; i < rx_frame->can_dlc; ++i) {
      std::cout << (int32_t)rx_frame->data[i] << " ";
    }
    std::cout << std::endl;

    ultrasonic_frame_seq++;
    sensor_msgs::Range us_msg;
    // header
    us_msg.header.seq = ultrasonic_frame_seq;
    us_msg.header.stamp = ros::Time::now();
    // common data fields
    us_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    us_msg.field_of_view = M_PI / 6.0;
    us_msg.min_range = 0.2;
    us_msg.max_range = 0.9;
    for (int i = 0; i < 8; ++i) {
      us_msg.header.frame_id = "ultrasonic" + std::to_string(i);
      us_msg.range = rx_frame->data[i] / 100.0;
      ultrasonic_pub_[i].publish(us_msg);
    }
  }
}

void UltrasonicRos::ParseUARTBuffer(uint8_t* buf, const size_t bufsize,
                                    size_t bytes_received) {
  W200dUltrasonicMessage msg;
  static uint32_t ultrasonic_frame_seq = 0;
  for (int k = 0; k < bytes_received; ++k) {
    if (W200dDecodeMessage(buf[k], &msg)) {
      std::cout << "received uart message: ";
      for (int i = 0; i < 8; ++i) {
        std::cout << (int32_t)msg.channels[i] << " ";
      }
      std::cout << std::endl;
      
      ultrasonic_frame_seq++;
      sensor_msgs::Range us_msg;
      // header
      us_msg.header.seq = ultrasonic_frame_seq;
      us_msg.header.stamp = ros::Time::now();
      // common data fields
      us_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
      us_msg.field_of_view = M_PI / 6.0;
      us_msg.min_range = 0.2;
      us_msg.max_range = 0.9;
      for (int i = 0; i < 8; ++i) {
        us_msg.header.frame_id = "ultrasonic" + std::to_string(i);
        us_msg.range = msg.channels[i] / 100.0;
        ultrasonic_pub_[i].publish(us_msg);
      }
    }
  }
}

void UltrasonicRos::Run() {
  ros::Rate rate_20hz(20);
  while (ros::ok()) {
    ros::spinOnce();
    rate_20hz.sleep();
  }
}

}  // namespace westonrobot