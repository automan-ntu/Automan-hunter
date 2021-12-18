// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hunter_msgs:msg/HunterMotorState.idl
// generated code does not contain a copyright notice

#ifndef HUNTER_MSGS__MSG__DETAIL__HUNTER_MOTOR_STATE__BUILDER_HPP_
#define HUNTER_MSGS__MSG__DETAIL__HUNTER_MOTOR_STATE__BUILDER_HPP_

#include "hunter_msgs/msg/detail/hunter_motor_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hunter_msgs
{

namespace msg
{

namespace builder
{

class Init_HunterMotorState_temperature
{
public:
  explicit Init_HunterMotorState_temperature(::hunter_msgs::msg::HunterMotorState & msg)
  : msg_(msg)
  {}
  ::hunter_msgs::msg::HunterMotorState temperature(::hunter_msgs::msg::HunterMotorState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hunter_msgs::msg::HunterMotorState msg_;
};

class Init_HunterMotorState_rpm
{
public:
  explicit Init_HunterMotorState_rpm(::hunter_msgs::msg::HunterMotorState & msg)
  : msg_(msg)
  {}
  Init_HunterMotorState_temperature rpm(::hunter_msgs::msg::HunterMotorState::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return Init_HunterMotorState_temperature(msg_);
  }

private:
  ::hunter_msgs::msg::HunterMotorState msg_;
};

class Init_HunterMotorState_current
{
public:
  Init_HunterMotorState_current()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HunterMotorState_rpm current(::hunter_msgs::msg::HunterMotorState::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_HunterMotorState_rpm(msg_);
  }

private:
  ::hunter_msgs::msg::HunterMotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hunter_msgs::msg::HunterMotorState>()
{
  return hunter_msgs::msg::builder::Init_HunterMotorState_current();
}

}  // namespace hunter_msgs

#endif  // HUNTER_MSGS__MSG__DETAIL__HUNTER_MOTOR_STATE__BUILDER_HPP_
