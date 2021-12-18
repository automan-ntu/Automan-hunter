// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hunter_msgs:msg/HunterStatus.idl
// generated code does not contain a copyright notice

#ifndef HUNTER_MSGS__MSG__DETAIL__HUNTER_STATUS__TRAITS_HPP_
#define HUNTER_MSGS__MSG__DETAIL__HUNTER_STATUS__TRAITS_HPP_

#include "hunter_msgs/msg/detail/hunter_status__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'motor_states'
#include "hunter_msgs/msg/detail/hunter_motor_state__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hunter_msgs::msg::HunterStatus>()
{
  return "hunter_msgs::msg::HunterStatus";
}

template<>
inline const char * name<hunter_msgs::msg::HunterStatus>()
{
  return "hunter_msgs/msg/HunterStatus";
}

template<>
struct has_fixed_size<hunter_msgs::msg::HunterStatus>
  : std::integral_constant<bool, has_fixed_size<hunter_msgs::msg::HunterMotorState>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<hunter_msgs::msg::HunterStatus>
  : std::integral_constant<bool, has_bounded_size<hunter_msgs::msg::HunterMotorState>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<hunter_msgs::msg::HunterStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUNTER_MSGS__MSG__DETAIL__HUNTER_STATUS__TRAITS_HPP_
