// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hunter_msgs:msg/HunterMotorState.idl
// generated code does not contain a copyright notice

#ifndef HUNTER_MSGS__MSG__DETAIL__HUNTER_MOTOR_STATE__STRUCT_HPP_
#define HUNTER_MSGS__MSG__DETAIL__HUNTER_MOTOR_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hunter_msgs__msg__HunterMotorState __attribute__((deprecated))
#else
# define DEPRECATED__hunter_msgs__msg__HunterMotorState __declspec(deprecated)
#endif

namespace hunter_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HunterMotorState_
{
  using Type = HunterMotorState_<ContainerAllocator>;

  explicit HunterMotorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current = 0.0;
      this->rpm = 0.0;
      this->temperature = 0.0;
    }
  }

  explicit HunterMotorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current = 0.0;
      this->rpm = 0.0;
      this->temperature = 0.0;
    }
  }

  // field types and members
  using _current_type =
    double;
  _current_type current;
  using _rpm_type =
    double;
  _rpm_type rpm;
  using _temperature_type =
    double;
  _temperature_type temperature;

  // setters for named parameter idiom
  Type & set__current(
    const double & _arg)
  {
    this->current = _arg;
    return *this;
  }
  Type & set__rpm(
    const double & _arg)
  {
    this->rpm = _arg;
    return *this;
  }
  Type & set__temperature(
    const double & _arg)
  {
    this->temperature = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hunter_msgs::msg::HunterMotorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const hunter_msgs::msg::HunterMotorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hunter_msgs::msg::HunterMotorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hunter_msgs::msg::HunterMotorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hunter_msgs::msg::HunterMotorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hunter_msgs::msg::HunterMotorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hunter_msgs::msg::HunterMotorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hunter_msgs::msg::HunterMotorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hunter_msgs::msg::HunterMotorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hunter_msgs::msg::HunterMotorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hunter_msgs__msg__HunterMotorState
    std::shared_ptr<hunter_msgs::msg::HunterMotorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hunter_msgs__msg__HunterMotorState
    std::shared_ptr<hunter_msgs::msg::HunterMotorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HunterMotorState_ & other) const
  {
    if (this->current != other.current) {
      return false;
    }
    if (this->rpm != other.rpm) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    return true;
  }
  bool operator!=(const HunterMotorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HunterMotorState_

// alias to use template instance with default allocator
using HunterMotorState =
  hunter_msgs::msg::HunterMotorState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hunter_msgs

#endif  // HUNTER_MSGS__MSG__DETAIL__HUNTER_MOTOR_STATE__STRUCT_HPP_
