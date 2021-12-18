// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from hunter_msgs:msg/HunterMotorState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "hunter_msgs/msg/detail/hunter_motor_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hunter_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void HunterMotorState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hunter_msgs::msg::HunterMotorState(_init);
}

void HunterMotorState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hunter_msgs::msg::HunterMotorState *>(message_memory);
  typed_message->~HunterMotorState();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember HunterMotorState_message_member_array[3] = {
  {
    "current",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hunter_msgs::msg::HunterMotorState, current),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rpm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hunter_msgs::msg::HunterMotorState, rpm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "temperature",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hunter_msgs::msg::HunterMotorState, temperature),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers HunterMotorState_message_members = {
  "hunter_msgs::msg",  // message namespace
  "HunterMotorState",  // message name
  3,  // number of fields
  sizeof(hunter_msgs::msg::HunterMotorState),
  HunterMotorState_message_member_array,  // message members
  HunterMotorState_init_function,  // function to initialize message memory (memory has to be allocated)
  HunterMotorState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t HunterMotorState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &HunterMotorState_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace hunter_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hunter_msgs::msg::HunterMotorState>()
{
  return &::hunter_msgs::msg::rosidl_typesupport_introspection_cpp::HunterMotorState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hunter_msgs, msg, HunterMotorState)() {
  return &::hunter_msgs::msg::rosidl_typesupport_introspection_cpp::HunterMotorState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
