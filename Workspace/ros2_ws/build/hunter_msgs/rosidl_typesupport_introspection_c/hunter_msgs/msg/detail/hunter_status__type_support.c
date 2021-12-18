// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hunter_msgs:msg/HunterStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hunter_msgs/msg/detail/hunter_status__rosidl_typesupport_introspection_c.h"
#include "hunter_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hunter_msgs/msg/detail/hunter_status__functions.h"
#include "hunter_msgs/msg/detail/hunter_status__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `motor_states`
#include "hunter_msgs/msg/hunter_motor_state.h"
// Member `motor_states`
#include "hunter_msgs/msg/detail/hunter_motor_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void HunterStatus__rosidl_typesupport_introspection_c__HunterStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hunter_msgs__msg__HunterStatus__init(message_memory);
}

void HunterStatus__rosidl_typesupport_introspection_c__HunterStatus_fini_function(void * message_memory)
{
  hunter_msgs__msg__HunterStatus__fini(message_memory);
}

size_t HunterStatus__rosidl_typesupport_introspection_c__size_function__HunterMotorState__motor_states(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * HunterStatus__rosidl_typesupport_introspection_c__get_const_function__HunterMotorState__motor_states(
  const void * untyped_member, size_t index)
{
  const hunter_msgs__msg__HunterMotorState ** member =
    (const hunter_msgs__msg__HunterMotorState **)(untyped_member);
  return &(*member)[index];
}

void * HunterStatus__rosidl_typesupport_introspection_c__get_function__HunterMotorState__motor_states(
  void * untyped_member, size_t index)
{
  hunter_msgs__msg__HunterMotorState ** member =
    (hunter_msgs__msg__HunterMotorState **)(untyped_member);
  return &(*member)[index];
}

static rosidl_typesupport_introspection_c__MessageMember HunterStatus__rosidl_typesupport_introspection_c__HunterStatus_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hunter_msgs__msg__HunterStatus, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hunter_msgs__msg__HunterStatus, linear_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "steering_angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hunter_msgs__msg__HunterStatus, steering_angle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "base_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hunter_msgs__msg__HunterStatus, base_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "control_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hunter_msgs__msg__HunterStatus, control_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fault_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hunter_msgs__msg__HunterStatus, fault_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "battery_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hunter_msgs__msg__HunterStatus, battery_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hunter_msgs__msg__HunterStatus, motor_states),  // bytes offset in struct
    NULL,  // default value
    HunterStatus__rosidl_typesupport_introspection_c__size_function__HunterMotorState__motor_states,  // size() function pointer
    HunterStatus__rosidl_typesupport_introspection_c__get_const_function__HunterMotorState__motor_states,  // get_const(index) function pointer
    HunterStatus__rosidl_typesupport_introspection_c__get_function__HunterMotorState__motor_states,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers HunterStatus__rosidl_typesupport_introspection_c__HunterStatus_message_members = {
  "hunter_msgs__msg",  // message namespace
  "HunterStatus",  // message name
  8,  // number of fields
  sizeof(hunter_msgs__msg__HunterStatus),
  HunterStatus__rosidl_typesupport_introspection_c__HunterStatus_message_member_array,  // message members
  HunterStatus__rosidl_typesupport_introspection_c__HunterStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  HunterStatus__rosidl_typesupport_introspection_c__HunterStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t HunterStatus__rosidl_typesupport_introspection_c__HunterStatus_message_type_support_handle = {
  0,
  &HunterStatus__rosidl_typesupport_introspection_c__HunterStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hunter_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hunter_msgs, msg, HunterStatus)() {
  HunterStatus__rosidl_typesupport_introspection_c__HunterStatus_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  HunterStatus__rosidl_typesupport_introspection_c__HunterStatus_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hunter_msgs, msg, HunterMotorState)();
  if (!HunterStatus__rosidl_typesupport_introspection_c__HunterStatus_message_type_support_handle.typesupport_identifier) {
    HunterStatus__rosidl_typesupport_introspection_c__HunterStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &HunterStatus__rosidl_typesupport_introspection_c__HunterStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
