// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hunter_msgs:msg/HunterMotorState.idl
// generated code does not contain a copyright notice

#ifndef HUNTER_MSGS__MSG__DETAIL__HUNTER_MOTOR_STATE__STRUCT_H_
#define HUNTER_MSGS__MSG__DETAIL__HUNTER_MOTOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/HunterMotorState in the package hunter_msgs.
typedef struct hunter_msgs__msg__HunterMotorState
{
  double current;
  double rpm;
  double temperature;
} hunter_msgs__msg__HunterMotorState;

// Struct for a sequence of hunter_msgs__msg__HunterMotorState.
typedef struct hunter_msgs__msg__HunterMotorState__Sequence
{
  hunter_msgs__msg__HunterMotorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hunter_msgs__msg__HunterMotorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUNTER_MSGS__MSG__DETAIL__HUNTER_MOTOR_STATE__STRUCT_H_
