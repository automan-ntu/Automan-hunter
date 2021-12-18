// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hunter_msgs:msg/HunterMotorState.idl
// generated code does not contain a copyright notice

#ifndef HUNTER_MSGS__MSG__DETAIL__HUNTER_MOTOR_STATE__FUNCTIONS_H_
#define HUNTER_MSGS__MSG__DETAIL__HUNTER_MOTOR_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hunter_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "hunter_msgs/msg/detail/hunter_motor_state__struct.h"

/// Initialize msg/HunterMotorState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hunter_msgs__msg__HunterMotorState
 * )) before or use
 * hunter_msgs__msg__HunterMotorState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hunter_msgs
bool
hunter_msgs__msg__HunterMotorState__init(hunter_msgs__msg__HunterMotorState * msg);

/// Finalize msg/HunterMotorState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hunter_msgs
void
hunter_msgs__msg__HunterMotorState__fini(hunter_msgs__msg__HunterMotorState * msg);

/// Create msg/HunterMotorState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hunter_msgs__msg__HunterMotorState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hunter_msgs
hunter_msgs__msg__HunterMotorState *
hunter_msgs__msg__HunterMotorState__create();

/// Destroy msg/HunterMotorState message.
/**
 * It calls
 * hunter_msgs__msg__HunterMotorState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hunter_msgs
void
hunter_msgs__msg__HunterMotorState__destroy(hunter_msgs__msg__HunterMotorState * msg);


/// Initialize array of msg/HunterMotorState messages.
/**
 * It allocates the memory for the number of elements and calls
 * hunter_msgs__msg__HunterMotorState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hunter_msgs
bool
hunter_msgs__msg__HunterMotorState__Sequence__init(hunter_msgs__msg__HunterMotorState__Sequence * array, size_t size);

/// Finalize array of msg/HunterMotorState messages.
/**
 * It calls
 * hunter_msgs__msg__HunterMotorState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hunter_msgs
void
hunter_msgs__msg__HunterMotorState__Sequence__fini(hunter_msgs__msg__HunterMotorState__Sequence * array);

/// Create array of msg/HunterMotorState messages.
/**
 * It allocates the memory for the array and calls
 * hunter_msgs__msg__HunterMotorState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hunter_msgs
hunter_msgs__msg__HunterMotorState__Sequence *
hunter_msgs__msg__HunterMotorState__Sequence__create(size_t size);

/// Destroy array of msg/HunterMotorState messages.
/**
 * It calls
 * hunter_msgs__msg__HunterMotorState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hunter_msgs
void
hunter_msgs__msg__HunterMotorState__Sequence__destroy(hunter_msgs__msg__HunterMotorState__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // HUNTER_MSGS__MSG__DETAIL__HUNTER_MOTOR_STATE__FUNCTIONS_H_
