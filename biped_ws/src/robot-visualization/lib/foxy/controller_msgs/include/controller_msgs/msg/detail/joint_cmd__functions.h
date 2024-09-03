// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from controller_msgs:msg/JointCmd.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER_MSGS__MSG__DETAIL__JOINT_CMD__FUNCTIONS_H_
#define CONTROLLER_MSGS__MSG__DETAIL__JOINT_CMD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "controller_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "controller_msgs/msg/detail/joint_cmd__struct.h"

/// Initialize msg/JointCmd message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * controller_msgs__msg__JointCmd
 * )) before or use
 * controller_msgs__msg__JointCmd__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_controller_msgs
bool
controller_msgs__msg__JointCmd__init(controller_msgs__msg__JointCmd * msg);

/// Finalize msg/JointCmd message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller_msgs
void
controller_msgs__msg__JointCmd__fini(controller_msgs__msg__JointCmd * msg);

/// Create msg/JointCmd message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * controller_msgs__msg__JointCmd__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_controller_msgs
controller_msgs__msg__JointCmd *
controller_msgs__msg__JointCmd__create();

/// Destroy msg/JointCmd message.
/**
 * It calls
 * controller_msgs__msg__JointCmd__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller_msgs
void
controller_msgs__msg__JointCmd__destroy(controller_msgs__msg__JointCmd * msg);

/// Check for msg/JointCmd message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller_msgs
bool
controller_msgs__msg__JointCmd__are_equal(const controller_msgs__msg__JointCmd * lhs, const controller_msgs__msg__JointCmd * rhs);

/// Copy a msg/JointCmd message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller_msgs
bool
controller_msgs__msg__JointCmd__copy(
  const controller_msgs__msg__JointCmd * input,
  controller_msgs__msg__JointCmd * output);

/// Initialize array of msg/JointCmd messages.
/**
 * It allocates the memory for the number of elements and calls
 * controller_msgs__msg__JointCmd__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller_msgs
bool
controller_msgs__msg__JointCmd__Sequence__init(controller_msgs__msg__JointCmd__Sequence * array, size_t size);

/// Finalize array of msg/JointCmd messages.
/**
 * It calls
 * controller_msgs__msg__JointCmd__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller_msgs
void
controller_msgs__msg__JointCmd__Sequence__fini(controller_msgs__msg__JointCmd__Sequence * array);

/// Create array of msg/JointCmd messages.
/**
 * It allocates the memory for the array and calls
 * controller_msgs__msg__JointCmd__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_controller_msgs
controller_msgs__msg__JointCmd__Sequence *
controller_msgs__msg__JointCmd__Sequence__create(size_t size);

/// Destroy array of msg/JointCmd messages.
/**
 * It calls
 * controller_msgs__msg__JointCmd__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller_msgs
void
controller_msgs__msg__JointCmd__Sequence__destroy(controller_msgs__msg__JointCmd__Sequence * array);

/// Check for msg/JointCmd message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller_msgs
bool
controller_msgs__msg__JointCmd__Sequence__are_equal(const controller_msgs__msg__JointCmd__Sequence * lhs, const controller_msgs__msg__JointCmd__Sequence * rhs);

/// Copy an array of msg/JointCmd messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller_msgs
bool
controller_msgs__msg__JointCmd__Sequence__copy(
  const controller_msgs__msg__JointCmd__Sequence * input,
  controller_msgs__msg__JointCmd__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER_MSGS__MSG__DETAIL__JOINT_CMD__FUNCTIONS_H_
