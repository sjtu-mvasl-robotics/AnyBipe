// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller_msgs:msg/IMUData.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_H_
#define CONTROLLER_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/IMUData in the package controller_msgs.
typedef struct controller_msgs__msg__IMUData
{
  std_msgs__msg__Header header;
  uint64_t imustamp;
  uint32_t status;
  double euler[3];
  double quat[4];
  double acc[3];
  double gyro[3];
} controller_msgs__msg__IMUData;

// Struct for a sequence of controller_msgs__msg__IMUData.
typedef struct controller_msgs__msg__IMUData__Sequence
{
  controller_msgs__msg__IMUData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller_msgs__msg__IMUData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_H_
