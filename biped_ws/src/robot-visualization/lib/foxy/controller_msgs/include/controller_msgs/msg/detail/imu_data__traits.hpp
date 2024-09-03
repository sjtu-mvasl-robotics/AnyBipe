// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller_msgs:msg/IMUData.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER_MSGS__MSG__DETAIL__IMU_DATA__TRAITS_HPP_
#define CONTROLLER_MSGS__MSG__DETAIL__IMU_DATA__TRAITS_HPP_

#include "controller_msgs/msg/detail/imu_data__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller_msgs::msg::IMUData>()
{
  return "controller_msgs::msg::IMUData";
}

template<>
inline const char * name<controller_msgs::msg::IMUData>()
{
  return "controller_msgs/msg/IMUData";
}

template<>
struct has_fixed_size<controller_msgs::msg::IMUData>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<controller_msgs::msg::IMUData>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<controller_msgs::msg::IMUData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER_MSGS__MSG__DETAIL__IMU_DATA__TRAITS_HPP_
