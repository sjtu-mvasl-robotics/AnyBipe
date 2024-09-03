// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller_msgs:msg/IMUData.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER_MSGS__MSG__DETAIL__IMU_DATA__BUILDER_HPP_
#define CONTROLLER_MSGS__MSG__DETAIL__IMU_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller_msgs/msg/detail/imu_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller_msgs
{

namespace msg
{

namespace builder
{

class Init_IMUData_gyro
{
public:
  explicit Init_IMUData_gyro(::controller_msgs::msg::IMUData & msg)
  : msg_(msg)
  {}
  ::controller_msgs::msg::IMUData gyro(::controller_msgs::msg::IMUData::_gyro_type arg)
  {
    msg_.gyro = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller_msgs::msg::IMUData msg_;
};

class Init_IMUData_acc
{
public:
  explicit Init_IMUData_acc(::controller_msgs::msg::IMUData & msg)
  : msg_(msg)
  {}
  Init_IMUData_gyro acc(::controller_msgs::msg::IMUData::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return Init_IMUData_gyro(msg_);
  }

private:
  ::controller_msgs::msg::IMUData msg_;
};

class Init_IMUData_quat
{
public:
  explicit Init_IMUData_quat(::controller_msgs::msg::IMUData & msg)
  : msg_(msg)
  {}
  Init_IMUData_acc quat(::controller_msgs::msg::IMUData::_quat_type arg)
  {
    msg_.quat = std::move(arg);
    return Init_IMUData_acc(msg_);
  }

private:
  ::controller_msgs::msg::IMUData msg_;
};

class Init_IMUData_euler
{
public:
  explicit Init_IMUData_euler(::controller_msgs::msg::IMUData & msg)
  : msg_(msg)
  {}
  Init_IMUData_quat euler(::controller_msgs::msg::IMUData::_euler_type arg)
  {
    msg_.euler = std::move(arg);
    return Init_IMUData_quat(msg_);
  }

private:
  ::controller_msgs::msg::IMUData msg_;
};

class Init_IMUData_status
{
public:
  explicit Init_IMUData_status(::controller_msgs::msg::IMUData & msg)
  : msg_(msg)
  {}
  Init_IMUData_euler status(::controller_msgs::msg::IMUData::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_IMUData_euler(msg_);
  }

private:
  ::controller_msgs::msg::IMUData msg_;
};

class Init_IMUData_imustamp
{
public:
  explicit Init_IMUData_imustamp(::controller_msgs::msg::IMUData & msg)
  : msg_(msg)
  {}
  Init_IMUData_status imustamp(::controller_msgs::msg::IMUData::_imustamp_type arg)
  {
    msg_.imustamp = std::move(arg);
    return Init_IMUData_status(msg_);
  }

private:
  ::controller_msgs::msg::IMUData msg_;
};

class Init_IMUData_header
{
public:
  Init_IMUData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IMUData_imustamp header(::controller_msgs::msg::IMUData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_IMUData_imustamp(msg_);
  }

private:
  ::controller_msgs::msg::IMUData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller_msgs::msg::IMUData>()
{
  return controller_msgs::msg::builder::Init_IMUData_header();
}

}  // namespace controller_msgs

#endif  // CONTROLLER_MSGS__MSG__DETAIL__IMU_DATA__BUILDER_HPP_
