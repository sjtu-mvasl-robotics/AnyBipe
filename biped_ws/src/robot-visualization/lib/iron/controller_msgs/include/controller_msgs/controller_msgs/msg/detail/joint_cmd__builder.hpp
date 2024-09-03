// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller_msgs:msg/JointCmd.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER_MSGS__MSG__DETAIL__JOINT_CMD__BUILDER_HPP_
#define CONTROLLER_MSGS__MSG__DETAIL__JOINT_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller_msgs/msg/detail/joint_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller_msgs
{

namespace msg
{

namespace builder
{

class Init_JointCmd_na
{
public:
  explicit Init_JointCmd_na(::controller_msgs::msg::JointCmd & msg)
  : msg_(msg)
  {}
  ::controller_msgs::msg::JointCmd na(::controller_msgs::msg::JointCmd::_na_type arg)
  {
    msg_.na = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller_msgs::msg::JointCmd msg_;
};

class Init_JointCmd_mode
{
public:
  explicit Init_JointCmd_mode(::controller_msgs::msg::JointCmd & msg)
  : msg_(msg)
  {}
  Init_JointCmd_na mode(::controller_msgs::msg::JointCmd::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_JointCmd_na(msg_);
  }

private:
  ::controller_msgs::msg::JointCmd msg_;
};

class Init_JointCmd_kd
{
public:
  explicit Init_JointCmd_kd(::controller_msgs::msg::JointCmd & msg)
  : msg_(msg)
  {}
  Init_JointCmd_mode kd(::controller_msgs::msg::JointCmd::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_JointCmd_mode(msg_);
  }

private:
  ::controller_msgs::msg::JointCmd msg_;
};

class Init_JointCmd_kp
{
public:
  explicit Init_JointCmd_kp(::controller_msgs::msg::JointCmd & msg)
  : msg_(msg)
  {}
  Init_JointCmd_kd kp(::controller_msgs::msg::JointCmd::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_JointCmd_kd(msg_);
  }

private:
  ::controller_msgs::msg::JointCmd msg_;
};

class Init_JointCmd_tau
{
public:
  explicit Init_JointCmd_tau(::controller_msgs::msg::JointCmd & msg)
  : msg_(msg)
  {}
  Init_JointCmd_kp tau(::controller_msgs::msg::JointCmd::_tau_type arg)
  {
    msg_.tau = std::move(arg);
    return Init_JointCmd_kp(msg_);
  }

private:
  ::controller_msgs::msg::JointCmd msg_;
};

class Init_JointCmd_v
{
public:
  explicit Init_JointCmd_v(::controller_msgs::msg::JointCmd & msg)
  : msg_(msg)
  {}
  Init_JointCmd_tau v(::controller_msgs::msg::JointCmd::_v_type arg)
  {
    msg_.v = std::move(arg);
    return Init_JointCmd_tau(msg_);
  }

private:
  ::controller_msgs::msg::JointCmd msg_;
};

class Init_JointCmd_q
{
public:
  explicit Init_JointCmd_q(::controller_msgs::msg::JointCmd & msg)
  : msg_(msg)
  {}
  Init_JointCmd_v q(::controller_msgs::msg::JointCmd::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_JointCmd_v(msg_);
  }

private:
  ::controller_msgs::msg::JointCmd msg_;
};

class Init_JointCmd_header
{
public:
  Init_JointCmd_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointCmd_q header(::controller_msgs::msg::JointCmd::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_JointCmd_q(msg_);
  }

private:
  ::controller_msgs::msg::JointCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller_msgs::msg::JointCmd>()
{
  return controller_msgs::msg::builder::Init_JointCmd_header();
}

}  // namespace controller_msgs

#endif  // CONTROLLER_MSGS__MSG__DETAIL__JOINT_CMD__BUILDER_HPP_
