// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller_msgs:msg/JointState.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER_MSGS__MSG__DETAIL__JOINT_STATE__BUILDER_HPP_
#define CONTROLLER_MSGS__MSG__DETAIL__JOINT_STATE__BUILDER_HPP_

#include "controller_msgs/msg/detail/joint_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace controller_msgs
{

namespace msg
{

namespace builder
{

class Init_JointState_na
{
public:
  explicit Init_JointState_na(::controller_msgs::msg::JointState & msg)
  : msg_(msg)
  {}
  ::controller_msgs::msg::JointState na(::controller_msgs::msg::JointState::_na_type arg)
  {
    msg_.na = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller_msgs::msg::JointState msg_;
};

class Init_JointState_tau
{
public:
  explicit Init_JointState_tau(::controller_msgs::msg::JointState & msg)
  : msg_(msg)
  {}
  Init_JointState_na tau(::controller_msgs::msg::JointState::_tau_type arg)
  {
    msg_.tau = std::move(arg);
    return Init_JointState_na(msg_);
  }

private:
  ::controller_msgs::msg::JointState msg_;
};

class Init_JointState_vd
{
public:
  explicit Init_JointState_vd(::controller_msgs::msg::JointState & msg)
  : msg_(msg)
  {}
  Init_JointState_tau vd(::controller_msgs::msg::JointState::_vd_type arg)
  {
    msg_.vd = std::move(arg);
    return Init_JointState_tau(msg_);
  }

private:
  ::controller_msgs::msg::JointState msg_;
};

class Init_JointState_v
{
public:
  explicit Init_JointState_v(::controller_msgs::msg::JointState & msg)
  : msg_(msg)
  {}
  Init_JointState_vd v(::controller_msgs::msg::JointState::_v_type arg)
  {
    msg_.v = std::move(arg);
    return Init_JointState_vd(msg_);
  }

private:
  ::controller_msgs::msg::JointState msg_;
};

class Init_JointState_q
{
public:
  explicit Init_JointState_q(::controller_msgs::msg::JointState & msg)
  : msg_(msg)
  {}
  Init_JointState_v q(::controller_msgs::msg::JointState::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_JointState_v(msg_);
  }

private:
  ::controller_msgs::msg::JointState msg_;
};

class Init_JointState_header
{
public:
  Init_JointState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointState_q header(::controller_msgs::msg::JointState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_JointState_q(msg_);
  }

private:
  ::controller_msgs::msg::JointState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller_msgs::msg::JointState>()
{
  return controller_msgs::msg::builder::Init_JointState_header();
}

}  // namespace controller_msgs

#endif  // CONTROLLER_MSGS__MSG__DETAIL__JOINT_STATE__BUILDER_HPP_
