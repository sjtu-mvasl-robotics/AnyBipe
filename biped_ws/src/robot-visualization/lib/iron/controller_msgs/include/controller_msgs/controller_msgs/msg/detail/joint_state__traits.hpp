// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller_msgs:msg/JointState.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER_MSGS__MSG__DETAIL__JOINT_STATE__TRAITS_HPP_
#define CONTROLLER_MSGS__MSG__DETAIL__JOINT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller_msgs/msg/detail/joint_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace controller_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const JointState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: q
  {
    if (msg.q.size() == 0) {
      out << "q: []";
    } else {
      out << "q: [";
      size_t pending_items = msg.q.size();
      for (auto item : msg.q) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: v
  {
    if (msg.v.size() == 0) {
      out << "v: []";
    } else {
      out << "v: [";
      size_t pending_items = msg.v.size();
      for (auto item : msg.v) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vd
  {
    if (msg.vd.size() == 0) {
      out << "vd: []";
    } else {
      out << "vd: [";
      size_t pending_items = msg.vd.size();
      for (auto item : msg.vd) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tau
  {
    if (msg.tau.size() == 0) {
      out << "tau: []";
    } else {
      out << "tau: [";
      size_t pending_items = msg.tau.size();
      for (auto item : msg.tau) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: na
  {
    out << "na: ";
    rosidl_generator_traits::value_to_yaml(msg.na, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: q
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.q.size() == 0) {
      out << "q: []\n";
    } else {
      out << "q:\n";
      for (auto item : msg.q) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: v
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.v.size() == 0) {
      out << "v: []\n";
    } else {
      out << "v:\n";
      for (auto item : msg.v) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vd.size() == 0) {
      out << "vd: []\n";
    } else {
      out << "vd:\n";
      for (auto item : msg.vd) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tau
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tau.size() == 0) {
      out << "tau: []\n";
    } else {
      out << "tau:\n";
      for (auto item : msg.tau) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: na
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "na: ";
    rosidl_generator_traits::value_to_yaml(msg.na, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace controller_msgs

namespace rosidl_generator_traits
{

[[deprecated("use controller_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const controller_msgs::msg::JointState & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const controller_msgs::msg::JointState & msg)
{
  return controller_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<controller_msgs::msg::JointState>()
{
  return "controller_msgs::msg::JointState";
}

template<>
inline const char * name<controller_msgs::msg::JointState>()
{
  return "controller_msgs/msg/JointState";
}

template<>
struct has_fixed_size<controller_msgs::msg::JointState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller_msgs::msg::JointState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller_msgs::msg::JointState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER_MSGS__MSG__DETAIL__JOINT_STATE__TRAITS_HPP_
