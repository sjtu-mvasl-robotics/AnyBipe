// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller_msgs:msg/IMUData.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER_MSGS__MSG__DETAIL__IMU_DATA__TRAITS_HPP_
#define CONTROLLER_MSGS__MSG__DETAIL__IMU_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller_msgs/msg/detail/imu_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace controller_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const IMUData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: imustamp
  {
    out << "imustamp: ";
    rosidl_generator_traits::value_to_yaml(msg.imustamp, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: euler
  {
    if (msg.euler.size() == 0) {
      out << "euler: []";
    } else {
      out << "euler: [";
      size_t pending_items = msg.euler.size();
      for (auto item : msg.euler) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: quat
  {
    if (msg.quat.size() == 0) {
      out << "quat: []";
    } else {
      out << "quat: [";
      size_t pending_items = msg.quat.size();
      for (auto item : msg.quat) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: acc
  {
    if (msg.acc.size() == 0) {
      out << "acc: []";
    } else {
      out << "acc: [";
      size_t pending_items = msg.acc.size();
      for (auto item : msg.acc) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gyro
  {
    if (msg.gyro.size() == 0) {
      out << "gyro: []";
    } else {
      out << "gyro: [";
      size_t pending_items = msg.gyro.size();
      for (auto item : msg.gyro) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IMUData & msg,
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

  // member: imustamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imustamp: ";
    rosidl_generator_traits::value_to_yaml(msg.imustamp, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: euler
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.euler.size() == 0) {
      out << "euler: []\n";
    } else {
      out << "euler:\n";
      for (auto item : msg.euler) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: quat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.quat.size() == 0) {
      out << "quat: []\n";
    } else {
      out << "quat:\n";
      for (auto item : msg.quat) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.acc.size() == 0) {
      out << "acc: []\n";
    } else {
      out << "acc:\n";
      for (auto item : msg.acc) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gyro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gyro.size() == 0) {
      out << "gyro: []\n";
    } else {
      out << "gyro:\n";
      for (auto item : msg.gyro) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IMUData & msg, bool use_flow_style = false)
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
  const controller_msgs::msg::IMUData & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const controller_msgs::msg::IMUData & msg)
{
  return controller_msgs::msg::to_yaml(msg);
}

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
