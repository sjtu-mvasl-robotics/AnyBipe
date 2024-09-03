// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from controller_msgs:msg/IMUData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "controller_msgs/msg/detail/imu_data__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace controller_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void IMUData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) controller_msgs::msg::IMUData(_init);
}

void IMUData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<controller_msgs::msg::IMUData *>(message_memory);
  typed_message->~IMUData();
}

size_t size_function__IMUData__euler(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__IMUData__euler(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__IMUData__euler(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__IMUData__euler(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__IMUData__euler(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__IMUData__euler(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__IMUData__euler(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__IMUData__quat(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__IMUData__quat(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__IMUData__quat(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__IMUData__quat(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__IMUData__quat(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__IMUData__quat(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__IMUData__quat(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__IMUData__acc(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__IMUData__acc(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__IMUData__acc(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__IMUData__acc(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__IMUData__acc(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__IMUData__acc(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__IMUData__acc(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__IMUData__gyro(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__IMUData__gyro(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__IMUData__gyro(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__IMUData__gyro(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__IMUData__gyro(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__IMUData__gyro(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__IMUData__gyro(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember IMUData_message_member_array[7] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs::msg::IMUData, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "imustamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs::msg::IMUData, imustamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs::msg::IMUData, status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "euler",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(controller_msgs::msg::IMUData, euler),  // bytes offset in struct
    nullptr,  // default value
    size_function__IMUData__euler,  // size() function pointer
    get_const_function__IMUData__euler,  // get_const(index) function pointer
    get_function__IMUData__euler,  // get(index) function pointer
    fetch_function__IMUData__euler,  // fetch(index, &value) function pointer
    assign_function__IMUData__euler,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "quat",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(controller_msgs::msg::IMUData, quat),  // bytes offset in struct
    nullptr,  // default value
    size_function__IMUData__quat,  // size() function pointer
    get_const_function__IMUData__quat,  // get_const(index) function pointer
    get_function__IMUData__quat,  // get(index) function pointer
    fetch_function__IMUData__quat,  // fetch(index, &value) function pointer
    assign_function__IMUData__quat,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "acc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(controller_msgs::msg::IMUData, acc),  // bytes offset in struct
    nullptr,  // default value
    size_function__IMUData__acc,  // size() function pointer
    get_const_function__IMUData__acc,  // get_const(index) function pointer
    get_function__IMUData__acc,  // get(index) function pointer
    fetch_function__IMUData__acc,  // fetch(index, &value) function pointer
    assign_function__IMUData__acc,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gyro",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(controller_msgs::msg::IMUData, gyro),  // bytes offset in struct
    nullptr,  // default value
    size_function__IMUData__gyro,  // size() function pointer
    get_const_function__IMUData__gyro,  // get_const(index) function pointer
    get_function__IMUData__gyro,  // get(index) function pointer
    fetch_function__IMUData__gyro,  // fetch(index, &value) function pointer
    assign_function__IMUData__gyro,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers IMUData_message_members = {
  "controller_msgs::msg",  // message namespace
  "IMUData",  // message name
  7,  // number of fields
  sizeof(controller_msgs::msg::IMUData),
  IMUData_message_member_array,  // message members
  IMUData_init_function,  // function to initialize message memory (memory has to be allocated)
  IMUData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t IMUData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &IMUData_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace controller_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<controller_msgs::msg::IMUData>()
{
  return &::controller_msgs::msg::rosidl_typesupport_introspection_cpp::IMUData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller_msgs, msg, IMUData)() {
  return &::controller_msgs::msg::rosidl_typesupport_introspection_cpp::IMUData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
