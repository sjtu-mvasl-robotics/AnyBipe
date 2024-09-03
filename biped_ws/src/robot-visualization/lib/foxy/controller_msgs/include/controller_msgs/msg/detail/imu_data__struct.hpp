// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller_msgs:msg/IMUData.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_HPP_
#define CONTROLLER_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__controller_msgs__msg__IMUData __attribute__((deprecated))
#else
# define DEPRECATED__controller_msgs__msg__IMUData __declspec(deprecated)
#endif

namespace controller_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct IMUData_
{
  using Type = IMUData_<ContainerAllocator>;

  explicit IMUData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->imustamp = 0ull;
      this->status = 0ul;
      std::fill<typename std::array<double, 3>::iterator, double>(this->euler.begin(), this->euler.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->quat.begin(), this->quat.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->acc.begin(), this->acc.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->gyro.begin(), this->gyro.end(), 0.0);
    }
  }

  explicit IMUData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    euler(_alloc),
    quat(_alloc),
    acc(_alloc),
    gyro(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->imustamp = 0ull;
      this->status = 0ul;
      std::fill<typename std::array<double, 3>::iterator, double>(this->euler.begin(), this->euler.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->quat.begin(), this->quat.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->acc.begin(), this->acc.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->gyro.begin(), this->gyro.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _imustamp_type =
    uint64_t;
  _imustamp_type imustamp;
  using _status_type =
    uint32_t;
  _status_type status;
  using _euler_type =
    std::array<double, 3>;
  _euler_type euler;
  using _quat_type =
    std::array<double, 4>;
  _quat_type quat;
  using _acc_type =
    std::array<double, 3>;
  _acc_type acc;
  using _gyro_type =
    std::array<double, 3>;
  _gyro_type gyro;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__imustamp(
    const uint64_t & _arg)
  {
    this->imustamp = _arg;
    return *this;
  }
  Type & set__status(
    const uint32_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__euler(
    const std::array<double, 3> & _arg)
  {
    this->euler = _arg;
    return *this;
  }
  Type & set__quat(
    const std::array<double, 4> & _arg)
  {
    this->quat = _arg;
    return *this;
  }
  Type & set__acc(
    const std::array<double, 3> & _arg)
  {
    this->acc = _arg;
    return *this;
  }
  Type & set__gyro(
    const std::array<double, 3> & _arg)
  {
    this->gyro = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller_msgs::msg::IMUData_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller_msgs::msg::IMUData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller_msgs::msg::IMUData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller_msgs::msg::IMUData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller_msgs::msg::IMUData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller_msgs::msg::IMUData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller_msgs::msg::IMUData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller_msgs::msg::IMUData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller_msgs::msg::IMUData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller_msgs::msg::IMUData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller_msgs__msg__IMUData
    std::shared_ptr<controller_msgs::msg::IMUData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller_msgs__msg__IMUData
    std::shared_ptr<controller_msgs::msg::IMUData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IMUData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->imustamp != other.imustamp) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->euler != other.euler) {
      return false;
    }
    if (this->quat != other.quat) {
      return false;
    }
    if (this->acc != other.acc) {
      return false;
    }
    if (this->gyro != other.gyro) {
      return false;
    }
    return true;
  }
  bool operator!=(const IMUData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IMUData_

// alias to use template instance with default allocator
using IMUData =
  controller_msgs::msg::IMUData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller_msgs

#endif  // CONTROLLER_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_HPP_
