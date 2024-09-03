// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from controller_msgs:msg/IMUData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "controller_msgs/msg/detail/imu_data__rosidl_typesupport_introspection_c.h"
#include "controller_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "controller_msgs/msg/detail/imu_data__functions.h"
#include "controller_msgs/msg/detail/imu_data__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__IMUData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller_msgs__msg__IMUData__init(message_memory);
}

void controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__IMUData_fini_function(void * message_memory)
{
  controller_msgs__msg__IMUData__fini(message_memory);
}

size_t controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__size_function__IMUData__euler(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_const_function__IMUData__euler(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_function__IMUData__euler(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__fetch_function__IMUData__euler(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_const_function__IMUData__euler(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__assign_function__IMUData__euler(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_function__IMUData__euler(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__size_function__IMUData__quat(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_const_function__IMUData__quat(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_function__IMUData__quat(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__fetch_function__IMUData__quat(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_const_function__IMUData__quat(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__assign_function__IMUData__quat(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_function__IMUData__quat(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__size_function__IMUData__acc(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_const_function__IMUData__acc(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_function__IMUData__acc(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__fetch_function__IMUData__acc(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_const_function__IMUData__acc(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__assign_function__IMUData__acc(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_function__IMUData__acc(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__size_function__IMUData__gyro(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_const_function__IMUData__gyro(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_function__IMUData__gyro(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__fetch_function__IMUData__gyro(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_const_function__IMUData__gyro(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__assign_function__IMUData__gyro(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_function__IMUData__gyro(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__IMUData_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__IMUData, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imustamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__IMUData, imustamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__IMUData, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "euler",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__IMUData, euler),  // bytes offset in struct
    NULL,  // default value
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__size_function__IMUData__euler,  // size() function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_const_function__IMUData__euler,  // get_const(index) function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_function__IMUData__euler,  // get(index) function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__fetch_function__IMUData__euler,  // fetch(index, &value) function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__assign_function__IMUData__euler,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "quat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__IMUData, quat),  // bytes offset in struct
    NULL,  // default value
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__size_function__IMUData__quat,  // size() function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_const_function__IMUData__quat,  // get_const(index) function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_function__IMUData__quat,  // get(index) function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__fetch_function__IMUData__quat,  // fetch(index, &value) function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__assign_function__IMUData__quat,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__IMUData, acc),  // bytes offset in struct
    NULL,  // default value
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__size_function__IMUData__acc,  // size() function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_const_function__IMUData__acc,  // get_const(index) function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_function__IMUData__acc,  // get(index) function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__fetch_function__IMUData__acc,  // fetch(index, &value) function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__assign_function__IMUData__acc,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gyro",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__IMUData, gyro),  // bytes offset in struct
    NULL,  // default value
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__size_function__IMUData__gyro,  // size() function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_const_function__IMUData__gyro,  // get_const(index) function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__get_function__IMUData__gyro,  // get(index) function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__fetch_function__IMUData__gyro,  // fetch(index, &value) function pointer
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__assign_function__IMUData__gyro,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__IMUData_message_members = {
  "controller_msgs__msg",  // message namespace
  "IMUData",  // message name
  7,  // number of fields
  sizeof(controller_msgs__msg__IMUData),
  controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__IMUData_message_member_array,  // message members
  controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__IMUData_init_function,  // function to initialize message memory (memory has to be allocated)
  controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__IMUData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__IMUData_message_type_support_handle = {
  0,
  &controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__IMUData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller_msgs, msg, IMUData)() {
  controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__IMUData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__IMUData_message_type_support_handle.typesupport_identifier) {
    controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__IMUData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller_msgs__msg__IMUData__rosidl_typesupport_introspection_c__IMUData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
