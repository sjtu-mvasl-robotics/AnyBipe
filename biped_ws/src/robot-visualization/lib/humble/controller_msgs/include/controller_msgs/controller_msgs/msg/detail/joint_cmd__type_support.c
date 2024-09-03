// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from controller_msgs:msg/JointCmd.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "controller_msgs/msg/detail/joint_cmd__rosidl_typesupport_introspection_c.h"
#include "controller_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "controller_msgs/msg/detail/joint_cmd__functions.h"
#include "controller_msgs/msg/detail/joint_cmd__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `q`
// Member `v`
// Member `tau`
// Member `kp`
// Member `kd`
// Member `mode`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__JointCmd_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller_msgs__msg__JointCmd__init(message_memory);
}

void controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__JointCmd_fini_function(void * message_memory)
{
  controller_msgs__msg__JointCmd__fini(message_memory);
}

size_t controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__size_function__JointCmd__q(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__q(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__q(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__fetch_function__JointCmd__q(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__q(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__assign_function__JointCmd__q(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__q(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__resize_function__JointCmd__q(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__size_function__JointCmd__v(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__v(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__v(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__fetch_function__JointCmd__v(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__v(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__assign_function__JointCmd__v(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__v(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__resize_function__JointCmd__v(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__size_function__JointCmd__tau(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__tau(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__tau(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__fetch_function__JointCmd__tau(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__tau(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__assign_function__JointCmd__tau(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__tau(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__resize_function__JointCmd__tau(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__size_function__JointCmd__kp(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__kp(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__kp(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__fetch_function__JointCmd__kp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__kp(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__assign_function__JointCmd__kp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__kp(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__resize_function__JointCmd__kp(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__size_function__JointCmd__kd(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__kd(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__kd(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__fetch_function__JointCmd__kd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__kd(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__assign_function__JointCmd__kd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__kd(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__resize_function__JointCmd__kd(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__size_function__JointCmd__mode(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__mode(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__mode(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__fetch_function__JointCmd__mode(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__mode(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__assign_function__JointCmd__mode(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__mode(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__resize_function__JointCmd__mode(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__JointCmd_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__JointCmd, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "q",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__JointCmd, q),  // bytes offset in struct
    NULL,  // default value
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__size_function__JointCmd__q,  // size() function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__q,  // get_const(index) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__q,  // get(index) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__fetch_function__JointCmd__q,  // fetch(index, &value) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__assign_function__JointCmd__q,  // assign(index, value) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__resize_function__JointCmd__q  // resize(index) function pointer
  },
  {
    "v",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__JointCmd, v),  // bytes offset in struct
    NULL,  // default value
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__size_function__JointCmd__v,  // size() function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__v,  // get_const(index) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__v,  // get(index) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__fetch_function__JointCmd__v,  // fetch(index, &value) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__assign_function__JointCmd__v,  // assign(index, value) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__resize_function__JointCmd__v  // resize(index) function pointer
  },
  {
    "tau",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__JointCmd, tau),  // bytes offset in struct
    NULL,  // default value
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__size_function__JointCmd__tau,  // size() function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__tau,  // get_const(index) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__tau,  // get(index) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__fetch_function__JointCmd__tau,  // fetch(index, &value) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__assign_function__JointCmd__tau,  // assign(index, value) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__resize_function__JointCmd__tau  // resize(index) function pointer
  },
  {
    "kp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__JointCmd, kp),  // bytes offset in struct
    NULL,  // default value
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__size_function__JointCmd__kp,  // size() function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__kp,  // get_const(index) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__kp,  // get(index) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__fetch_function__JointCmd__kp,  // fetch(index, &value) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__assign_function__JointCmd__kp,  // assign(index, value) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__resize_function__JointCmd__kp  // resize(index) function pointer
  },
  {
    "kd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__JointCmd, kd),  // bytes offset in struct
    NULL,  // default value
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__size_function__JointCmd__kd,  // size() function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__kd,  // get_const(index) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__kd,  // get(index) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__fetch_function__JointCmd__kd,  // fetch(index, &value) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__assign_function__JointCmd__kd,  // assign(index, value) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__resize_function__JointCmd__kd  // resize(index) function pointer
  },
  {
    "mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__JointCmd, mode),  // bytes offset in struct
    NULL,  // default value
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__size_function__JointCmd__mode,  // size() function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_const_function__JointCmd__mode,  // get_const(index) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__get_function__JointCmd__mode,  // get(index) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__fetch_function__JointCmd__mode,  // fetch(index, &value) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__assign_function__JointCmd__mode,  // assign(index, value) function pointer
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__resize_function__JointCmd__mode  // resize(index) function pointer
  },
  {
    "na",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__JointCmd, na),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__JointCmd_message_members = {
  "controller_msgs__msg",  // message namespace
  "JointCmd",  // message name
  8,  // number of fields
  sizeof(controller_msgs__msg__JointCmd),
  controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__JointCmd_message_member_array,  // message members
  controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__JointCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__JointCmd_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__JointCmd_message_type_support_handle = {
  0,
  &controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__JointCmd_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller_msgs, msg, JointCmd)() {
  controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__JointCmd_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__JointCmd_message_type_support_handle.typesupport_identifier) {
    controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__JointCmd_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller_msgs__msg__JointCmd__rosidl_typesupport_introspection_c__JointCmd_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
