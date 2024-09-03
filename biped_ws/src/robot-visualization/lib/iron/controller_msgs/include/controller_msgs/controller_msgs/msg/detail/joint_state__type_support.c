// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from controller_msgs:msg/JointState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "controller_msgs/msg/detail/joint_state__rosidl_typesupport_introspection_c.h"
#include "controller_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "controller_msgs/msg/detail/joint_state__functions.h"
#include "controller_msgs/msg/detail/joint_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `q`
// Member `v`
// Member `vd`
// Member `tau`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__JointState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller_msgs__msg__JointState__init(message_memory);
}

void controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__JointState_fini_function(void * message_memory)
{
  controller_msgs__msg__JointState__fini(message_memory);
}

size_t controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__size_function__JointState__q(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_const_function__JointState__q(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_function__JointState__q(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__fetch_function__JointState__q(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_const_function__JointState__q(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__assign_function__JointState__q(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_function__JointState__q(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__resize_function__JointState__q(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__size_function__JointState__v(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_const_function__JointState__v(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_function__JointState__v(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__fetch_function__JointState__v(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_const_function__JointState__v(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__assign_function__JointState__v(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_function__JointState__v(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__resize_function__JointState__v(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__size_function__JointState__vd(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_const_function__JointState__vd(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_function__JointState__vd(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__fetch_function__JointState__vd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_const_function__JointState__vd(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__assign_function__JointState__vd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_function__JointState__vd(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__resize_function__JointState__vd(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__size_function__JointState__tau(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_const_function__JointState__tau(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_function__JointState__tau(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__fetch_function__JointState__tau(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_const_function__JointState__tau(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__assign_function__JointState__tau(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_function__JointState__tau(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__resize_function__JointState__tau(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__JointState_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__JointState, header),  // bytes offset in struct
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
    offsetof(controller_msgs__msg__JointState, q),  // bytes offset in struct
    NULL,  // default value
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__size_function__JointState__q,  // size() function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_const_function__JointState__q,  // get_const(index) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_function__JointState__q,  // get(index) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__fetch_function__JointState__q,  // fetch(index, &value) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__assign_function__JointState__q,  // assign(index, value) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__resize_function__JointState__q  // resize(index) function pointer
  },
  {
    "v",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__JointState, v),  // bytes offset in struct
    NULL,  // default value
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__size_function__JointState__v,  // size() function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_const_function__JointState__v,  // get_const(index) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_function__JointState__v,  // get(index) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__fetch_function__JointState__v,  // fetch(index, &value) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__assign_function__JointState__v,  // assign(index, value) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__resize_function__JointState__v  // resize(index) function pointer
  },
  {
    "vd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__JointState, vd),  // bytes offset in struct
    NULL,  // default value
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__size_function__JointState__vd,  // size() function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_const_function__JointState__vd,  // get_const(index) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_function__JointState__vd,  // get(index) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__fetch_function__JointState__vd,  // fetch(index, &value) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__assign_function__JointState__vd,  // assign(index, value) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__resize_function__JointState__vd  // resize(index) function pointer
  },
  {
    "tau",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__JointState, tau),  // bytes offset in struct
    NULL,  // default value
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__size_function__JointState__tau,  // size() function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_const_function__JointState__tau,  // get_const(index) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__get_function__JointState__tau,  // get(index) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__fetch_function__JointState__tau,  // fetch(index, &value) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__assign_function__JointState__tau,  // assign(index, value) function pointer
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__resize_function__JointState__tau  // resize(index) function pointer
  },
  {
    "na",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller_msgs__msg__JointState, na),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__JointState_message_members = {
  "controller_msgs__msg",  // message namespace
  "JointState",  // message name
  6,  // number of fields
  sizeof(controller_msgs__msg__JointState),
  controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__JointState_message_member_array,  // message members
  controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__JointState_init_function,  // function to initialize message memory (memory has to be allocated)
  controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__JointState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__JointState_message_type_support_handle = {
  0,
  &controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__JointState_message_members,
  get_message_typesupport_handle_function,
  &controller_msgs__msg__JointState__get_type_hash,
  &controller_msgs__msg__JointState__get_type_description,
  &controller_msgs__msg__JointState__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller_msgs, msg, JointState)() {
  controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__JointState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__JointState_message_type_support_handle.typesupport_identifier) {
    controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__JointState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller_msgs__msg__JointState__rosidl_typesupport_introspection_c__JointState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
