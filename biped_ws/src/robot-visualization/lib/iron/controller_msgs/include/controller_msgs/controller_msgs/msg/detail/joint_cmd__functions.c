// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller_msgs:msg/JointCmd.idl
// generated code does not contain a copyright notice
#include "controller_msgs/msg/detail/joint_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `q`
// Member `v`
// Member `tau`
// Member `kp`
// Member `kd`
// Member `mode`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
controller_msgs__msg__JointCmd__init(controller_msgs__msg__JointCmd * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    controller_msgs__msg__JointCmd__fini(msg);
    return false;
  }
  // q
  if (!rosidl_runtime_c__float__Sequence__init(&msg->q, 0)) {
    controller_msgs__msg__JointCmd__fini(msg);
    return false;
  }
  // v
  if (!rosidl_runtime_c__float__Sequence__init(&msg->v, 0)) {
    controller_msgs__msg__JointCmd__fini(msg);
    return false;
  }
  // tau
  if (!rosidl_runtime_c__float__Sequence__init(&msg->tau, 0)) {
    controller_msgs__msg__JointCmd__fini(msg);
    return false;
  }
  // kp
  if (!rosidl_runtime_c__float__Sequence__init(&msg->kp, 0)) {
    controller_msgs__msg__JointCmd__fini(msg);
    return false;
  }
  // kd
  if (!rosidl_runtime_c__float__Sequence__init(&msg->kd, 0)) {
    controller_msgs__msg__JointCmd__fini(msg);
    return false;
  }
  // mode
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->mode, 0)) {
    controller_msgs__msg__JointCmd__fini(msg);
    return false;
  }
  // na
  return true;
}

void
controller_msgs__msg__JointCmd__fini(controller_msgs__msg__JointCmd * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // q
  rosidl_runtime_c__float__Sequence__fini(&msg->q);
  // v
  rosidl_runtime_c__float__Sequence__fini(&msg->v);
  // tau
  rosidl_runtime_c__float__Sequence__fini(&msg->tau);
  // kp
  rosidl_runtime_c__float__Sequence__fini(&msg->kp);
  // kd
  rosidl_runtime_c__float__Sequence__fini(&msg->kd);
  // mode
  rosidl_runtime_c__uint8__Sequence__fini(&msg->mode);
  // na
}

bool
controller_msgs__msg__JointCmd__are_equal(const controller_msgs__msg__JointCmd * lhs, const controller_msgs__msg__JointCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // q
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->q), &(rhs->q)))
  {
    return false;
  }
  // v
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->v), &(rhs->v)))
  {
    return false;
  }
  // tau
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->tau), &(rhs->tau)))
  {
    return false;
  }
  // kp
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->kp), &(rhs->kp)))
  {
    return false;
  }
  // kd
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->kd), &(rhs->kd)))
  {
    return false;
  }
  // mode
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->mode), &(rhs->mode)))
  {
    return false;
  }
  // na
  if (lhs->na != rhs->na) {
    return false;
  }
  return true;
}

bool
controller_msgs__msg__JointCmd__copy(
  const controller_msgs__msg__JointCmd * input,
  controller_msgs__msg__JointCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // q
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->q), &(output->q)))
  {
    return false;
  }
  // v
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->v), &(output->v)))
  {
    return false;
  }
  // tau
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->tau), &(output->tau)))
  {
    return false;
  }
  // kp
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->kp), &(output->kp)))
  {
    return false;
  }
  // kd
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->kd), &(output->kd)))
  {
    return false;
  }
  // mode
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->mode), &(output->mode)))
  {
    return false;
  }
  // na
  output->na = input->na;
  return true;
}

controller_msgs__msg__JointCmd *
controller_msgs__msg__JointCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller_msgs__msg__JointCmd * msg = (controller_msgs__msg__JointCmd *)allocator.allocate(sizeof(controller_msgs__msg__JointCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller_msgs__msg__JointCmd));
  bool success = controller_msgs__msg__JointCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller_msgs__msg__JointCmd__destroy(controller_msgs__msg__JointCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller_msgs__msg__JointCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller_msgs__msg__JointCmd__Sequence__init(controller_msgs__msg__JointCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller_msgs__msg__JointCmd * data = NULL;

  if (size) {
    data = (controller_msgs__msg__JointCmd *)allocator.zero_allocate(size, sizeof(controller_msgs__msg__JointCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller_msgs__msg__JointCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller_msgs__msg__JointCmd__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
controller_msgs__msg__JointCmd__Sequence__fini(controller_msgs__msg__JointCmd__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      controller_msgs__msg__JointCmd__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

controller_msgs__msg__JointCmd__Sequence *
controller_msgs__msg__JointCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller_msgs__msg__JointCmd__Sequence * array = (controller_msgs__msg__JointCmd__Sequence *)allocator.allocate(sizeof(controller_msgs__msg__JointCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller_msgs__msg__JointCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller_msgs__msg__JointCmd__Sequence__destroy(controller_msgs__msg__JointCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller_msgs__msg__JointCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller_msgs__msg__JointCmd__Sequence__are_equal(const controller_msgs__msg__JointCmd__Sequence * lhs, const controller_msgs__msg__JointCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller_msgs__msg__JointCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller_msgs__msg__JointCmd__Sequence__copy(
  const controller_msgs__msg__JointCmd__Sequence * input,
  controller_msgs__msg__JointCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller_msgs__msg__JointCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller_msgs__msg__JointCmd * data =
      (controller_msgs__msg__JointCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller_msgs__msg__JointCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller_msgs__msg__JointCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller_msgs__msg__JointCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
