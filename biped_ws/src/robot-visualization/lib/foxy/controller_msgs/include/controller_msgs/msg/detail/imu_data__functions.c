// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller_msgs:msg/IMUData.idl
// generated code does not contain a copyright notice
#include "controller_msgs/msg/detail/imu_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
controller_msgs__msg__IMUData__init(controller_msgs__msg__IMUData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    controller_msgs__msg__IMUData__fini(msg);
    return false;
  }
  // imustamp
  // status
  // euler
  // quat
  // acc
  // gyro
  return true;
}

void
controller_msgs__msg__IMUData__fini(controller_msgs__msg__IMUData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // imustamp
  // status
  // euler
  // quat
  // acc
  // gyro
}

bool
controller_msgs__msg__IMUData__are_equal(const controller_msgs__msg__IMUData * lhs, const controller_msgs__msg__IMUData * rhs)
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
  // imustamp
  if (lhs->imustamp != rhs->imustamp) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // euler
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->euler[i] != rhs->euler[i]) {
      return false;
    }
  }
  // quat
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->quat[i] != rhs->quat[i]) {
      return false;
    }
  }
  // acc
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->acc[i] != rhs->acc[i]) {
      return false;
    }
  }
  // gyro
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->gyro[i] != rhs->gyro[i]) {
      return false;
    }
  }
  return true;
}

bool
controller_msgs__msg__IMUData__copy(
  const controller_msgs__msg__IMUData * input,
  controller_msgs__msg__IMUData * output)
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
  // imustamp
  output->imustamp = input->imustamp;
  // status
  output->status = input->status;
  // euler
  for (size_t i = 0; i < 3; ++i) {
    output->euler[i] = input->euler[i];
  }
  // quat
  for (size_t i = 0; i < 4; ++i) {
    output->quat[i] = input->quat[i];
  }
  // acc
  for (size_t i = 0; i < 3; ++i) {
    output->acc[i] = input->acc[i];
  }
  // gyro
  for (size_t i = 0; i < 3; ++i) {
    output->gyro[i] = input->gyro[i];
  }
  return true;
}

controller_msgs__msg__IMUData *
controller_msgs__msg__IMUData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller_msgs__msg__IMUData * msg = (controller_msgs__msg__IMUData *)allocator.allocate(sizeof(controller_msgs__msg__IMUData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller_msgs__msg__IMUData));
  bool success = controller_msgs__msg__IMUData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller_msgs__msg__IMUData__destroy(controller_msgs__msg__IMUData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller_msgs__msg__IMUData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller_msgs__msg__IMUData__Sequence__init(controller_msgs__msg__IMUData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller_msgs__msg__IMUData * data = NULL;

  if (size) {
    data = (controller_msgs__msg__IMUData *)allocator.zero_allocate(size, sizeof(controller_msgs__msg__IMUData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller_msgs__msg__IMUData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller_msgs__msg__IMUData__fini(&data[i - 1]);
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
controller_msgs__msg__IMUData__Sequence__fini(controller_msgs__msg__IMUData__Sequence * array)
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
      controller_msgs__msg__IMUData__fini(&array->data[i]);
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

controller_msgs__msg__IMUData__Sequence *
controller_msgs__msg__IMUData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller_msgs__msg__IMUData__Sequence * array = (controller_msgs__msg__IMUData__Sequence *)allocator.allocate(sizeof(controller_msgs__msg__IMUData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller_msgs__msg__IMUData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller_msgs__msg__IMUData__Sequence__destroy(controller_msgs__msg__IMUData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller_msgs__msg__IMUData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller_msgs__msg__IMUData__Sequence__are_equal(const controller_msgs__msg__IMUData__Sequence * lhs, const controller_msgs__msg__IMUData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller_msgs__msg__IMUData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller_msgs__msg__IMUData__Sequence__copy(
  const controller_msgs__msg__IMUData__Sequence * input,
  controller_msgs__msg__IMUData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller_msgs__msg__IMUData);
    controller_msgs__msg__IMUData * data =
      (controller_msgs__msg__IMUData *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller_msgs__msg__IMUData__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          controller_msgs__msg__IMUData__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller_msgs__msg__IMUData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
