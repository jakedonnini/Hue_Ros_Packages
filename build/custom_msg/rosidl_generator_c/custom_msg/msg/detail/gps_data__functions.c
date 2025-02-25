// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msg:msg/GpsData.idl
// generated code does not contain a copyright notice
#include "custom_msg/msg/detail/gps_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
custom_msg__msg__GpsData__init(custom_msg__msg__GpsData * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // x1
  // y1
  // x2
  // y2
  // angle
  return true;
}

void
custom_msg__msg__GpsData__fini(custom_msg__msg__GpsData * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // x1
  // y1
  // x2
  // y2
  // angle
}

bool
custom_msg__msg__GpsData__are_equal(const custom_msg__msg__GpsData * lhs, const custom_msg__msg__GpsData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // x1
  if (lhs->x1 != rhs->x1) {
    return false;
  }
  // y1
  if (lhs->y1 != rhs->y1) {
    return false;
  }
  // x2
  if (lhs->x2 != rhs->x2) {
    return false;
  }
  // y2
  if (lhs->y2 != rhs->y2) {
    return false;
  }
  // angle
  if (lhs->angle != rhs->angle) {
    return false;
  }
  return true;
}

bool
custom_msg__msg__GpsData__copy(
  const custom_msg__msg__GpsData * input,
  custom_msg__msg__GpsData * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // x1
  output->x1 = input->x1;
  // y1
  output->y1 = input->y1;
  // x2
  output->x2 = input->x2;
  // y2
  output->y2 = input->y2;
  // angle
  output->angle = input->angle;
  return true;
}

custom_msg__msg__GpsData *
custom_msg__msg__GpsData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msg__msg__GpsData * msg = (custom_msg__msg__GpsData *)allocator.allocate(sizeof(custom_msg__msg__GpsData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msg__msg__GpsData));
  bool success = custom_msg__msg__GpsData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_msg__msg__GpsData__destroy(custom_msg__msg__GpsData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_msg__msg__GpsData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_msg__msg__GpsData__Sequence__init(custom_msg__msg__GpsData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msg__msg__GpsData * data = NULL;

  if (size) {
    data = (custom_msg__msg__GpsData *)allocator.zero_allocate(size, sizeof(custom_msg__msg__GpsData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msg__msg__GpsData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msg__msg__GpsData__fini(&data[i - 1]);
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
custom_msg__msg__GpsData__Sequence__fini(custom_msg__msg__GpsData__Sequence * array)
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
      custom_msg__msg__GpsData__fini(&array->data[i]);
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

custom_msg__msg__GpsData__Sequence *
custom_msg__msg__GpsData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msg__msg__GpsData__Sequence * array = (custom_msg__msg__GpsData__Sequence *)allocator.allocate(sizeof(custom_msg__msg__GpsData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_msg__msg__GpsData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_msg__msg__GpsData__Sequence__destroy(custom_msg__msg__GpsData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_msg__msg__GpsData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_msg__msg__GpsData__Sequence__are_equal(const custom_msg__msg__GpsData__Sequence * lhs, const custom_msg__msg__GpsData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msg__msg__GpsData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msg__msg__GpsData__Sequence__copy(
  const custom_msg__msg__GpsData__Sequence * input,
  custom_msg__msg__GpsData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msg__msg__GpsData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_msg__msg__GpsData * data =
      (custom_msg__msg__GpsData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msg__msg__GpsData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_msg__msg__GpsData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_msg__msg__GpsData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
