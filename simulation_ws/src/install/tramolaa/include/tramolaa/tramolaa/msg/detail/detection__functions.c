// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tramolaa:msg/Detection.idl
// generated code does not contain a copyright notice
#include "tramolaa/msg/detail/detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
tramolaa__msg__Detection__init(tramolaa__msg__Detection * msg)
{
  if (!msg) {
    return false;
  }
  // x_center
  // y_center
  // width
  // height
  // confidence
  // class_id
  return true;
}

void
tramolaa__msg__Detection__fini(tramolaa__msg__Detection * msg)
{
  if (!msg) {
    return;
  }
  // x_center
  // y_center
  // width
  // height
  // confidence
  // class_id
}

bool
tramolaa__msg__Detection__are_equal(const tramolaa__msg__Detection * lhs, const tramolaa__msg__Detection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x_center
  if (lhs->x_center != rhs->x_center) {
    return false;
  }
  // y_center
  if (lhs->y_center != rhs->y_center) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // class_id
  if (lhs->class_id != rhs->class_id) {
    return false;
  }
  return true;
}

bool
tramolaa__msg__Detection__copy(
  const tramolaa__msg__Detection * input,
  tramolaa__msg__Detection * output)
{
  if (!input || !output) {
    return false;
  }
  // x_center
  output->x_center = input->x_center;
  // y_center
  output->y_center = input->y_center;
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // confidence
  output->confidence = input->confidence;
  // class_id
  output->class_id = input->class_id;
  return true;
}

tramolaa__msg__Detection *
tramolaa__msg__Detection__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tramolaa__msg__Detection * msg = (tramolaa__msg__Detection *)allocator.allocate(sizeof(tramolaa__msg__Detection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tramolaa__msg__Detection));
  bool success = tramolaa__msg__Detection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tramolaa__msg__Detection__destroy(tramolaa__msg__Detection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tramolaa__msg__Detection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tramolaa__msg__Detection__Sequence__init(tramolaa__msg__Detection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tramolaa__msg__Detection * data = NULL;

  if (size) {
    data = (tramolaa__msg__Detection *)allocator.zero_allocate(size, sizeof(tramolaa__msg__Detection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tramolaa__msg__Detection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tramolaa__msg__Detection__fini(&data[i - 1]);
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
tramolaa__msg__Detection__Sequence__fini(tramolaa__msg__Detection__Sequence * array)
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
      tramolaa__msg__Detection__fini(&array->data[i]);
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

tramolaa__msg__Detection__Sequence *
tramolaa__msg__Detection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tramolaa__msg__Detection__Sequence * array = (tramolaa__msg__Detection__Sequence *)allocator.allocate(sizeof(tramolaa__msg__Detection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tramolaa__msg__Detection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tramolaa__msg__Detection__Sequence__destroy(tramolaa__msg__Detection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tramolaa__msg__Detection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tramolaa__msg__Detection__Sequence__are_equal(const tramolaa__msg__Detection__Sequence * lhs, const tramolaa__msg__Detection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tramolaa__msg__Detection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tramolaa__msg__Detection__Sequence__copy(
  const tramolaa__msg__Detection__Sequence * input,
  tramolaa__msg__Detection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tramolaa__msg__Detection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tramolaa__msg__Detection * data =
      (tramolaa__msg__Detection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tramolaa__msg__Detection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tramolaa__msg__Detection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tramolaa__msg__Detection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
