// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tramolaa:msg/DetectionList.idl
// generated code does not contain a copyright notice

#ifndef TRAMOLAA__MSG__DETAIL__DETECTION_LIST__STRUCT_H_
#define TRAMOLAA__MSG__DETAIL__DETECTION_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'objects'
#include "tramolaa/msg/detail/detection__struct.h"

/// Struct defined in msg/DetectionList in the package tramolaa.
typedef struct tramolaa__msg__DetectionList
{
  tramolaa__msg__Detection__Sequence objects;
} tramolaa__msg__DetectionList;

// Struct for a sequence of tramolaa__msg__DetectionList.
typedef struct tramolaa__msg__DetectionList__Sequence
{
  tramolaa__msg__DetectionList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tramolaa__msg__DetectionList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRAMOLAA__MSG__DETAIL__DETECTION_LIST__STRUCT_H_
