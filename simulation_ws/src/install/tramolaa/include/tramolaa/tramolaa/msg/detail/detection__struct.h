// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tramolaa:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef TRAMOLAA__MSG__DETAIL__DETECTION__STRUCT_H_
#define TRAMOLAA__MSG__DETAIL__DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Detection in the package tramolaa.
typedef struct tramolaa__msg__Detection
{
  /// Normalized x-center of the bounding box (0 to 1)
  float x_center;
  /// Normalized y-center of the bounding box (0 to 1)
  float y_center;
  /// Normalized width of the bounding box (0 to 1)
  float width;
  /// Normalized height of the bounding box (0 to 1)
  float height;
  /// Confidence score (0 to 1)
  float confidence;
  /// Class ID of the detected object
  int32_t class_id;
} tramolaa__msg__Detection;

// Struct for a sequence of tramolaa__msg__Detection.
typedef struct tramolaa__msg__Detection__Sequence
{
  tramolaa__msg__Detection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tramolaa__msg__Detection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRAMOLAA__MSG__DETAIL__DETECTION__STRUCT_H_
