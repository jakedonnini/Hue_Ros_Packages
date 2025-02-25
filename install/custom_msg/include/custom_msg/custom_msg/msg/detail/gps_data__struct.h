// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msg:msg/GpsData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__GPS_DATA__STRUCT_H_
#define CUSTOM_MSG__MSG__DETAIL__GPS_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/GpsData in the package custom_msg.
typedef struct custom_msg__msg__GpsData
{
  double x;
  double y;
  double x1;
  double y1;
  double x2;
  double y2;
  double angle;
} custom_msg__msg__GpsData;

// Struct for a sequence of custom_msg__msg__GpsData.
typedef struct custom_msg__msg__GpsData__Sequence
{
  custom_msg__msg__GpsData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msg__msg__GpsData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSG__MSG__DETAIL__GPS_DATA__STRUCT_H_
