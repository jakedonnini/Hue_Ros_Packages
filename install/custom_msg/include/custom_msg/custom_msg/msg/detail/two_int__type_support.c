// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_msg:msg/TwoInt.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_msg/msg/detail/two_int__rosidl_typesupport_introspection_c.h"
#include "custom_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_msg/msg/detail/two_int__functions.h"
#include "custom_msg/msg/detail/two_int__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void custom_msg__msg__TwoInt__rosidl_typesupport_introspection_c__TwoInt_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_msg__msg__TwoInt__init(message_memory);
}

void custom_msg__msg__TwoInt__rosidl_typesupport_introspection_c__TwoInt_fini_function(void * message_memory)
{
  custom_msg__msg__TwoInt__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_msg__msg__TwoInt__rosidl_typesupport_introspection_c__TwoInt_message_member_array[3] = {
  {
    "r",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msg__msg__TwoInt, r),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "l",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msg__msg__TwoInt, l),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "toggle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msg__msg__TwoInt, toggle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_msg__msg__TwoInt__rosidl_typesupport_introspection_c__TwoInt_message_members = {
  "custom_msg__msg",  // message namespace
  "TwoInt",  // message name
  3,  // number of fields
  sizeof(custom_msg__msg__TwoInt),
  custom_msg__msg__TwoInt__rosidl_typesupport_introspection_c__TwoInt_message_member_array,  // message members
  custom_msg__msg__TwoInt__rosidl_typesupport_introspection_c__TwoInt_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_msg__msg__TwoInt__rosidl_typesupport_introspection_c__TwoInt_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_msg__msg__TwoInt__rosidl_typesupport_introspection_c__TwoInt_message_type_support_handle = {
  0,
  &custom_msg__msg__TwoInt__rosidl_typesupport_introspection_c__TwoInt_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_msg, msg, TwoInt)() {
  if (!custom_msg__msg__TwoInt__rosidl_typesupport_introspection_c__TwoInt_message_type_support_handle.typesupport_identifier) {
    custom_msg__msg__TwoInt__rosidl_typesupport_introspection_c__TwoInt_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_msg__msg__TwoInt__rosidl_typesupport_introspection_c__TwoInt_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
