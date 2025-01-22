// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tramolaa:msg/DetectionList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tramolaa/msg/detail/detection_list__rosidl_typesupport_introspection_c.h"
#include "tramolaa/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tramolaa/msg/detail/detection_list__functions.h"
#include "tramolaa/msg/detail/detection_list__struct.h"


// Include directives for member types
// Member `objects`
#include "tramolaa/msg/detection.h"
// Member `objects`
#include "tramolaa/msg/detail/detection__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__DetectionList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tramolaa__msg__DetectionList__init(message_memory);
}

void tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__DetectionList_fini_function(void * message_memory)
{
  tramolaa__msg__DetectionList__fini(message_memory);
}

size_t tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__size_function__DetectionList__objects(
  const void * untyped_member)
{
  const tramolaa__msg__Detection__Sequence * member =
    (const tramolaa__msg__Detection__Sequence *)(untyped_member);
  return member->size;
}

const void * tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__get_const_function__DetectionList__objects(
  const void * untyped_member, size_t index)
{
  const tramolaa__msg__Detection__Sequence * member =
    (const tramolaa__msg__Detection__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__get_function__DetectionList__objects(
  void * untyped_member, size_t index)
{
  tramolaa__msg__Detection__Sequence * member =
    (tramolaa__msg__Detection__Sequence *)(untyped_member);
  return &member->data[index];
}

void tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__fetch_function__DetectionList__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const tramolaa__msg__Detection * item =
    ((const tramolaa__msg__Detection *)
    tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__get_const_function__DetectionList__objects(untyped_member, index));
  tramolaa__msg__Detection * value =
    (tramolaa__msg__Detection *)(untyped_value);
  *value = *item;
}

void tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__assign_function__DetectionList__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  tramolaa__msg__Detection * item =
    ((tramolaa__msg__Detection *)
    tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__get_function__DetectionList__objects(untyped_member, index));
  const tramolaa__msg__Detection * value =
    (const tramolaa__msg__Detection *)(untyped_value);
  *item = *value;
}

bool tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__resize_function__DetectionList__objects(
  void * untyped_member, size_t size)
{
  tramolaa__msg__Detection__Sequence * member =
    (tramolaa__msg__Detection__Sequence *)(untyped_member);
  tramolaa__msg__Detection__Sequence__fini(member);
  return tramolaa__msg__Detection__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__DetectionList_message_member_array[1] = {
  {
    "objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tramolaa__msg__DetectionList, objects),  // bytes offset in struct
    NULL,  // default value
    tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__size_function__DetectionList__objects,  // size() function pointer
    tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__get_const_function__DetectionList__objects,  // get_const(index) function pointer
    tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__get_function__DetectionList__objects,  // get(index) function pointer
    tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__fetch_function__DetectionList__objects,  // fetch(index, &value) function pointer
    tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__assign_function__DetectionList__objects,  // assign(index, value) function pointer
    tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__resize_function__DetectionList__objects  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__DetectionList_message_members = {
  "tramolaa__msg",  // message namespace
  "DetectionList",  // message name
  1,  // number of fields
  sizeof(tramolaa__msg__DetectionList),
  tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__DetectionList_message_member_array,  // message members
  tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__DetectionList_init_function,  // function to initialize message memory (memory has to be allocated)
  tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__DetectionList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__DetectionList_message_type_support_handle = {
  0,
  &tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__DetectionList_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tramolaa
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tramolaa, msg, DetectionList)() {
  tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__DetectionList_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tramolaa, msg, Detection)();
  if (!tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__DetectionList_message_type_support_handle.typesupport_identifier) {
    tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__DetectionList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tramolaa__msg__DetectionList__rosidl_typesupport_introspection_c__DetectionList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
