// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from tramolaa:msg/DetectionList.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "tramolaa/msg/detail/detection_list__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace tramolaa
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DetectionList_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) tramolaa::msg::DetectionList(_init);
}

void DetectionList_fini_function(void * message_memory)
{
  auto typed_message = static_cast<tramolaa::msg::DetectionList *>(message_memory);
  typed_message->~DetectionList();
}

size_t size_function__DetectionList__objects(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<tramolaa::msg::Detection> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectionList__objects(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<tramolaa::msg::Detection> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectionList__objects(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<tramolaa::msg::Detection> *>(untyped_member);
  return &member[index];
}

void fetch_function__DetectionList__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const tramolaa::msg::Detection *>(
    get_const_function__DetectionList__objects(untyped_member, index));
  auto & value = *reinterpret_cast<tramolaa::msg::Detection *>(untyped_value);
  value = item;
}

void assign_function__DetectionList__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<tramolaa::msg::Detection *>(
    get_function__DetectionList__objects(untyped_member, index));
  const auto & value = *reinterpret_cast<const tramolaa::msg::Detection *>(untyped_value);
  item = value;
}

void resize_function__DetectionList__objects(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<tramolaa::msg::Detection> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DetectionList_message_member_array[1] = {
  {
    "objects",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<tramolaa::msg::Detection>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tramolaa::msg::DetectionList, objects),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectionList__objects,  // size() function pointer
    get_const_function__DetectionList__objects,  // get_const(index) function pointer
    get_function__DetectionList__objects,  // get(index) function pointer
    fetch_function__DetectionList__objects,  // fetch(index, &value) function pointer
    assign_function__DetectionList__objects,  // assign(index, value) function pointer
    resize_function__DetectionList__objects  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DetectionList_message_members = {
  "tramolaa::msg",  // message namespace
  "DetectionList",  // message name
  1,  // number of fields
  sizeof(tramolaa::msg::DetectionList),
  DetectionList_message_member_array,  // message members
  DetectionList_init_function,  // function to initialize message memory (memory has to be allocated)
  DetectionList_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DetectionList_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DetectionList_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace tramolaa


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tramolaa::msg::DetectionList>()
{
  return &::tramolaa::msg::rosidl_typesupport_introspection_cpp::DetectionList_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tramolaa, msg, DetectionList)() {
  return &::tramolaa::msg::rosidl_typesupport_introspection_cpp::DetectionList_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
