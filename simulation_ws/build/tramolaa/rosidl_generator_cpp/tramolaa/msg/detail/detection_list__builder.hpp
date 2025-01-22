// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tramolaa:msg/DetectionList.idl
// generated code does not contain a copyright notice

#ifndef TRAMOLAA__MSG__DETAIL__DETECTION_LIST__BUILDER_HPP_
#define TRAMOLAA__MSG__DETAIL__DETECTION_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tramolaa/msg/detail/detection_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tramolaa
{

namespace msg
{

namespace builder
{

class Init_DetectionList_detections
{
public:
  Init_DetectionList_detections()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tramolaa::msg::DetectionList detections(::tramolaa::msg::DetectionList::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tramolaa::msg::DetectionList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tramolaa::msg::DetectionList>()
{
  return tramolaa::msg::builder::Init_DetectionList_detections();
}

}  // namespace tramolaa

#endif  // TRAMOLAA__MSG__DETAIL__DETECTION_LIST__BUILDER_HPP_
