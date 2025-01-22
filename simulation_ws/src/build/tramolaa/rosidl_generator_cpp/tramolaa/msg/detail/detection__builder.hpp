// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tramolaa:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef TRAMOLAA__MSG__DETAIL__DETECTION__BUILDER_HPP_
#define TRAMOLAA__MSG__DETAIL__DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tramolaa/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tramolaa
{

namespace msg
{

namespace builder
{

class Init_Detection_class_id
{
public:
  explicit Init_Detection_class_id(::tramolaa::msg::Detection & msg)
  : msg_(msg)
  {}
  ::tramolaa::msg::Detection class_id(::tramolaa::msg::Detection::_class_id_type arg)
  {
    msg_.class_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tramolaa::msg::Detection msg_;
};

class Init_Detection_confidence
{
public:
  explicit Init_Detection_confidence(::tramolaa::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_class_id confidence(::tramolaa::msg::Detection::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Detection_class_id(msg_);
  }

private:
  ::tramolaa::msg::Detection msg_;
};

class Init_Detection_height
{
public:
  explicit Init_Detection_height(::tramolaa::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_confidence height(::tramolaa::msg::Detection::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_Detection_confidence(msg_);
  }

private:
  ::tramolaa::msg::Detection msg_;
};

class Init_Detection_width
{
public:
  explicit Init_Detection_width(::tramolaa::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_height width(::tramolaa::msg::Detection::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_Detection_height(msg_);
  }

private:
  ::tramolaa::msg::Detection msg_;
};

class Init_Detection_y_center
{
public:
  explicit Init_Detection_y_center(::tramolaa::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_width y_center(::tramolaa::msg::Detection::_y_center_type arg)
  {
    msg_.y_center = std::move(arg);
    return Init_Detection_width(msg_);
  }

private:
  ::tramolaa::msg::Detection msg_;
};

class Init_Detection_x_center
{
public:
  Init_Detection_x_center()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection_y_center x_center(::tramolaa::msg::Detection::_x_center_type arg)
  {
    msg_.x_center = std::move(arg);
    return Init_Detection_y_center(msg_);
  }

private:
  ::tramolaa::msg::Detection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tramolaa::msg::Detection>()
{
  return tramolaa::msg::builder::Init_Detection_x_center();
}

}  // namespace tramolaa

#endif  // TRAMOLAA__MSG__DETAIL__DETECTION__BUILDER_HPP_
