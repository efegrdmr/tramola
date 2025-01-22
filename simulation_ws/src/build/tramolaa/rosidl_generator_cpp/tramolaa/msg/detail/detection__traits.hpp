// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tramolaa:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef TRAMOLAA__MSG__DETAIL__DETECTION__TRAITS_HPP_
#define TRAMOLAA__MSG__DETAIL__DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tramolaa/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tramolaa
{

namespace msg
{

inline void to_flow_style_yaml(
  const Detection & msg,
  std::ostream & out)
{
  out << "{";
  // member: x_center
  {
    out << "x_center: ";
    rosidl_generator_traits::value_to_yaml(msg.x_center, out);
    out << ", ";
  }

  // member: y_center
  {
    out << "y_center: ";
    rosidl_generator_traits::value_to_yaml(msg.y_center, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: class_id
  {
    out << "class_id: ";
    rosidl_generator_traits::value_to_yaml(msg.class_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Detection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x_center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_center: ";
    rosidl_generator_traits::value_to_yaml(msg.x_center, out);
    out << "\n";
  }

  // member: y_center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_center: ";
    rosidl_generator_traits::value_to_yaml(msg.y_center, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: class_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_id: ";
    rosidl_generator_traits::value_to_yaml(msg.class_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Detection & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace tramolaa

namespace rosidl_generator_traits
{

[[deprecated("use tramolaa::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tramolaa::msg::Detection & msg,
  std::ostream & out, size_t indentation = 0)
{
  tramolaa::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tramolaa::msg::to_yaml() instead")]]
inline std::string to_yaml(const tramolaa::msg::Detection & msg)
{
  return tramolaa::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tramolaa::msg::Detection>()
{
  return "tramolaa::msg::Detection";
}

template<>
inline const char * name<tramolaa::msg::Detection>()
{
  return "tramolaa/msg/Detection";
}

template<>
struct has_fixed_size<tramolaa::msg::Detection>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<tramolaa::msg::Detection>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<tramolaa::msg::Detection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TRAMOLAA__MSG__DETAIL__DETECTION__TRAITS_HPP_
