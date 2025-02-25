// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msg:msg/TwoInt.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__TWO_INT__TRAITS_HPP_
#define CUSTOM_MSG__MSG__DETAIL__TWO_INT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msg/msg/detail/two_int__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const TwoInt & msg,
  std::ostream & out)
{
  out << "{";
  // member: r
  {
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << ", ";
  }

  // member: l
  {
    out << "l: ";
    rosidl_generator_traits::value_to_yaml(msg.l, out);
    out << ", ";
  }

  // member: toggle
  {
    out << "toggle: ";
    rosidl_generator_traits::value_to_yaml(msg.toggle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TwoInt & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << "\n";
  }

  // member: l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "l: ";
    rosidl_generator_traits::value_to_yaml(msg.l, out);
    out << "\n";
  }

  // member: toggle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "toggle: ";
    rosidl_generator_traits::value_to_yaml(msg.toggle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TwoInt & msg, bool use_flow_style = false)
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

}  // namespace custom_msg

namespace rosidl_generator_traits
{

[[deprecated("use custom_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msg::msg::TwoInt & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msg::msg::TwoInt & msg)
{
  return custom_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msg::msg::TwoInt>()
{
  return "custom_msg::msg::TwoInt";
}

template<>
inline const char * name<custom_msg::msg::TwoInt>()
{
  return "custom_msg/msg/TwoInt";
}

template<>
struct has_fixed_size<custom_msg::msg::TwoInt>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_msg::msg::TwoInt>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_msg::msg::TwoInt>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSG__MSG__DETAIL__TWO_INT__TRAITS_HPP_
