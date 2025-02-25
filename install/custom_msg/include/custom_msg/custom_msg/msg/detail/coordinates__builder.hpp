// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msg:msg/Coordinates.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__COORDINATES__BUILDER_HPP_
#define CUSTOM_MSG__MSG__DETAIL__COORDINATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msg/msg/detail/coordinates__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msg
{

namespace msg
{

namespace builder
{

class Init_Coordinates_toggle
{
public:
  explicit Init_Coordinates_toggle(::custom_msg::msg::Coordinates & msg)
  : msg_(msg)
  {}
  ::custom_msg::msg::Coordinates toggle(::custom_msg::msg::Coordinates::_toggle_type arg)
  {
    msg_.toggle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msg::msg::Coordinates msg_;
};

class Init_Coordinates_y
{
public:
  explicit Init_Coordinates_y(::custom_msg::msg::Coordinates & msg)
  : msg_(msg)
  {}
  Init_Coordinates_toggle y(::custom_msg::msg::Coordinates::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Coordinates_toggle(msg_);
  }

private:
  ::custom_msg::msg::Coordinates msg_;
};

class Init_Coordinates_x
{
public:
  Init_Coordinates_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Coordinates_y x(::custom_msg::msg::Coordinates::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Coordinates_y(msg_);
  }

private:
  ::custom_msg::msg::Coordinates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msg::msg::Coordinates>()
{
  return custom_msg::msg::builder::Init_Coordinates_x();
}

}  // namespace custom_msg

#endif  // CUSTOM_MSG__MSG__DETAIL__COORDINATES__BUILDER_HPP_
