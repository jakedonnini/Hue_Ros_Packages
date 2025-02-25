// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msg:msg/TwoInt.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__TWO_INT__BUILDER_HPP_
#define CUSTOM_MSG__MSG__DETAIL__TWO_INT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msg/msg/detail/two_int__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msg
{

namespace msg
{

namespace builder
{

class Init_TwoInt_toggle
{
public:
  explicit Init_TwoInt_toggle(::custom_msg::msg::TwoInt & msg)
  : msg_(msg)
  {}
  ::custom_msg::msg::TwoInt toggle(::custom_msg::msg::TwoInt::_toggle_type arg)
  {
    msg_.toggle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msg::msg::TwoInt msg_;
};

class Init_TwoInt_l
{
public:
  explicit Init_TwoInt_l(::custom_msg::msg::TwoInt & msg)
  : msg_(msg)
  {}
  Init_TwoInt_toggle l(::custom_msg::msg::TwoInt::_l_type arg)
  {
    msg_.l = std::move(arg);
    return Init_TwoInt_toggle(msg_);
  }

private:
  ::custom_msg::msg::TwoInt msg_;
};

class Init_TwoInt_r
{
public:
  Init_TwoInt_r()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TwoInt_l r(::custom_msg::msg::TwoInt::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_TwoInt_l(msg_);
  }

private:
  ::custom_msg::msg::TwoInt msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msg::msg::TwoInt>()
{
  return custom_msg::msg::builder::Init_TwoInt_r();
}

}  // namespace custom_msg

#endif  // CUSTOM_MSG__MSG__DETAIL__TWO_INT__BUILDER_HPP_
