// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msg:msg/GpsData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__GPS_DATA__BUILDER_HPP_
#define CUSTOM_MSG__MSG__DETAIL__GPS_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msg/msg/detail/gps_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msg
{

namespace msg
{

namespace builder
{

class Init_GpsData_angle
{
public:
  explicit Init_GpsData_angle(::custom_msg::msg::GpsData & msg)
  : msg_(msg)
  {}
  ::custom_msg::msg::GpsData angle(::custom_msg::msg::GpsData::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msg::msg::GpsData msg_;
};

class Init_GpsData_y2
{
public:
  explicit Init_GpsData_y2(::custom_msg::msg::GpsData & msg)
  : msg_(msg)
  {}
  Init_GpsData_angle y2(::custom_msg::msg::GpsData::_y2_type arg)
  {
    msg_.y2 = std::move(arg);
    return Init_GpsData_angle(msg_);
  }

private:
  ::custom_msg::msg::GpsData msg_;
};

class Init_GpsData_x2
{
public:
  explicit Init_GpsData_x2(::custom_msg::msg::GpsData & msg)
  : msg_(msg)
  {}
  Init_GpsData_y2 x2(::custom_msg::msg::GpsData::_x2_type arg)
  {
    msg_.x2 = std::move(arg);
    return Init_GpsData_y2(msg_);
  }

private:
  ::custom_msg::msg::GpsData msg_;
};

class Init_GpsData_y1
{
public:
  explicit Init_GpsData_y1(::custom_msg::msg::GpsData & msg)
  : msg_(msg)
  {}
  Init_GpsData_x2 y1(::custom_msg::msg::GpsData::_y1_type arg)
  {
    msg_.y1 = std::move(arg);
    return Init_GpsData_x2(msg_);
  }

private:
  ::custom_msg::msg::GpsData msg_;
};

class Init_GpsData_x1
{
public:
  explicit Init_GpsData_x1(::custom_msg::msg::GpsData & msg)
  : msg_(msg)
  {}
  Init_GpsData_y1 x1(::custom_msg::msg::GpsData::_x1_type arg)
  {
    msg_.x1 = std::move(arg);
    return Init_GpsData_y1(msg_);
  }

private:
  ::custom_msg::msg::GpsData msg_;
};

class Init_GpsData_y
{
public:
  explicit Init_GpsData_y(::custom_msg::msg::GpsData & msg)
  : msg_(msg)
  {}
  Init_GpsData_x1 y(::custom_msg::msg::GpsData::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_GpsData_x1(msg_);
  }

private:
  ::custom_msg::msg::GpsData msg_;
};

class Init_GpsData_x
{
public:
  Init_GpsData_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GpsData_y x(::custom_msg::msg::GpsData::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_GpsData_y(msg_);
  }

private:
  ::custom_msg::msg::GpsData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msg::msg::GpsData>()
{
  return custom_msg::msg::builder::Init_GpsData_x();
}

}  // namespace custom_msg

#endif  // CUSTOM_MSG__MSG__DETAIL__GPS_DATA__BUILDER_HPP_
