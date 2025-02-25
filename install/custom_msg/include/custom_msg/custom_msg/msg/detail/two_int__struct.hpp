// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msg:msg/TwoInt.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__TWO_INT__STRUCT_HPP_
#define CUSTOM_MSG__MSG__DETAIL__TWO_INT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_msg__msg__TwoInt __attribute__((deprecated))
#else
# define DEPRECATED__custom_msg__msg__TwoInt __declspec(deprecated)
#endif

namespace custom_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TwoInt_
{
  using Type = TwoInt_<ContainerAllocator>;

  explicit TwoInt_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->r = 0ll;
      this->l = 0ll;
      this->toggle = 0;
    }
  }

  explicit TwoInt_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->r = 0ll;
      this->l = 0ll;
      this->toggle = 0;
    }
  }

  // field types and members
  using _r_type =
    int64_t;
  _r_type r;
  using _l_type =
    int64_t;
  _l_type l;
  using _toggle_type =
    int8_t;
  _toggle_type toggle;

  // setters for named parameter idiom
  Type & set__r(
    const int64_t & _arg)
  {
    this->r = _arg;
    return *this;
  }
  Type & set__l(
    const int64_t & _arg)
  {
    this->l = _arg;
    return *this;
  }
  Type & set__toggle(
    const int8_t & _arg)
  {
    this->toggle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msg::msg::TwoInt_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msg::msg::TwoInt_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msg::msg::TwoInt_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msg::msg::TwoInt_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msg::msg::TwoInt_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msg::msg::TwoInt_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msg::msg::TwoInt_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msg::msg::TwoInt_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msg::msg::TwoInt_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msg::msg::TwoInt_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msg__msg__TwoInt
    std::shared_ptr<custom_msg::msg::TwoInt_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msg__msg__TwoInt
    std::shared_ptr<custom_msg::msg::TwoInt_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TwoInt_ & other) const
  {
    if (this->r != other.r) {
      return false;
    }
    if (this->l != other.l) {
      return false;
    }
    if (this->toggle != other.toggle) {
      return false;
    }
    return true;
  }
  bool operator!=(const TwoInt_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TwoInt_

// alias to use template instance with default allocator
using TwoInt =
  custom_msg::msg::TwoInt_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msg

#endif  // CUSTOM_MSG__MSG__DETAIL__TWO_INT__STRUCT_HPP_
