// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/FindClosestWall.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/find_closest_wall__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::FindClosestWall_Request>()
{
  return ::custom_interfaces::srv::FindClosestWall_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_FindClosestWall_Response_successful
{
public:
  Init_FindClosestWall_Response_successful()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::FindClosestWall_Response successful(::custom_interfaces::srv::FindClosestWall_Response::_successful_type arg)
  {
    msg_.successful = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::FindClosestWall_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::FindClosestWall_Response>()
{
  return custom_interfaces::srv::builder::Init_FindClosestWall_Response_successful();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_
