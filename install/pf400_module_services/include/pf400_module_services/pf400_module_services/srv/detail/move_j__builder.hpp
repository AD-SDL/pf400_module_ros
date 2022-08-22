// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pf400_module_services:srv/MoveJ.idl
// generated code does not contain a copyright notice

#ifndef PF400_MODULE_SERVICES__SRV__DETAIL__MOVE_J__BUILDER_HPP_
#define PF400_MODULE_SERVICES__SRV__DETAIL__MOVE_J__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pf400_module_services/srv/detail/move_j__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pf400_module_services
{

namespace srv
{

namespace builder
{

class Init_MoveJ_Request_joint_positions
{
public:
  Init_MoveJ_Request_joint_positions()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pf400_module_services::srv::MoveJ_Request joint_positions(::pf400_module_services::srv::MoveJ_Request::_joint_positions_type arg)
  {
    msg_.joint_positions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pf400_module_services::srv::MoveJ_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pf400_module_services::srv::MoveJ_Request>()
{
  return pf400_module_services::srv::builder::Init_MoveJ_Request_joint_positions();
}

}  // namespace pf400_module_services


namespace pf400_module_services
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pf400_module_services::srv::MoveJ_Response>()
{
  return ::pf400_module_services::srv::MoveJ_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace pf400_module_services

#endif  // PF400_MODULE_SERVICES__SRV__DETAIL__MOVE_J__BUILDER_HPP_
