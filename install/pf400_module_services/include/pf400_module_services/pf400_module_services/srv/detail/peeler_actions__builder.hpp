// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pf400_module_services:srv/PeelerActions.idl
// generated code does not contain a copyright notice

#ifndef PF400_MODULE_SERVICES__SRV__DETAIL__PEELER_ACTIONS__BUILDER_HPP_
#define PF400_MODULE_SERVICES__SRV__DETAIL__PEELER_ACTIONS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pf400_module_services/srv/detail/peeler_actions__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pf400_module_services
{

namespace srv
{

namespace builder
{

class Init_PeelerActions_Request_action_request
{
public:
  Init_PeelerActions_Request_action_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pf400_module_services::srv::PeelerActions_Request action_request(::pf400_module_services::srv::PeelerActions_Request::_action_request_type arg)
  {
    msg_.action_request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pf400_module_services::srv::PeelerActions_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pf400_module_services::srv::PeelerActions_Request>()
{
  return pf400_module_services::srv::builder::Init_PeelerActions_Request_action_request();
}

}  // namespace pf400_module_services


namespace pf400_module_services
{

namespace srv
{

namespace builder
{

class Init_PeelerActions_Response_action_response
{
public:
  Init_PeelerActions_Response_action_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pf400_module_services::srv::PeelerActions_Response action_response(::pf400_module_services::srv::PeelerActions_Response::_action_response_type arg)
  {
    msg_.action_response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pf400_module_services::srv::PeelerActions_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pf400_module_services::srv::PeelerActions_Response>()
{
  return pf400_module_services::srv::builder::Init_PeelerActions_Response_action_response();
}

}  // namespace pf400_module_services

#endif  // PF400_MODULE_SERVICES__SRV__DETAIL__PEELER_ACTIONS__BUILDER_HPP_
