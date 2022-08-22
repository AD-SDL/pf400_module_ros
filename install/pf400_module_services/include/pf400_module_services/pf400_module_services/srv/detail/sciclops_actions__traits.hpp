// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pf400_module_services:srv/SciclopsActions.idl
// generated code does not contain a copyright notice

#ifndef PF400_MODULE_SERVICES__SRV__DETAIL__SCICLOPS_ACTIONS__TRAITS_HPP_
#define PF400_MODULE_SERVICES__SRV__DETAIL__SCICLOPS_ACTIONS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pf400_module_services/srv/detail/sciclops_actions__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pf400_module_services
{

namespace srv
{

inline void to_flow_style_yaml(
  const SciclopsActions_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: action_request
  {
    out << "action_request: ";
    rosidl_generator_traits::value_to_yaml(msg.action_request, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SciclopsActions_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: action_request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action_request: ";
    rosidl_generator_traits::value_to_yaml(msg.action_request, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SciclopsActions_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pf400_module_services

namespace rosidl_generator_traits
{

[[deprecated("use pf400_module_services::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pf400_module_services::srv::SciclopsActions_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pf400_module_services::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pf400_module_services::srv::to_yaml() instead")]]
inline std::string to_yaml(const pf400_module_services::srv::SciclopsActions_Request & msg)
{
  return pf400_module_services::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pf400_module_services::srv::SciclopsActions_Request>()
{
  return "pf400_module_services::srv::SciclopsActions_Request";
}

template<>
inline const char * name<pf400_module_services::srv::SciclopsActions_Request>()
{
  return "pf400_module_services/srv/SciclopsActions_Request";
}

template<>
struct has_fixed_size<pf400_module_services::srv::SciclopsActions_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pf400_module_services::srv::SciclopsActions_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pf400_module_services::srv::SciclopsActions_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace pf400_module_services
{

namespace srv
{

inline void to_flow_style_yaml(
  const SciclopsActions_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: action_response
  {
    out << "action_response: ";
    rosidl_generator_traits::value_to_yaml(msg.action_response, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SciclopsActions_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: action_response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action_response: ";
    rosidl_generator_traits::value_to_yaml(msg.action_response, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SciclopsActions_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pf400_module_services

namespace rosidl_generator_traits
{

[[deprecated("use pf400_module_services::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pf400_module_services::srv::SciclopsActions_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pf400_module_services::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pf400_module_services::srv::to_yaml() instead")]]
inline std::string to_yaml(const pf400_module_services::srv::SciclopsActions_Response & msg)
{
  return pf400_module_services::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pf400_module_services::srv::SciclopsActions_Response>()
{
  return "pf400_module_services::srv::SciclopsActions_Response";
}

template<>
inline const char * name<pf400_module_services::srv::SciclopsActions_Response>()
{
  return "pf400_module_services/srv/SciclopsActions_Response";
}

template<>
struct has_fixed_size<pf400_module_services::srv::SciclopsActions_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pf400_module_services::srv::SciclopsActions_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pf400_module_services::srv::SciclopsActions_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pf400_module_services::srv::SciclopsActions>()
{
  return "pf400_module_services::srv::SciclopsActions";
}

template<>
inline const char * name<pf400_module_services::srv::SciclopsActions>()
{
  return "pf400_module_services/srv/SciclopsActions";
}

template<>
struct has_fixed_size<pf400_module_services::srv::SciclopsActions>
  : std::integral_constant<
    bool,
    has_fixed_size<pf400_module_services::srv::SciclopsActions_Request>::value &&
    has_fixed_size<pf400_module_services::srv::SciclopsActions_Response>::value
  >
{
};

template<>
struct has_bounded_size<pf400_module_services::srv::SciclopsActions>
  : std::integral_constant<
    bool,
    has_bounded_size<pf400_module_services::srv::SciclopsActions_Request>::value &&
    has_bounded_size<pf400_module_services::srv::SciclopsActions_Response>::value
  >
{
};

template<>
struct is_service<pf400_module_services::srv::SciclopsActions>
  : std::true_type
{
};

template<>
struct is_service_request<pf400_module_services::srv::SciclopsActions_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pf400_module_services::srv::SciclopsActions_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PF400_MODULE_SERVICES__SRV__DETAIL__SCICLOPS_ACTIONS__TRAITS_HPP_
