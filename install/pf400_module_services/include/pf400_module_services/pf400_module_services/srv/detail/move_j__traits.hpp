// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pf400_module_services:srv/MoveJ.idl
// generated code does not contain a copyright notice

#ifndef PF400_MODULE_SERVICES__SRV__DETAIL__MOVE_J__TRAITS_HPP_
#define PF400_MODULE_SERVICES__SRV__DETAIL__MOVE_J__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pf400_module_services/srv/detail/move_j__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pf400_module_services
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveJ_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: joint_positions
  {
    if (msg.joint_positions.size() == 0) {
      out << "joint_positions: []";
    } else {
      out << "joint_positions: [";
      size_t pending_items = msg.joint_positions.size();
      for (auto item : msg.joint_positions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveJ_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joint_positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_positions.size() == 0) {
      out << "joint_positions: []\n";
    } else {
      out << "joint_positions:\n";
      for (auto item : msg.joint_positions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveJ_Request & msg, bool use_flow_style = false)
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
  const pf400_module_services::srv::MoveJ_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pf400_module_services::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pf400_module_services::srv::to_yaml() instead")]]
inline std::string to_yaml(const pf400_module_services::srv::MoveJ_Request & msg)
{
  return pf400_module_services::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pf400_module_services::srv::MoveJ_Request>()
{
  return "pf400_module_services::srv::MoveJ_Request";
}

template<>
inline const char * name<pf400_module_services::srv::MoveJ_Request>()
{
  return "pf400_module_services/srv/MoveJ_Request";
}

template<>
struct has_fixed_size<pf400_module_services::srv::MoveJ_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pf400_module_services::srv::MoveJ_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pf400_module_services::srv::MoveJ_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace pf400_module_services
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveJ_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveJ_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveJ_Response & msg, bool use_flow_style = false)
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
  const pf400_module_services::srv::MoveJ_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pf400_module_services::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pf400_module_services::srv::to_yaml() instead")]]
inline std::string to_yaml(const pf400_module_services::srv::MoveJ_Response & msg)
{
  return pf400_module_services::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pf400_module_services::srv::MoveJ_Response>()
{
  return "pf400_module_services::srv::MoveJ_Response";
}

template<>
inline const char * name<pf400_module_services::srv::MoveJ_Response>()
{
  return "pf400_module_services/srv/MoveJ_Response";
}

template<>
struct has_fixed_size<pf400_module_services::srv::MoveJ_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pf400_module_services::srv::MoveJ_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pf400_module_services::srv::MoveJ_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pf400_module_services::srv::MoveJ>()
{
  return "pf400_module_services::srv::MoveJ";
}

template<>
inline const char * name<pf400_module_services::srv::MoveJ>()
{
  return "pf400_module_services/srv/MoveJ";
}

template<>
struct has_fixed_size<pf400_module_services::srv::MoveJ>
  : std::integral_constant<
    bool,
    has_fixed_size<pf400_module_services::srv::MoveJ_Request>::value &&
    has_fixed_size<pf400_module_services::srv::MoveJ_Response>::value
  >
{
};

template<>
struct has_bounded_size<pf400_module_services::srv::MoveJ>
  : std::integral_constant<
    bool,
    has_bounded_size<pf400_module_services::srv::MoveJ_Request>::value &&
    has_bounded_size<pf400_module_services::srv::MoveJ_Response>::value
  >
{
};

template<>
struct is_service<pf400_module_services::srv::MoveJ>
  : std::true_type
{
};

template<>
struct is_service_request<pf400_module_services::srv::MoveJ_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pf400_module_services::srv::MoveJ_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PF400_MODULE_SERVICES__SRV__DETAIL__MOVE_J__TRAITS_HPP_
