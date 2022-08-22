// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pf400_module_services:srv/SciclopsActions.idl
// generated code does not contain a copyright notice

#ifndef PF400_MODULE_SERVICES__SRV__DETAIL__SCICLOPS_ACTIONS__STRUCT_HPP_
#define PF400_MODULE_SERVICES__SRV__DETAIL__SCICLOPS_ACTIONS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pf400_module_services__srv__SciclopsActions_Request __attribute__((deprecated))
#else
# define DEPRECATED__pf400_module_services__srv__SciclopsActions_Request __declspec(deprecated)
#endif

namespace pf400_module_services
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SciclopsActions_Request_
{
  using Type = SciclopsActions_Request_<ContainerAllocator>;

  explicit SciclopsActions_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->action_request = "";
    }
  }

  explicit SciclopsActions_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : action_request(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->action_request = "";
    }
  }

  // field types and members
  using _action_request_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _action_request_type action_request;

  // setters for named parameter idiom
  Type & set__action_request(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->action_request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pf400_module_services::srv::SciclopsActions_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pf400_module_services::srv::SciclopsActions_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pf400_module_services::srv::SciclopsActions_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pf400_module_services::srv::SciclopsActions_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pf400_module_services::srv::SciclopsActions_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pf400_module_services::srv::SciclopsActions_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pf400_module_services::srv::SciclopsActions_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pf400_module_services::srv::SciclopsActions_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pf400_module_services::srv::SciclopsActions_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pf400_module_services::srv::SciclopsActions_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pf400_module_services__srv__SciclopsActions_Request
    std::shared_ptr<pf400_module_services::srv::SciclopsActions_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pf400_module_services__srv__SciclopsActions_Request
    std::shared_ptr<pf400_module_services::srv::SciclopsActions_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SciclopsActions_Request_ & other) const
  {
    if (this->action_request != other.action_request) {
      return false;
    }
    return true;
  }
  bool operator!=(const SciclopsActions_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SciclopsActions_Request_

// alias to use template instance with default allocator
using SciclopsActions_Request =
  pf400_module_services::srv::SciclopsActions_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pf400_module_services


#ifndef _WIN32
# define DEPRECATED__pf400_module_services__srv__SciclopsActions_Response __attribute__((deprecated))
#else
# define DEPRECATED__pf400_module_services__srv__SciclopsActions_Response __declspec(deprecated)
#endif

namespace pf400_module_services
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SciclopsActions_Response_
{
  using Type = SciclopsActions_Response_<ContainerAllocator>;

  explicit SciclopsActions_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->action_response = false;
    }
  }

  explicit SciclopsActions_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->action_response = false;
    }
  }

  // field types and members
  using _action_response_type =
    bool;
  _action_response_type action_response;

  // setters for named parameter idiom
  Type & set__action_response(
    const bool & _arg)
  {
    this->action_response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pf400_module_services::srv::SciclopsActions_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pf400_module_services::srv::SciclopsActions_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pf400_module_services::srv::SciclopsActions_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pf400_module_services::srv::SciclopsActions_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pf400_module_services::srv::SciclopsActions_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pf400_module_services::srv::SciclopsActions_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pf400_module_services::srv::SciclopsActions_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pf400_module_services::srv::SciclopsActions_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pf400_module_services::srv::SciclopsActions_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pf400_module_services::srv::SciclopsActions_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pf400_module_services__srv__SciclopsActions_Response
    std::shared_ptr<pf400_module_services::srv::SciclopsActions_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pf400_module_services__srv__SciclopsActions_Response
    std::shared_ptr<pf400_module_services::srv::SciclopsActions_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SciclopsActions_Response_ & other) const
  {
    if (this->action_response != other.action_response) {
      return false;
    }
    return true;
  }
  bool operator!=(const SciclopsActions_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SciclopsActions_Response_

// alias to use template instance with default allocator
using SciclopsActions_Response =
  pf400_module_services::srv::SciclopsActions_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pf400_module_services

namespace pf400_module_services
{

namespace srv
{

struct SciclopsActions
{
  using Request = pf400_module_services::srv::SciclopsActions_Request;
  using Response = pf400_module_services::srv::SciclopsActions_Response;
};

}  // namespace srv

}  // namespace pf400_module_services

#endif  // PF400_MODULE_SERVICES__SRV__DETAIL__SCICLOPS_ACTIONS__STRUCT_HPP_
