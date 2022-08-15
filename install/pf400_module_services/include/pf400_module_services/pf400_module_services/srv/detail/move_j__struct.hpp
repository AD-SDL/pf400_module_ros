// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pf400_module_services:srv/MoveJ.idl
// generated code does not contain a copyright notice

#ifndef PF400_MODULE_SERVICES__SRV__DETAIL__MOVE_J__STRUCT_HPP_
#define PF400_MODULE_SERVICES__SRV__DETAIL__MOVE_J__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pf400_module_services__srv__MoveJ_Request __attribute__((deprecated))
#else
# define DEPRECATED__pf400_module_services__srv__MoveJ_Request __declspec(deprecated)
#endif

namespace pf400_module_services
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveJ_Request_
{
  using Type = MoveJ_Request_<ContainerAllocator>;

  explicit MoveJ_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MoveJ_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _joint_positions_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _joint_positions_type joint_positions;

  // setters for named parameter idiom
  Type & set__joint_positions(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->joint_positions = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pf400_module_services::srv::MoveJ_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pf400_module_services::srv::MoveJ_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pf400_module_services::srv::MoveJ_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pf400_module_services::srv::MoveJ_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pf400_module_services::srv::MoveJ_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pf400_module_services::srv::MoveJ_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pf400_module_services::srv::MoveJ_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pf400_module_services::srv::MoveJ_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pf400_module_services::srv::MoveJ_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pf400_module_services::srv::MoveJ_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pf400_module_services__srv__MoveJ_Request
    std::shared_ptr<pf400_module_services::srv::MoveJ_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pf400_module_services__srv__MoveJ_Request
    std::shared_ptr<pf400_module_services::srv::MoveJ_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveJ_Request_ & other) const
  {
    if (this->joint_positions != other.joint_positions) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveJ_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveJ_Request_

// alias to use template instance with default allocator
using MoveJ_Request =
  pf400_module_services::srv::MoveJ_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pf400_module_services


#ifndef _WIN32
# define DEPRECATED__pf400_module_services__srv__MoveJ_Response __attribute__((deprecated))
#else
# define DEPRECATED__pf400_module_services__srv__MoveJ_Response __declspec(deprecated)
#endif

namespace pf400_module_services
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveJ_Response_
{
  using Type = MoveJ_Response_<ContainerAllocator>;

  explicit MoveJ_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit MoveJ_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    pf400_module_services::srv::MoveJ_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pf400_module_services::srv::MoveJ_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pf400_module_services::srv::MoveJ_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pf400_module_services::srv::MoveJ_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pf400_module_services::srv::MoveJ_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pf400_module_services::srv::MoveJ_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pf400_module_services::srv::MoveJ_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pf400_module_services::srv::MoveJ_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pf400_module_services::srv::MoveJ_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pf400_module_services::srv::MoveJ_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pf400_module_services__srv__MoveJ_Response
    std::shared_ptr<pf400_module_services::srv::MoveJ_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pf400_module_services__srv__MoveJ_Response
    std::shared_ptr<pf400_module_services::srv::MoveJ_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveJ_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveJ_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveJ_Response_

// alias to use template instance with default allocator
using MoveJ_Response =
  pf400_module_services::srv::MoveJ_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pf400_module_services

namespace pf400_module_services
{

namespace srv
{

struct MoveJ
{
  using Request = pf400_module_services::srv::MoveJ_Request;
  using Response = pf400_module_services::srv::MoveJ_Response;
};

}  // namespace srv

}  // namespace pf400_module_services

#endif  // PF400_MODULE_SERVICES__SRV__DETAIL__MOVE_J__STRUCT_HPP_
