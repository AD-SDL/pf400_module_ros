// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pf400_module_services:srv/MoveJ.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pf400_module_services/srv/detail/move_j__rosidl_typesupport_introspection_c.h"
#include "pf400_module_services/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pf400_module_services/srv/detail/move_j__functions.h"
#include "pf400_module_services/srv/detail/move_j__struct.h"


// Include directives for member types
// Member `joint_positions`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__MoveJ_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pf400_module_services__srv__MoveJ_Request__init(message_memory);
}

void pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__MoveJ_Request_fini_function(void * message_memory)
{
  pf400_module_services__srv__MoveJ_Request__fini(message_memory);
}

size_t pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__size_function__MoveJ_Request__joint_positions(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__get_const_function__MoveJ_Request__joint_positions(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__get_function__MoveJ_Request__joint_positions(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__fetch_function__MoveJ_Request__joint_positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__get_const_function__MoveJ_Request__joint_positions(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__assign_function__MoveJ_Request__joint_positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__get_function__MoveJ_Request__joint_positions(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__resize_function__MoveJ_Request__joint_positions(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__MoveJ_Request_message_member_array[1] = {
  {
    "joint_positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pf400_module_services__srv__MoveJ_Request, joint_positions),  // bytes offset in struct
    NULL,  // default value
    pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__size_function__MoveJ_Request__joint_positions,  // size() function pointer
    pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__get_const_function__MoveJ_Request__joint_positions,  // get_const(index) function pointer
    pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__get_function__MoveJ_Request__joint_positions,  // get(index) function pointer
    pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__fetch_function__MoveJ_Request__joint_positions,  // fetch(index, &value) function pointer
    pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__assign_function__MoveJ_Request__joint_positions,  // assign(index, value) function pointer
    pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__resize_function__MoveJ_Request__joint_positions  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__MoveJ_Request_message_members = {
  "pf400_module_services__srv",  // message namespace
  "MoveJ_Request",  // message name
  1,  // number of fields
  sizeof(pf400_module_services__srv__MoveJ_Request),
  pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__MoveJ_Request_message_member_array,  // message members
  pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__MoveJ_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__MoveJ_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__MoveJ_Request_message_type_support_handle = {
  0,
  &pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__MoveJ_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pf400_module_services
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pf400_module_services, srv, MoveJ_Request)() {
  if (!pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__MoveJ_Request_message_type_support_handle.typesupport_identifier) {
    pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__MoveJ_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pf400_module_services__srv__MoveJ_Request__rosidl_typesupport_introspection_c__MoveJ_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "pf400_module_services/srv/detail/move_j__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pf400_module_services/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pf400_module_services/srv/detail/move_j__functions.h"
// already included above
// #include "pf400_module_services/srv/detail/move_j__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void pf400_module_services__srv__MoveJ_Response__rosidl_typesupport_introspection_c__MoveJ_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pf400_module_services__srv__MoveJ_Response__init(message_memory);
}

void pf400_module_services__srv__MoveJ_Response__rosidl_typesupport_introspection_c__MoveJ_Response_fini_function(void * message_memory)
{
  pf400_module_services__srv__MoveJ_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pf400_module_services__srv__MoveJ_Response__rosidl_typesupport_introspection_c__MoveJ_Response_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pf400_module_services__srv__MoveJ_Response, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pf400_module_services__srv__MoveJ_Response__rosidl_typesupport_introspection_c__MoveJ_Response_message_members = {
  "pf400_module_services__srv",  // message namespace
  "MoveJ_Response",  // message name
  1,  // number of fields
  sizeof(pf400_module_services__srv__MoveJ_Response),
  pf400_module_services__srv__MoveJ_Response__rosidl_typesupport_introspection_c__MoveJ_Response_message_member_array,  // message members
  pf400_module_services__srv__MoveJ_Response__rosidl_typesupport_introspection_c__MoveJ_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  pf400_module_services__srv__MoveJ_Response__rosidl_typesupport_introspection_c__MoveJ_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pf400_module_services__srv__MoveJ_Response__rosidl_typesupport_introspection_c__MoveJ_Response_message_type_support_handle = {
  0,
  &pf400_module_services__srv__MoveJ_Response__rosidl_typesupport_introspection_c__MoveJ_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pf400_module_services
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pf400_module_services, srv, MoveJ_Response)() {
  if (!pf400_module_services__srv__MoveJ_Response__rosidl_typesupport_introspection_c__MoveJ_Response_message_type_support_handle.typesupport_identifier) {
    pf400_module_services__srv__MoveJ_Response__rosidl_typesupport_introspection_c__MoveJ_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pf400_module_services__srv__MoveJ_Response__rosidl_typesupport_introspection_c__MoveJ_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "pf400_module_services/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "pf400_module_services/srv/detail/move_j__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers pf400_module_services__srv__detail__move_j__rosidl_typesupport_introspection_c__MoveJ_service_members = {
  "pf400_module_services__srv",  // service namespace
  "MoveJ",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // pf400_module_services__srv__detail__move_j__rosidl_typesupport_introspection_c__MoveJ_Request_message_type_support_handle,
  NULL  // response message
  // pf400_module_services__srv__detail__move_j__rosidl_typesupport_introspection_c__MoveJ_Response_message_type_support_handle
};

static rosidl_service_type_support_t pf400_module_services__srv__detail__move_j__rosidl_typesupport_introspection_c__MoveJ_service_type_support_handle = {
  0,
  &pf400_module_services__srv__detail__move_j__rosidl_typesupport_introspection_c__MoveJ_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pf400_module_services, srv, MoveJ_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pf400_module_services, srv, MoveJ_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pf400_module_services
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pf400_module_services, srv, MoveJ)() {
  if (!pf400_module_services__srv__detail__move_j__rosidl_typesupport_introspection_c__MoveJ_service_type_support_handle.typesupport_identifier) {
    pf400_module_services__srv__detail__move_j__rosidl_typesupport_introspection_c__MoveJ_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)pf400_module_services__srv__detail__move_j__rosidl_typesupport_introspection_c__MoveJ_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pf400_module_services, srv, MoveJ_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pf400_module_services, srv, MoveJ_Response)()->data;
  }

  return &pf400_module_services__srv__detail__move_j__rosidl_typesupport_introspection_c__MoveJ_service_type_support_handle;
}
