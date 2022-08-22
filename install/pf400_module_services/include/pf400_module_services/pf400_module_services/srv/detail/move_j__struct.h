// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pf400_module_services:srv/MoveJ.idl
// generated code does not contain a copyright notice

#ifndef PF400_MODULE_SERVICES__SRV__DETAIL__MOVE_J__STRUCT_H_
#define PF400_MODULE_SERVICES__SRV__DETAIL__MOVE_J__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'joint_positions'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/MoveJ in the package pf400_module_services.
typedef struct pf400_module_services__srv__MoveJ_Request
{
  rosidl_runtime_c__float__Sequence joint_positions;
} pf400_module_services__srv__MoveJ_Request;

// Struct for a sequence of pf400_module_services__srv__MoveJ_Request.
typedef struct pf400_module_services__srv__MoveJ_Request__Sequence
{
  pf400_module_services__srv__MoveJ_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pf400_module_services__srv__MoveJ_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MoveJ in the package pf400_module_services.
typedef struct pf400_module_services__srv__MoveJ_Response
{
  uint8_t structure_needs_at_least_one_member;
} pf400_module_services__srv__MoveJ_Response;

// Struct for a sequence of pf400_module_services__srv__MoveJ_Response.
typedef struct pf400_module_services__srv__MoveJ_Response__Sequence
{
  pf400_module_services__srv__MoveJ_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pf400_module_services__srv__MoveJ_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PF400_MODULE_SERVICES__SRV__DETAIL__MOVE_J__STRUCT_H_
