// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pf400_module_services:srv/PeelerActions.idl
// generated code does not contain a copyright notice

#ifndef PF400_MODULE_SERVICES__SRV__DETAIL__PEELER_ACTIONS__STRUCT_H_
#define PF400_MODULE_SERVICES__SRV__DETAIL__PEELER_ACTIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'action_request'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PeelerActions in the package pf400_module_services.
typedef struct pf400_module_services__srv__PeelerActions_Request
{
  rosidl_runtime_c__String action_request;
} pf400_module_services__srv__PeelerActions_Request;

// Struct for a sequence of pf400_module_services__srv__PeelerActions_Request.
typedef struct pf400_module_services__srv__PeelerActions_Request__Sequence
{
  pf400_module_services__srv__PeelerActions_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pf400_module_services__srv__PeelerActions_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/PeelerActions in the package pf400_module_services.
typedef struct pf400_module_services__srv__PeelerActions_Response
{
  bool action_response;
} pf400_module_services__srv__PeelerActions_Response;

// Struct for a sequence of pf400_module_services__srv__PeelerActions_Response.
typedef struct pf400_module_services__srv__PeelerActions_Response__Sequence
{
  pf400_module_services__srv__PeelerActions_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pf400_module_services__srv__PeelerActions_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PF400_MODULE_SERVICES__SRV__DETAIL__PEELER_ACTIONS__STRUCT_H_
