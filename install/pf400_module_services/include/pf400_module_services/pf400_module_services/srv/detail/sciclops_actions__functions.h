// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from pf400_module_services:srv/SciclopsActions.idl
// generated code does not contain a copyright notice

#ifndef PF400_MODULE_SERVICES__SRV__DETAIL__SCICLOPS_ACTIONS__FUNCTIONS_H_
#define PF400_MODULE_SERVICES__SRV__DETAIL__SCICLOPS_ACTIONS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "pf400_module_services/msg/rosidl_generator_c__visibility_control.h"

#include "pf400_module_services/srv/detail/sciclops_actions__struct.h"

/// Initialize srv/SciclopsActions message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pf400_module_services__srv__SciclopsActions_Request
 * )) before or use
 * pf400_module_services__srv__SciclopsActions_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
bool
pf400_module_services__srv__SciclopsActions_Request__init(pf400_module_services__srv__SciclopsActions_Request * msg);

/// Finalize srv/SciclopsActions message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
void
pf400_module_services__srv__SciclopsActions_Request__fini(pf400_module_services__srv__SciclopsActions_Request * msg);

/// Create srv/SciclopsActions message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pf400_module_services__srv__SciclopsActions_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
pf400_module_services__srv__SciclopsActions_Request *
pf400_module_services__srv__SciclopsActions_Request__create();

/// Destroy srv/SciclopsActions message.
/**
 * It calls
 * pf400_module_services__srv__SciclopsActions_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
void
pf400_module_services__srv__SciclopsActions_Request__destroy(pf400_module_services__srv__SciclopsActions_Request * msg);

/// Check for srv/SciclopsActions message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
bool
pf400_module_services__srv__SciclopsActions_Request__are_equal(const pf400_module_services__srv__SciclopsActions_Request * lhs, const pf400_module_services__srv__SciclopsActions_Request * rhs);

/// Copy a srv/SciclopsActions message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
bool
pf400_module_services__srv__SciclopsActions_Request__copy(
  const pf400_module_services__srv__SciclopsActions_Request * input,
  pf400_module_services__srv__SciclopsActions_Request * output);

/// Initialize array of srv/SciclopsActions messages.
/**
 * It allocates the memory for the number of elements and calls
 * pf400_module_services__srv__SciclopsActions_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
bool
pf400_module_services__srv__SciclopsActions_Request__Sequence__init(pf400_module_services__srv__SciclopsActions_Request__Sequence * array, size_t size);

/// Finalize array of srv/SciclopsActions messages.
/**
 * It calls
 * pf400_module_services__srv__SciclopsActions_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
void
pf400_module_services__srv__SciclopsActions_Request__Sequence__fini(pf400_module_services__srv__SciclopsActions_Request__Sequence * array);

/// Create array of srv/SciclopsActions messages.
/**
 * It allocates the memory for the array and calls
 * pf400_module_services__srv__SciclopsActions_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
pf400_module_services__srv__SciclopsActions_Request__Sequence *
pf400_module_services__srv__SciclopsActions_Request__Sequence__create(size_t size);

/// Destroy array of srv/SciclopsActions messages.
/**
 * It calls
 * pf400_module_services__srv__SciclopsActions_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
void
pf400_module_services__srv__SciclopsActions_Request__Sequence__destroy(pf400_module_services__srv__SciclopsActions_Request__Sequence * array);

/// Check for srv/SciclopsActions message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
bool
pf400_module_services__srv__SciclopsActions_Request__Sequence__are_equal(const pf400_module_services__srv__SciclopsActions_Request__Sequence * lhs, const pf400_module_services__srv__SciclopsActions_Request__Sequence * rhs);

/// Copy an array of srv/SciclopsActions messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
bool
pf400_module_services__srv__SciclopsActions_Request__Sequence__copy(
  const pf400_module_services__srv__SciclopsActions_Request__Sequence * input,
  pf400_module_services__srv__SciclopsActions_Request__Sequence * output);

/// Initialize srv/SciclopsActions message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pf400_module_services__srv__SciclopsActions_Response
 * )) before or use
 * pf400_module_services__srv__SciclopsActions_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
bool
pf400_module_services__srv__SciclopsActions_Response__init(pf400_module_services__srv__SciclopsActions_Response * msg);

/// Finalize srv/SciclopsActions message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
void
pf400_module_services__srv__SciclopsActions_Response__fini(pf400_module_services__srv__SciclopsActions_Response * msg);

/// Create srv/SciclopsActions message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pf400_module_services__srv__SciclopsActions_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
pf400_module_services__srv__SciclopsActions_Response *
pf400_module_services__srv__SciclopsActions_Response__create();

/// Destroy srv/SciclopsActions message.
/**
 * It calls
 * pf400_module_services__srv__SciclopsActions_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
void
pf400_module_services__srv__SciclopsActions_Response__destroy(pf400_module_services__srv__SciclopsActions_Response * msg);

/// Check for srv/SciclopsActions message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
bool
pf400_module_services__srv__SciclopsActions_Response__are_equal(const pf400_module_services__srv__SciclopsActions_Response * lhs, const pf400_module_services__srv__SciclopsActions_Response * rhs);

/// Copy a srv/SciclopsActions message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
bool
pf400_module_services__srv__SciclopsActions_Response__copy(
  const pf400_module_services__srv__SciclopsActions_Response * input,
  pf400_module_services__srv__SciclopsActions_Response * output);

/// Initialize array of srv/SciclopsActions messages.
/**
 * It allocates the memory for the number of elements and calls
 * pf400_module_services__srv__SciclopsActions_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
bool
pf400_module_services__srv__SciclopsActions_Response__Sequence__init(pf400_module_services__srv__SciclopsActions_Response__Sequence * array, size_t size);

/// Finalize array of srv/SciclopsActions messages.
/**
 * It calls
 * pf400_module_services__srv__SciclopsActions_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
void
pf400_module_services__srv__SciclopsActions_Response__Sequence__fini(pf400_module_services__srv__SciclopsActions_Response__Sequence * array);

/// Create array of srv/SciclopsActions messages.
/**
 * It allocates the memory for the array and calls
 * pf400_module_services__srv__SciclopsActions_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
pf400_module_services__srv__SciclopsActions_Response__Sequence *
pf400_module_services__srv__SciclopsActions_Response__Sequence__create(size_t size);

/// Destroy array of srv/SciclopsActions messages.
/**
 * It calls
 * pf400_module_services__srv__SciclopsActions_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
void
pf400_module_services__srv__SciclopsActions_Response__Sequence__destroy(pf400_module_services__srv__SciclopsActions_Response__Sequence * array);

/// Check for srv/SciclopsActions message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
bool
pf400_module_services__srv__SciclopsActions_Response__Sequence__are_equal(const pf400_module_services__srv__SciclopsActions_Response__Sequence * lhs, const pf400_module_services__srv__SciclopsActions_Response__Sequence * rhs);

/// Copy an array of srv/SciclopsActions messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_pf400_module_services
bool
pf400_module_services__srv__SciclopsActions_Response__Sequence__copy(
  const pf400_module_services__srv__SciclopsActions_Response__Sequence * input,
  pf400_module_services__srv__SciclopsActions_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PF400_MODULE_SERVICES__SRV__DETAIL__SCICLOPS_ACTIONS__FUNCTIONS_H_
