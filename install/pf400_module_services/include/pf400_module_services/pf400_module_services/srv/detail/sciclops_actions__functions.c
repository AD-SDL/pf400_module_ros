// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pf400_module_services:srv/SciclopsActions.idl
// generated code does not contain a copyright notice
#include "pf400_module_services/srv/detail/sciclops_actions__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `action_request`
#include "rosidl_runtime_c/string_functions.h"

bool
pf400_module_services__srv__SciclopsActions_Request__init(pf400_module_services__srv__SciclopsActions_Request * msg)
{
  if (!msg) {
    return false;
  }
  // action_request
  if (!rosidl_runtime_c__String__init(&msg->action_request)) {
    pf400_module_services__srv__SciclopsActions_Request__fini(msg);
    return false;
  }
  return true;
}

void
pf400_module_services__srv__SciclopsActions_Request__fini(pf400_module_services__srv__SciclopsActions_Request * msg)
{
  if (!msg) {
    return;
  }
  // action_request
  rosidl_runtime_c__String__fini(&msg->action_request);
}

bool
pf400_module_services__srv__SciclopsActions_Request__are_equal(const pf400_module_services__srv__SciclopsActions_Request * lhs, const pf400_module_services__srv__SciclopsActions_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // action_request
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->action_request), &(rhs->action_request)))
  {
    return false;
  }
  return true;
}

bool
pf400_module_services__srv__SciclopsActions_Request__copy(
  const pf400_module_services__srv__SciclopsActions_Request * input,
  pf400_module_services__srv__SciclopsActions_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // action_request
  if (!rosidl_runtime_c__String__copy(
      &(input->action_request), &(output->action_request)))
  {
    return false;
  }
  return true;
}

pf400_module_services__srv__SciclopsActions_Request *
pf400_module_services__srv__SciclopsActions_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pf400_module_services__srv__SciclopsActions_Request * msg = (pf400_module_services__srv__SciclopsActions_Request *)allocator.allocate(sizeof(pf400_module_services__srv__SciclopsActions_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pf400_module_services__srv__SciclopsActions_Request));
  bool success = pf400_module_services__srv__SciclopsActions_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pf400_module_services__srv__SciclopsActions_Request__destroy(pf400_module_services__srv__SciclopsActions_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pf400_module_services__srv__SciclopsActions_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pf400_module_services__srv__SciclopsActions_Request__Sequence__init(pf400_module_services__srv__SciclopsActions_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pf400_module_services__srv__SciclopsActions_Request * data = NULL;

  if (size) {
    data = (pf400_module_services__srv__SciclopsActions_Request *)allocator.zero_allocate(size, sizeof(pf400_module_services__srv__SciclopsActions_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pf400_module_services__srv__SciclopsActions_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pf400_module_services__srv__SciclopsActions_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
pf400_module_services__srv__SciclopsActions_Request__Sequence__fini(pf400_module_services__srv__SciclopsActions_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pf400_module_services__srv__SciclopsActions_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

pf400_module_services__srv__SciclopsActions_Request__Sequence *
pf400_module_services__srv__SciclopsActions_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pf400_module_services__srv__SciclopsActions_Request__Sequence * array = (pf400_module_services__srv__SciclopsActions_Request__Sequence *)allocator.allocate(sizeof(pf400_module_services__srv__SciclopsActions_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pf400_module_services__srv__SciclopsActions_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pf400_module_services__srv__SciclopsActions_Request__Sequence__destroy(pf400_module_services__srv__SciclopsActions_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pf400_module_services__srv__SciclopsActions_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pf400_module_services__srv__SciclopsActions_Request__Sequence__are_equal(const pf400_module_services__srv__SciclopsActions_Request__Sequence * lhs, const pf400_module_services__srv__SciclopsActions_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pf400_module_services__srv__SciclopsActions_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pf400_module_services__srv__SciclopsActions_Request__Sequence__copy(
  const pf400_module_services__srv__SciclopsActions_Request__Sequence * input,
  pf400_module_services__srv__SciclopsActions_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pf400_module_services__srv__SciclopsActions_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pf400_module_services__srv__SciclopsActions_Request * data =
      (pf400_module_services__srv__SciclopsActions_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pf400_module_services__srv__SciclopsActions_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pf400_module_services__srv__SciclopsActions_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pf400_module_services__srv__SciclopsActions_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
pf400_module_services__srv__SciclopsActions_Response__init(pf400_module_services__srv__SciclopsActions_Response * msg)
{
  if (!msg) {
    return false;
  }
  // action_response
  return true;
}

void
pf400_module_services__srv__SciclopsActions_Response__fini(pf400_module_services__srv__SciclopsActions_Response * msg)
{
  if (!msg) {
    return;
  }
  // action_response
}

bool
pf400_module_services__srv__SciclopsActions_Response__are_equal(const pf400_module_services__srv__SciclopsActions_Response * lhs, const pf400_module_services__srv__SciclopsActions_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // action_response
  if (lhs->action_response != rhs->action_response) {
    return false;
  }
  return true;
}

bool
pf400_module_services__srv__SciclopsActions_Response__copy(
  const pf400_module_services__srv__SciclopsActions_Response * input,
  pf400_module_services__srv__SciclopsActions_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // action_response
  output->action_response = input->action_response;
  return true;
}

pf400_module_services__srv__SciclopsActions_Response *
pf400_module_services__srv__SciclopsActions_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pf400_module_services__srv__SciclopsActions_Response * msg = (pf400_module_services__srv__SciclopsActions_Response *)allocator.allocate(sizeof(pf400_module_services__srv__SciclopsActions_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pf400_module_services__srv__SciclopsActions_Response));
  bool success = pf400_module_services__srv__SciclopsActions_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pf400_module_services__srv__SciclopsActions_Response__destroy(pf400_module_services__srv__SciclopsActions_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pf400_module_services__srv__SciclopsActions_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pf400_module_services__srv__SciclopsActions_Response__Sequence__init(pf400_module_services__srv__SciclopsActions_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pf400_module_services__srv__SciclopsActions_Response * data = NULL;

  if (size) {
    data = (pf400_module_services__srv__SciclopsActions_Response *)allocator.zero_allocate(size, sizeof(pf400_module_services__srv__SciclopsActions_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pf400_module_services__srv__SciclopsActions_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pf400_module_services__srv__SciclopsActions_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
pf400_module_services__srv__SciclopsActions_Response__Sequence__fini(pf400_module_services__srv__SciclopsActions_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pf400_module_services__srv__SciclopsActions_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

pf400_module_services__srv__SciclopsActions_Response__Sequence *
pf400_module_services__srv__SciclopsActions_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pf400_module_services__srv__SciclopsActions_Response__Sequence * array = (pf400_module_services__srv__SciclopsActions_Response__Sequence *)allocator.allocate(sizeof(pf400_module_services__srv__SciclopsActions_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pf400_module_services__srv__SciclopsActions_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pf400_module_services__srv__SciclopsActions_Response__Sequence__destroy(pf400_module_services__srv__SciclopsActions_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pf400_module_services__srv__SciclopsActions_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pf400_module_services__srv__SciclopsActions_Response__Sequence__are_equal(const pf400_module_services__srv__SciclopsActions_Response__Sequence * lhs, const pf400_module_services__srv__SciclopsActions_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pf400_module_services__srv__SciclopsActions_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pf400_module_services__srv__SciclopsActions_Response__Sequence__copy(
  const pf400_module_services__srv__SciclopsActions_Response__Sequence * input,
  pf400_module_services__srv__SciclopsActions_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pf400_module_services__srv__SciclopsActions_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pf400_module_services__srv__SciclopsActions_Response * data =
      (pf400_module_services__srv__SciclopsActions_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pf400_module_services__srv__SciclopsActions_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pf400_module_services__srv__SciclopsActions_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pf400_module_services__srv__SciclopsActions_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
