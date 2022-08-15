// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pf400_module_services:srv/MoveJ.idl
// generated code does not contain a copyright notice
#include "pf400_module_services/srv/detail/move_j__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `joint_positions`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
pf400_module_services__srv__MoveJ_Request__init(pf400_module_services__srv__MoveJ_Request * msg)
{
  if (!msg) {
    return false;
  }
  // joint_positions
  if (!rosidl_runtime_c__float__Sequence__init(&msg->joint_positions, 0)) {
    pf400_module_services__srv__MoveJ_Request__fini(msg);
    return false;
  }
  return true;
}

void
pf400_module_services__srv__MoveJ_Request__fini(pf400_module_services__srv__MoveJ_Request * msg)
{
  if (!msg) {
    return;
  }
  // joint_positions
  rosidl_runtime_c__float__Sequence__fini(&msg->joint_positions);
}

bool
pf400_module_services__srv__MoveJ_Request__are_equal(const pf400_module_services__srv__MoveJ_Request * lhs, const pf400_module_services__srv__MoveJ_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // joint_positions
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->joint_positions), &(rhs->joint_positions)))
  {
    return false;
  }
  return true;
}

bool
pf400_module_services__srv__MoveJ_Request__copy(
  const pf400_module_services__srv__MoveJ_Request * input,
  pf400_module_services__srv__MoveJ_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // joint_positions
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->joint_positions), &(output->joint_positions)))
  {
    return false;
  }
  return true;
}

pf400_module_services__srv__MoveJ_Request *
pf400_module_services__srv__MoveJ_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pf400_module_services__srv__MoveJ_Request * msg = (pf400_module_services__srv__MoveJ_Request *)allocator.allocate(sizeof(pf400_module_services__srv__MoveJ_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pf400_module_services__srv__MoveJ_Request));
  bool success = pf400_module_services__srv__MoveJ_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pf400_module_services__srv__MoveJ_Request__destroy(pf400_module_services__srv__MoveJ_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pf400_module_services__srv__MoveJ_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pf400_module_services__srv__MoveJ_Request__Sequence__init(pf400_module_services__srv__MoveJ_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pf400_module_services__srv__MoveJ_Request * data = NULL;

  if (size) {
    data = (pf400_module_services__srv__MoveJ_Request *)allocator.zero_allocate(size, sizeof(pf400_module_services__srv__MoveJ_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pf400_module_services__srv__MoveJ_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pf400_module_services__srv__MoveJ_Request__fini(&data[i - 1]);
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
pf400_module_services__srv__MoveJ_Request__Sequence__fini(pf400_module_services__srv__MoveJ_Request__Sequence * array)
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
      pf400_module_services__srv__MoveJ_Request__fini(&array->data[i]);
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

pf400_module_services__srv__MoveJ_Request__Sequence *
pf400_module_services__srv__MoveJ_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pf400_module_services__srv__MoveJ_Request__Sequence * array = (pf400_module_services__srv__MoveJ_Request__Sequence *)allocator.allocate(sizeof(pf400_module_services__srv__MoveJ_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pf400_module_services__srv__MoveJ_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pf400_module_services__srv__MoveJ_Request__Sequence__destroy(pf400_module_services__srv__MoveJ_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pf400_module_services__srv__MoveJ_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pf400_module_services__srv__MoveJ_Request__Sequence__are_equal(const pf400_module_services__srv__MoveJ_Request__Sequence * lhs, const pf400_module_services__srv__MoveJ_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pf400_module_services__srv__MoveJ_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pf400_module_services__srv__MoveJ_Request__Sequence__copy(
  const pf400_module_services__srv__MoveJ_Request__Sequence * input,
  pf400_module_services__srv__MoveJ_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pf400_module_services__srv__MoveJ_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pf400_module_services__srv__MoveJ_Request * data =
      (pf400_module_services__srv__MoveJ_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pf400_module_services__srv__MoveJ_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pf400_module_services__srv__MoveJ_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pf400_module_services__srv__MoveJ_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
pf400_module_services__srv__MoveJ_Response__init(pf400_module_services__srv__MoveJ_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
pf400_module_services__srv__MoveJ_Response__fini(pf400_module_services__srv__MoveJ_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
pf400_module_services__srv__MoveJ_Response__are_equal(const pf400_module_services__srv__MoveJ_Response * lhs, const pf400_module_services__srv__MoveJ_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
pf400_module_services__srv__MoveJ_Response__copy(
  const pf400_module_services__srv__MoveJ_Response * input,
  pf400_module_services__srv__MoveJ_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

pf400_module_services__srv__MoveJ_Response *
pf400_module_services__srv__MoveJ_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pf400_module_services__srv__MoveJ_Response * msg = (pf400_module_services__srv__MoveJ_Response *)allocator.allocate(sizeof(pf400_module_services__srv__MoveJ_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pf400_module_services__srv__MoveJ_Response));
  bool success = pf400_module_services__srv__MoveJ_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pf400_module_services__srv__MoveJ_Response__destroy(pf400_module_services__srv__MoveJ_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pf400_module_services__srv__MoveJ_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pf400_module_services__srv__MoveJ_Response__Sequence__init(pf400_module_services__srv__MoveJ_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pf400_module_services__srv__MoveJ_Response * data = NULL;

  if (size) {
    data = (pf400_module_services__srv__MoveJ_Response *)allocator.zero_allocate(size, sizeof(pf400_module_services__srv__MoveJ_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pf400_module_services__srv__MoveJ_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pf400_module_services__srv__MoveJ_Response__fini(&data[i - 1]);
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
pf400_module_services__srv__MoveJ_Response__Sequence__fini(pf400_module_services__srv__MoveJ_Response__Sequence * array)
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
      pf400_module_services__srv__MoveJ_Response__fini(&array->data[i]);
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

pf400_module_services__srv__MoveJ_Response__Sequence *
pf400_module_services__srv__MoveJ_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pf400_module_services__srv__MoveJ_Response__Sequence * array = (pf400_module_services__srv__MoveJ_Response__Sequence *)allocator.allocate(sizeof(pf400_module_services__srv__MoveJ_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pf400_module_services__srv__MoveJ_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pf400_module_services__srv__MoveJ_Response__Sequence__destroy(pf400_module_services__srv__MoveJ_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pf400_module_services__srv__MoveJ_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pf400_module_services__srv__MoveJ_Response__Sequence__are_equal(const pf400_module_services__srv__MoveJ_Response__Sequence * lhs, const pf400_module_services__srv__MoveJ_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pf400_module_services__srv__MoveJ_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pf400_module_services__srv__MoveJ_Response__Sequence__copy(
  const pf400_module_services__srv__MoveJ_Response__Sequence * input,
  pf400_module_services__srv__MoveJ_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pf400_module_services__srv__MoveJ_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pf400_module_services__srv__MoveJ_Response * data =
      (pf400_module_services__srv__MoveJ_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pf400_module_services__srv__MoveJ_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pf400_module_services__srv__MoveJ_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pf400_module_services__srv__MoveJ_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
