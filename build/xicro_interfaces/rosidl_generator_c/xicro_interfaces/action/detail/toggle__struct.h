// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xicro_interfaces:action/Toggle.idl
// generated code does not contain a copyright notice

#ifndef XICRO_INTERFACES__ACTION__DETAIL__TOGGLE__STRUCT_H_
#define XICRO_INTERFACES__ACTION__DETAIL__TOGGLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/Toggle in the package xicro_interfaces.
typedef struct xicro_interfaces__action__Toggle_Goal
{
  uint64_t a;
} xicro_interfaces__action__Toggle_Goal;

// Struct for a sequence of xicro_interfaces__action__Toggle_Goal.
typedef struct xicro_interfaces__action__Toggle_Goal__Sequence
{
  xicro_interfaces__action__Toggle_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xicro_interfaces__action__Toggle_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/Toggle in the package xicro_interfaces.
typedef struct xicro_interfaces__action__Toggle_Result
{
  uint64_t sum;
} xicro_interfaces__action__Toggle_Result;

// Struct for a sequence of xicro_interfaces__action__Toggle_Result.
typedef struct xicro_interfaces__action__Toggle_Result__Sequence
{
  xicro_interfaces__action__Toggle_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xicro_interfaces__action__Toggle_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/Toggle in the package xicro_interfaces.
typedef struct xicro_interfaces__action__Toggle_Feedback
{
  bool flag;
} xicro_interfaces__action__Toggle_Feedback;

// Struct for a sequence of xicro_interfaces__action__Toggle_Feedback.
typedef struct xicro_interfaces__action__Toggle_Feedback__Sequence
{
  xicro_interfaces__action__Toggle_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xicro_interfaces__action__Toggle_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "xicro_interfaces/action/detail/toggle__struct.h"

/// Struct defined in action/Toggle in the package xicro_interfaces.
typedef struct xicro_interfaces__action__Toggle_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  xicro_interfaces__action__Toggle_Goal goal;
} xicro_interfaces__action__Toggle_SendGoal_Request;

// Struct for a sequence of xicro_interfaces__action__Toggle_SendGoal_Request.
typedef struct xicro_interfaces__action__Toggle_SendGoal_Request__Sequence
{
  xicro_interfaces__action__Toggle_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xicro_interfaces__action__Toggle_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Toggle in the package xicro_interfaces.
typedef struct xicro_interfaces__action__Toggle_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} xicro_interfaces__action__Toggle_SendGoal_Response;

// Struct for a sequence of xicro_interfaces__action__Toggle_SendGoal_Response.
typedef struct xicro_interfaces__action__Toggle_SendGoal_Response__Sequence
{
  xicro_interfaces__action__Toggle_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xicro_interfaces__action__Toggle_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Toggle in the package xicro_interfaces.
typedef struct xicro_interfaces__action__Toggle_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} xicro_interfaces__action__Toggle_GetResult_Request;

// Struct for a sequence of xicro_interfaces__action__Toggle_GetResult_Request.
typedef struct xicro_interfaces__action__Toggle_GetResult_Request__Sequence
{
  xicro_interfaces__action__Toggle_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xicro_interfaces__action__Toggle_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "xicro_interfaces/action/detail/toggle__struct.h"

/// Struct defined in action/Toggle in the package xicro_interfaces.
typedef struct xicro_interfaces__action__Toggle_GetResult_Response
{
  int8_t status;
  xicro_interfaces__action__Toggle_Result result;
} xicro_interfaces__action__Toggle_GetResult_Response;

// Struct for a sequence of xicro_interfaces__action__Toggle_GetResult_Response.
typedef struct xicro_interfaces__action__Toggle_GetResult_Response__Sequence
{
  xicro_interfaces__action__Toggle_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xicro_interfaces__action__Toggle_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "xicro_interfaces/action/detail/toggle__struct.h"

/// Struct defined in action/Toggle in the package xicro_interfaces.
typedef struct xicro_interfaces__action__Toggle_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  xicro_interfaces__action__Toggle_Feedback feedback;
} xicro_interfaces__action__Toggle_FeedbackMessage;

// Struct for a sequence of xicro_interfaces__action__Toggle_FeedbackMessage.
typedef struct xicro_interfaces__action__Toggle_FeedbackMessage__Sequence
{
  xicro_interfaces__action__Toggle_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xicro_interfaces__action__Toggle_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XICRO_INTERFACES__ACTION__DETAIL__TOGGLE__STRUCT_H_
