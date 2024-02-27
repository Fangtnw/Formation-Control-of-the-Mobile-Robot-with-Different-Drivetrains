// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xicro_interfaces:action/Toggle.idl
// generated code does not contain a copyright notice

#ifndef XICRO_INTERFACES__ACTION__DETAIL__TOGGLE__TRAITS_HPP_
#define XICRO_INTERFACES__ACTION__DETAIL__TOGGLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "xicro_interfaces/action/detail/toggle__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace xicro_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Toggle_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: a
  {
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Toggle_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Toggle_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xicro_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xicro_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xicro_interfaces::action::Toggle_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  xicro_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xicro_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xicro_interfaces::action::Toggle_Goal & msg)
{
  return xicro_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xicro_interfaces::action::Toggle_Goal>()
{
  return "xicro_interfaces::action::Toggle_Goal";
}

template<>
inline const char * name<xicro_interfaces::action::Toggle_Goal>()
{
  return "xicro_interfaces/action/Toggle_Goal";
}

template<>
struct has_fixed_size<xicro_interfaces::action::Toggle_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<xicro_interfaces::action::Toggle_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<xicro_interfaces::action::Toggle_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace xicro_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Toggle_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: sum
  {
    out << "sum: ";
    rosidl_generator_traits::value_to_yaml(msg.sum, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Toggle_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sum
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sum: ";
    rosidl_generator_traits::value_to_yaml(msg.sum, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Toggle_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xicro_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xicro_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xicro_interfaces::action::Toggle_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  xicro_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xicro_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xicro_interfaces::action::Toggle_Result & msg)
{
  return xicro_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xicro_interfaces::action::Toggle_Result>()
{
  return "xicro_interfaces::action::Toggle_Result";
}

template<>
inline const char * name<xicro_interfaces::action::Toggle_Result>()
{
  return "xicro_interfaces/action/Toggle_Result";
}

template<>
struct has_fixed_size<xicro_interfaces::action::Toggle_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<xicro_interfaces::action::Toggle_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<xicro_interfaces::action::Toggle_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace xicro_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Toggle_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: flag
  {
    out << "flag: ";
    rosidl_generator_traits::value_to_yaml(msg.flag, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Toggle_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flag: ";
    rosidl_generator_traits::value_to_yaml(msg.flag, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Toggle_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xicro_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xicro_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xicro_interfaces::action::Toggle_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  xicro_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xicro_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xicro_interfaces::action::Toggle_Feedback & msg)
{
  return xicro_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xicro_interfaces::action::Toggle_Feedback>()
{
  return "xicro_interfaces::action::Toggle_Feedback";
}

template<>
inline const char * name<xicro_interfaces::action::Toggle_Feedback>()
{
  return "xicro_interfaces/action/Toggle_Feedback";
}

template<>
struct has_fixed_size<xicro_interfaces::action::Toggle_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<xicro_interfaces::action::Toggle_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<xicro_interfaces::action::Toggle_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "xicro_interfaces/action/detail/toggle__traits.hpp"

namespace xicro_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Toggle_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Toggle_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Toggle_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xicro_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xicro_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xicro_interfaces::action::Toggle_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  xicro_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xicro_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xicro_interfaces::action::Toggle_SendGoal_Request & msg)
{
  return xicro_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xicro_interfaces::action::Toggle_SendGoal_Request>()
{
  return "xicro_interfaces::action::Toggle_SendGoal_Request";
}

template<>
inline const char * name<xicro_interfaces::action::Toggle_SendGoal_Request>()
{
  return "xicro_interfaces/action/Toggle_SendGoal_Request";
}

template<>
struct has_fixed_size<xicro_interfaces::action::Toggle_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<xicro_interfaces::action::Toggle_Goal>::value> {};

template<>
struct has_bounded_size<xicro_interfaces::action::Toggle_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<xicro_interfaces::action::Toggle_Goal>::value> {};

template<>
struct is_message<xicro_interfaces::action::Toggle_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace xicro_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Toggle_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Toggle_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Toggle_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xicro_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xicro_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xicro_interfaces::action::Toggle_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  xicro_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xicro_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xicro_interfaces::action::Toggle_SendGoal_Response & msg)
{
  return xicro_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xicro_interfaces::action::Toggle_SendGoal_Response>()
{
  return "xicro_interfaces::action::Toggle_SendGoal_Response";
}

template<>
inline const char * name<xicro_interfaces::action::Toggle_SendGoal_Response>()
{
  return "xicro_interfaces/action/Toggle_SendGoal_Response";
}

template<>
struct has_fixed_size<xicro_interfaces::action::Toggle_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<xicro_interfaces::action::Toggle_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<xicro_interfaces::action::Toggle_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xicro_interfaces::action::Toggle_SendGoal>()
{
  return "xicro_interfaces::action::Toggle_SendGoal";
}

template<>
inline const char * name<xicro_interfaces::action::Toggle_SendGoal>()
{
  return "xicro_interfaces/action/Toggle_SendGoal";
}

template<>
struct has_fixed_size<xicro_interfaces::action::Toggle_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<xicro_interfaces::action::Toggle_SendGoal_Request>::value &&
    has_fixed_size<xicro_interfaces::action::Toggle_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<xicro_interfaces::action::Toggle_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<xicro_interfaces::action::Toggle_SendGoal_Request>::value &&
    has_bounded_size<xicro_interfaces::action::Toggle_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<xicro_interfaces::action::Toggle_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<xicro_interfaces::action::Toggle_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xicro_interfaces::action::Toggle_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace xicro_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Toggle_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Toggle_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Toggle_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xicro_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xicro_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xicro_interfaces::action::Toggle_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  xicro_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xicro_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xicro_interfaces::action::Toggle_GetResult_Request & msg)
{
  return xicro_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xicro_interfaces::action::Toggle_GetResult_Request>()
{
  return "xicro_interfaces::action::Toggle_GetResult_Request";
}

template<>
inline const char * name<xicro_interfaces::action::Toggle_GetResult_Request>()
{
  return "xicro_interfaces/action/Toggle_GetResult_Request";
}

template<>
struct has_fixed_size<xicro_interfaces::action::Toggle_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<xicro_interfaces::action::Toggle_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<xicro_interfaces::action::Toggle_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "xicro_interfaces/action/detail/toggle__traits.hpp"

namespace xicro_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Toggle_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Toggle_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Toggle_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xicro_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xicro_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xicro_interfaces::action::Toggle_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  xicro_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xicro_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xicro_interfaces::action::Toggle_GetResult_Response & msg)
{
  return xicro_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xicro_interfaces::action::Toggle_GetResult_Response>()
{
  return "xicro_interfaces::action::Toggle_GetResult_Response";
}

template<>
inline const char * name<xicro_interfaces::action::Toggle_GetResult_Response>()
{
  return "xicro_interfaces/action/Toggle_GetResult_Response";
}

template<>
struct has_fixed_size<xicro_interfaces::action::Toggle_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<xicro_interfaces::action::Toggle_Result>::value> {};

template<>
struct has_bounded_size<xicro_interfaces::action::Toggle_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<xicro_interfaces::action::Toggle_Result>::value> {};

template<>
struct is_message<xicro_interfaces::action::Toggle_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xicro_interfaces::action::Toggle_GetResult>()
{
  return "xicro_interfaces::action::Toggle_GetResult";
}

template<>
inline const char * name<xicro_interfaces::action::Toggle_GetResult>()
{
  return "xicro_interfaces/action/Toggle_GetResult";
}

template<>
struct has_fixed_size<xicro_interfaces::action::Toggle_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<xicro_interfaces::action::Toggle_GetResult_Request>::value &&
    has_fixed_size<xicro_interfaces::action::Toggle_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<xicro_interfaces::action::Toggle_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<xicro_interfaces::action::Toggle_GetResult_Request>::value &&
    has_bounded_size<xicro_interfaces::action::Toggle_GetResult_Response>::value
  >
{
};

template<>
struct is_service<xicro_interfaces::action::Toggle_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<xicro_interfaces::action::Toggle_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xicro_interfaces::action::Toggle_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "xicro_interfaces/action/detail/toggle__traits.hpp"

namespace xicro_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const Toggle_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Toggle_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Toggle_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xicro_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xicro_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xicro_interfaces::action::Toggle_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  xicro_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xicro_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xicro_interfaces::action::Toggle_FeedbackMessage & msg)
{
  return xicro_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xicro_interfaces::action::Toggle_FeedbackMessage>()
{
  return "xicro_interfaces::action::Toggle_FeedbackMessage";
}

template<>
inline const char * name<xicro_interfaces::action::Toggle_FeedbackMessage>()
{
  return "xicro_interfaces/action/Toggle_FeedbackMessage";
}

template<>
struct has_fixed_size<xicro_interfaces::action::Toggle_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<xicro_interfaces::action::Toggle_Feedback>::value> {};

template<>
struct has_bounded_size<xicro_interfaces::action::Toggle_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<xicro_interfaces::action::Toggle_Feedback>::value> {};

template<>
struct is_message<xicro_interfaces::action::Toggle_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<xicro_interfaces::action::Toggle>
  : std::true_type
{
};

template<>
struct is_action_goal<xicro_interfaces::action::Toggle_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<xicro_interfaces::action::Toggle_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<xicro_interfaces::action::Toggle_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // XICRO_INTERFACES__ACTION__DETAIL__TOGGLE__TRAITS_HPP_
