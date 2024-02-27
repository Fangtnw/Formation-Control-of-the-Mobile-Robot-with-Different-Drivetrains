// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xicro_interfaces:action/Toggle.idl
// generated code does not contain a copyright notice

#ifndef XICRO_INTERFACES__ACTION__DETAIL__TOGGLE__BUILDER_HPP_
#define XICRO_INTERFACES__ACTION__DETAIL__TOGGLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xicro_interfaces/action/detail/toggle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xicro_interfaces
{

namespace action
{

namespace builder
{

class Init_Toggle_Goal_a
{
public:
  Init_Toggle_Goal_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::xicro_interfaces::action::Toggle_Goal a(::xicro_interfaces::action::Toggle_Goal::_a_type arg)
  {
    msg_.a = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xicro_interfaces::action::Toggle_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xicro_interfaces::action::Toggle_Goal>()
{
  return xicro_interfaces::action::builder::Init_Toggle_Goal_a();
}

}  // namespace xicro_interfaces


namespace xicro_interfaces
{

namespace action
{

namespace builder
{

class Init_Toggle_Result_sum
{
public:
  Init_Toggle_Result_sum()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::xicro_interfaces::action::Toggle_Result sum(::xicro_interfaces::action::Toggle_Result::_sum_type arg)
  {
    msg_.sum = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xicro_interfaces::action::Toggle_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xicro_interfaces::action::Toggle_Result>()
{
  return xicro_interfaces::action::builder::Init_Toggle_Result_sum();
}

}  // namespace xicro_interfaces


namespace xicro_interfaces
{

namespace action
{

namespace builder
{

class Init_Toggle_Feedback_flag
{
public:
  Init_Toggle_Feedback_flag()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::xicro_interfaces::action::Toggle_Feedback flag(::xicro_interfaces::action::Toggle_Feedback::_flag_type arg)
  {
    msg_.flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xicro_interfaces::action::Toggle_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xicro_interfaces::action::Toggle_Feedback>()
{
  return xicro_interfaces::action::builder::Init_Toggle_Feedback_flag();
}

}  // namespace xicro_interfaces


namespace xicro_interfaces
{

namespace action
{

namespace builder
{

class Init_Toggle_SendGoal_Request_goal
{
public:
  explicit Init_Toggle_SendGoal_Request_goal(::xicro_interfaces::action::Toggle_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::xicro_interfaces::action::Toggle_SendGoal_Request goal(::xicro_interfaces::action::Toggle_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xicro_interfaces::action::Toggle_SendGoal_Request msg_;
};

class Init_Toggle_SendGoal_Request_goal_id
{
public:
  Init_Toggle_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Toggle_SendGoal_Request_goal goal_id(::xicro_interfaces::action::Toggle_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Toggle_SendGoal_Request_goal(msg_);
  }

private:
  ::xicro_interfaces::action::Toggle_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xicro_interfaces::action::Toggle_SendGoal_Request>()
{
  return xicro_interfaces::action::builder::Init_Toggle_SendGoal_Request_goal_id();
}

}  // namespace xicro_interfaces


namespace xicro_interfaces
{

namespace action
{

namespace builder
{

class Init_Toggle_SendGoal_Response_stamp
{
public:
  explicit Init_Toggle_SendGoal_Response_stamp(::xicro_interfaces::action::Toggle_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::xicro_interfaces::action::Toggle_SendGoal_Response stamp(::xicro_interfaces::action::Toggle_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xicro_interfaces::action::Toggle_SendGoal_Response msg_;
};

class Init_Toggle_SendGoal_Response_accepted
{
public:
  Init_Toggle_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Toggle_SendGoal_Response_stamp accepted(::xicro_interfaces::action::Toggle_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Toggle_SendGoal_Response_stamp(msg_);
  }

private:
  ::xicro_interfaces::action::Toggle_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xicro_interfaces::action::Toggle_SendGoal_Response>()
{
  return xicro_interfaces::action::builder::Init_Toggle_SendGoal_Response_accepted();
}

}  // namespace xicro_interfaces


namespace xicro_interfaces
{

namespace action
{

namespace builder
{

class Init_Toggle_GetResult_Request_goal_id
{
public:
  Init_Toggle_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::xicro_interfaces::action::Toggle_GetResult_Request goal_id(::xicro_interfaces::action::Toggle_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xicro_interfaces::action::Toggle_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xicro_interfaces::action::Toggle_GetResult_Request>()
{
  return xicro_interfaces::action::builder::Init_Toggle_GetResult_Request_goal_id();
}

}  // namespace xicro_interfaces


namespace xicro_interfaces
{

namespace action
{

namespace builder
{

class Init_Toggle_GetResult_Response_result
{
public:
  explicit Init_Toggle_GetResult_Response_result(::xicro_interfaces::action::Toggle_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::xicro_interfaces::action::Toggle_GetResult_Response result(::xicro_interfaces::action::Toggle_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xicro_interfaces::action::Toggle_GetResult_Response msg_;
};

class Init_Toggle_GetResult_Response_status
{
public:
  Init_Toggle_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Toggle_GetResult_Response_result status(::xicro_interfaces::action::Toggle_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Toggle_GetResult_Response_result(msg_);
  }

private:
  ::xicro_interfaces::action::Toggle_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xicro_interfaces::action::Toggle_GetResult_Response>()
{
  return xicro_interfaces::action::builder::Init_Toggle_GetResult_Response_status();
}

}  // namespace xicro_interfaces


namespace xicro_interfaces
{

namespace action
{

namespace builder
{

class Init_Toggle_FeedbackMessage_feedback
{
public:
  explicit Init_Toggle_FeedbackMessage_feedback(::xicro_interfaces::action::Toggle_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::xicro_interfaces::action::Toggle_FeedbackMessage feedback(::xicro_interfaces::action::Toggle_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xicro_interfaces::action::Toggle_FeedbackMessage msg_;
};

class Init_Toggle_FeedbackMessage_goal_id
{
public:
  Init_Toggle_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Toggle_FeedbackMessage_feedback goal_id(::xicro_interfaces::action::Toggle_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Toggle_FeedbackMessage_feedback(msg_);
  }

private:
  ::xicro_interfaces::action::Toggle_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::xicro_interfaces::action::Toggle_FeedbackMessage>()
{
  return xicro_interfaces::action::builder::Init_Toggle_FeedbackMessage_goal_id();
}

}  // namespace xicro_interfaces

#endif  // XICRO_INTERFACES__ACTION__DETAIL__TOGGLE__BUILDER_HPP_
