// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from xicro_interfaces:action/Toggle.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "xicro_interfaces/action/detail/toggle__struct.h"
#include "xicro_interfaces/action/detail/toggle__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace xicro_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Toggle_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Toggle_Goal_type_support_ids_t;

static const _Toggle_Goal_type_support_ids_t _Toggle_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Toggle_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Toggle_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Toggle_Goal_type_support_symbol_names_t _Toggle_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xicro_interfaces, action, Toggle_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xicro_interfaces, action, Toggle_Goal)),
  }
};

typedef struct _Toggle_Goal_type_support_data_t
{
  void * data[2];
} _Toggle_Goal_type_support_data_t;

static _Toggle_Goal_type_support_data_t _Toggle_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Toggle_Goal_message_typesupport_map = {
  2,
  "xicro_interfaces",
  &_Toggle_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_Toggle_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_Toggle_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Toggle_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Toggle_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xicro_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xicro_interfaces, action, Toggle_Goal)() {
  return &::xicro_interfaces::action::rosidl_typesupport_c::Toggle_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xicro_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Toggle_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Toggle_Result_type_support_ids_t;

static const _Toggle_Result_type_support_ids_t _Toggle_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Toggle_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Toggle_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Toggle_Result_type_support_symbol_names_t _Toggle_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xicro_interfaces, action, Toggle_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xicro_interfaces, action, Toggle_Result)),
  }
};

typedef struct _Toggle_Result_type_support_data_t
{
  void * data[2];
} _Toggle_Result_type_support_data_t;

static _Toggle_Result_type_support_data_t _Toggle_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Toggle_Result_message_typesupport_map = {
  2,
  "xicro_interfaces",
  &_Toggle_Result_message_typesupport_ids.typesupport_identifier[0],
  &_Toggle_Result_message_typesupport_symbol_names.symbol_name[0],
  &_Toggle_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Toggle_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Toggle_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xicro_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xicro_interfaces, action, Toggle_Result)() {
  return &::xicro_interfaces::action::rosidl_typesupport_c::Toggle_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xicro_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Toggle_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Toggle_Feedback_type_support_ids_t;

static const _Toggle_Feedback_type_support_ids_t _Toggle_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Toggle_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Toggle_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Toggle_Feedback_type_support_symbol_names_t _Toggle_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xicro_interfaces, action, Toggle_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xicro_interfaces, action, Toggle_Feedback)),
  }
};

typedef struct _Toggle_Feedback_type_support_data_t
{
  void * data[2];
} _Toggle_Feedback_type_support_data_t;

static _Toggle_Feedback_type_support_data_t _Toggle_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Toggle_Feedback_message_typesupport_map = {
  2,
  "xicro_interfaces",
  &_Toggle_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_Toggle_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_Toggle_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Toggle_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Toggle_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xicro_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xicro_interfaces, action, Toggle_Feedback)() {
  return &::xicro_interfaces::action::rosidl_typesupport_c::Toggle_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xicro_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Toggle_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Toggle_SendGoal_Request_type_support_ids_t;

static const _Toggle_SendGoal_Request_type_support_ids_t _Toggle_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Toggle_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Toggle_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Toggle_SendGoal_Request_type_support_symbol_names_t _Toggle_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xicro_interfaces, action, Toggle_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xicro_interfaces, action, Toggle_SendGoal_Request)),
  }
};

typedef struct _Toggle_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _Toggle_SendGoal_Request_type_support_data_t;

static _Toggle_SendGoal_Request_type_support_data_t _Toggle_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Toggle_SendGoal_Request_message_typesupport_map = {
  2,
  "xicro_interfaces",
  &_Toggle_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Toggle_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Toggle_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Toggle_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Toggle_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xicro_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xicro_interfaces, action, Toggle_SendGoal_Request)() {
  return &::xicro_interfaces::action::rosidl_typesupport_c::Toggle_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xicro_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Toggle_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Toggle_SendGoal_Response_type_support_ids_t;

static const _Toggle_SendGoal_Response_type_support_ids_t _Toggle_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Toggle_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Toggle_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Toggle_SendGoal_Response_type_support_symbol_names_t _Toggle_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xicro_interfaces, action, Toggle_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xicro_interfaces, action, Toggle_SendGoal_Response)),
  }
};

typedef struct _Toggle_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _Toggle_SendGoal_Response_type_support_data_t;

static _Toggle_SendGoal_Response_type_support_data_t _Toggle_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Toggle_SendGoal_Response_message_typesupport_map = {
  2,
  "xicro_interfaces",
  &_Toggle_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Toggle_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Toggle_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Toggle_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Toggle_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xicro_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xicro_interfaces, action, Toggle_SendGoal_Response)() {
  return &::xicro_interfaces::action::rosidl_typesupport_c::Toggle_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xicro_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Toggle_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Toggle_SendGoal_type_support_ids_t;

static const _Toggle_SendGoal_type_support_ids_t _Toggle_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Toggle_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Toggle_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Toggle_SendGoal_type_support_symbol_names_t _Toggle_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xicro_interfaces, action, Toggle_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xicro_interfaces, action, Toggle_SendGoal)),
  }
};

typedef struct _Toggle_SendGoal_type_support_data_t
{
  void * data[2];
} _Toggle_SendGoal_type_support_data_t;

static _Toggle_SendGoal_type_support_data_t _Toggle_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Toggle_SendGoal_service_typesupport_map = {
  2,
  "xicro_interfaces",
  &_Toggle_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_Toggle_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_Toggle_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Toggle_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Toggle_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xicro_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, xicro_interfaces, action, Toggle_SendGoal)() {
  return &::xicro_interfaces::action::rosidl_typesupport_c::Toggle_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xicro_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Toggle_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Toggle_GetResult_Request_type_support_ids_t;

static const _Toggle_GetResult_Request_type_support_ids_t _Toggle_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Toggle_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Toggle_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Toggle_GetResult_Request_type_support_symbol_names_t _Toggle_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xicro_interfaces, action, Toggle_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xicro_interfaces, action, Toggle_GetResult_Request)),
  }
};

typedef struct _Toggle_GetResult_Request_type_support_data_t
{
  void * data[2];
} _Toggle_GetResult_Request_type_support_data_t;

static _Toggle_GetResult_Request_type_support_data_t _Toggle_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Toggle_GetResult_Request_message_typesupport_map = {
  2,
  "xicro_interfaces",
  &_Toggle_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Toggle_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Toggle_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Toggle_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Toggle_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xicro_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xicro_interfaces, action, Toggle_GetResult_Request)() {
  return &::xicro_interfaces::action::rosidl_typesupport_c::Toggle_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xicro_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Toggle_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Toggle_GetResult_Response_type_support_ids_t;

static const _Toggle_GetResult_Response_type_support_ids_t _Toggle_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Toggle_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Toggle_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Toggle_GetResult_Response_type_support_symbol_names_t _Toggle_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xicro_interfaces, action, Toggle_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xicro_interfaces, action, Toggle_GetResult_Response)),
  }
};

typedef struct _Toggle_GetResult_Response_type_support_data_t
{
  void * data[2];
} _Toggle_GetResult_Response_type_support_data_t;

static _Toggle_GetResult_Response_type_support_data_t _Toggle_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Toggle_GetResult_Response_message_typesupport_map = {
  2,
  "xicro_interfaces",
  &_Toggle_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Toggle_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Toggle_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Toggle_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Toggle_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xicro_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xicro_interfaces, action, Toggle_GetResult_Response)() {
  return &::xicro_interfaces::action::rosidl_typesupport_c::Toggle_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xicro_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Toggle_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Toggle_GetResult_type_support_ids_t;

static const _Toggle_GetResult_type_support_ids_t _Toggle_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Toggle_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Toggle_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Toggle_GetResult_type_support_symbol_names_t _Toggle_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xicro_interfaces, action, Toggle_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xicro_interfaces, action, Toggle_GetResult)),
  }
};

typedef struct _Toggle_GetResult_type_support_data_t
{
  void * data[2];
} _Toggle_GetResult_type_support_data_t;

static _Toggle_GetResult_type_support_data_t _Toggle_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Toggle_GetResult_service_typesupport_map = {
  2,
  "xicro_interfaces",
  &_Toggle_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_Toggle_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_Toggle_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Toggle_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Toggle_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xicro_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, xicro_interfaces, action, Toggle_GetResult)() {
  return &::xicro_interfaces::action::rosidl_typesupport_c::Toggle_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__struct.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace xicro_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Toggle_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Toggle_FeedbackMessage_type_support_ids_t;

static const _Toggle_FeedbackMessage_type_support_ids_t _Toggle_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Toggle_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Toggle_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Toggle_FeedbackMessage_type_support_symbol_names_t _Toggle_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, xicro_interfaces, action, Toggle_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xicro_interfaces, action, Toggle_FeedbackMessage)),
  }
};

typedef struct _Toggle_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _Toggle_FeedbackMessage_type_support_data_t;

static _Toggle_FeedbackMessage_type_support_data_t _Toggle_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Toggle_FeedbackMessage_message_typesupport_map = {
  2,
  "xicro_interfaces",
  &_Toggle_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_Toggle_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_Toggle_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Toggle_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Toggle_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace xicro_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, xicro_interfaces, action, Toggle_FeedbackMessage)() {
  return &::xicro_interfaces::action::rosidl_typesupport_c::Toggle_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "xicro_interfaces/action/toggle.h"
// already included above
// #include "xicro_interfaces/action/detail/toggle__type_support.h"

static rosidl_action_type_support_t _xicro_interfaces__action__Toggle__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, xicro_interfaces, action, Toggle)()
{
  // Thread-safe by always writing the same values to the static struct
  _xicro_interfaces__action__Toggle__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, xicro_interfaces, action, Toggle_SendGoal)();
  _xicro_interfaces__action__Toggle__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, xicro_interfaces, action, Toggle_GetResult)();
  _xicro_interfaces__action__Toggle__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _xicro_interfaces__action__Toggle__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, xicro_interfaces, action, Toggle_FeedbackMessage)();
  _xicro_interfaces__action__Toggle__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_xicro_interfaces__action__Toggle__typesupport_c;
}

#ifdef __cplusplus
}
#endif
