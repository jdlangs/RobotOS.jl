# Julia wrapper for ROS package 'rmw' common definitions
# Automatically generated using Clang.jl, do not edit manually
# Use `wrap_rospkg` in `clang_wrap.jl` for regeneration

#TODO: make rcutils macros exist
#const RMW_SAFE_FWRITE_TO_STDERR = RCUTILS_SAFE_FWRITE_TO_STDERR
const rmw_initialize_error_handling_thread_local_storage = rcutils_initialize_error_handling_thread_local_storage
const rmw_set_error_state = rcutils_set_error_state

# Skipping MacroDefinition: RMW_CHECK_ARGUMENT_FOR_NULL ( argument , error_return_type ) RCUTILS_CHECK_ARGUMENT_FOR_NULL ( argument , error_return_type )
# Skipping MacroDefinition: RMW_CHECK_FOR_NULL_WITH_MSG ( value , msg , error_statement ) RCUTILS_CHECK_FOR_NULL_WITH_MSG ( value , msg , error_statement )
# Skipping MacroDefinition: RMW_SET_ERROR_MSG ( msg ) RCUTILS_SET_ERROR_MSG ( msg )
# Skipping MacroDefinition: RMW_SET_ERROR_MSG_WITH_FORMAT_STRING ( format_string , ... ) RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING ( format_string , __VA_ARGS__ )

const rmw_error_is_set = rcutils_error_is_set
const rmw_get_error_state = rcutils_get_error_state
const rmw_get_error_string = rcutils_get_error_string
const rmw_reset_error = rcutils_reset_error
const rmw_error_string_t = rcutils_error_string_t
const rmw_error_state_t = rcutils_error_state_t

@cenum rmw_event_type_t::UInt32 begin
    RMW_EVENT_LIVELINESS_CHANGED = 0
    RMW_EVENT_REQUESTED_DEADLINE_MISSED = 1
    RMW_EVENT_LIVELINESS_LOST = 2
    RMW_EVENT_OFFERED_DEADLINE_MISSED = 3
    RMW_EVENT_INVALID = 4
end


struct rmw_event_t
    implementation_identifier::Cstring
    data::Ptr{Cvoid}
    event_type::rmw_event_type_t
end

const rmw_context_impl_t = Cvoid

struct rmw_context_t
    instance_id::UInt64
    implementation_identifier::Cstring
    impl::Ptr{rmw_context_impl_t}
end

const rmw_init_options_impl_t = Cvoid

struct rmw_init_options_t
    instance_id::UInt64
    implementation_identifier::Cstring
    allocator::rcutils_allocator_t
    impl::Ptr{rmw_init_options_impl_t}
end

struct rmw_loaned_message_sequence_t
    message_sequence::Ptr{Cvoid}
    size::Csize_t
    capacity::Csize_t
end

# Skipping MacroDefinition: RMW_STRINGIFY ( x ) RCUTILS_STRINGIFY ( x )

#TODO: rcutils macros
#const RMW_WARN_UNUSED = RCUTILS_WARN_UNUSED

struct rmw_names_and_types_t
    names::rcutils_string_array_t
    types::Ptr{rcutils_string_array_t}
end

const RMW_RET_OK = 0
const RMW_RET_ERROR = 1
const RMW_RET_TIMEOUT = 2
const RMW_RET_UNSUPPORTED = 3
const RMW_RET_BAD_ALLOC = 10
const RMW_RET_INVALID_ARGUMENT = 11
const RMW_RET_INCORRECT_RMW_IMPLEMENTATION = 12
const RMW_RET_NODE_NAME_NON_EXISTENT = 203
const rmw_ret_t = Int32
const rmw_get_zero_initialized_serialized_message = rcutils_get_zero_initialized_uint8_array
const rmw_serialized_message_init = rcutils_uint8_array_init
const rmw_serialized_message_fini = rcutils_uint8_array_fini
const rmw_serialized_message_resize = rcutils_uint8_array_resize
const rmw_serialized_message_t = rcutils_uint8_array_t
const RMW_GID_STORAGE_SIZE = 24

# Skipping MacroDefinition: RMW_QOS_DEADLINE_DEFAULT { 0 , 0 }
# Skipping MacroDefinition: RMW_QOS_LIFESPAN_DEFAULT { 0 , 0 }
# Skipping MacroDefinition: RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT { 0 , 0 }

struct rmw_node_t
    implementation_identifier::Cstring
    data::Ptr{Cvoid}
    name::Cstring
    namespace_::Cstring
    context::Ptr{rmw_context_t}
end

struct rmw_publisher_options_t
    rmw_specific_publisher_payload::Ptr{Cvoid}
end

struct rmw_publisher_t
    implementation_identifier::Cstring
    data::Ptr{Cvoid}
    topic_name::Cstring
    options::rmw_publisher_options_t
    can_loan_messages::Bool
end

struct rmw_subscription_options_t
    rmw_specific_subscription_payload::Ptr{Cvoid}
    ignore_local_publications::Bool
end

struct rmw_subscription_t
    implementation_identifier::Cstring
    data::Ptr{Cvoid}
    topic_name::Cstring
    options::rmw_subscription_options_t
    can_loan_messages::Bool
end

struct rmw_service_t
    implementation_identifier::Cstring
    data::Ptr{Cvoid}
    service_name::Cstring
end

struct rmw_client_t
    implementation_identifier::Cstring
    data::Ptr{Cvoid}
    service_name::Cstring
end

struct rmw_guard_condition_t
    implementation_identifier::Cstring
    data::Ptr{Cvoid}
    context::Ptr{rmw_context_t}
end

struct rmw_publisher_allocation_t
    implementation_identifier::Cstring
    data::Ptr{Cvoid}
end

struct rmw_subscription_allocation_t
    implementation_identifier::Cstring
    data::Ptr{Cvoid}
end

struct rmw_subscriptions_t
    subscriber_count::Csize_t
    subscribers::Ptr{Ptr{Cvoid}}
end

struct rmw_services_t
    service_count::Csize_t
    services::Ptr{Ptr{Cvoid}}
end

struct rmw_clients_t
    client_count::Csize_t
    clients::Ptr{Ptr{Cvoid}}
end

struct rmw_events_t
    event_count::Csize_t
    events::Ptr{Ptr{Cvoid}}
end

struct rmw_guard_conditions_t
    guard_condition_count::Csize_t
    guard_conditions::Ptr{Ptr{Cvoid}}
end

struct rmw_wait_set_t
    implementation_identifier::Cstring
    guard_conditions::Ptr{rmw_guard_conditions_t}
    data::Ptr{Cvoid}
end

struct rmw_request_id_t
    writer_guid::NTuple{16, Int8}
    sequence_number::Int64
end

struct rmw_time_t
    sec::UInt64
    nsec::UInt64
end

@cenum rmw_security_enforcement_policy_t::UInt32 begin
    RMW_SECURITY_ENFORCEMENT_PERMISSIVE = 0
    RMW_SECURITY_ENFORCEMENT_ENFORCE = 1
end


struct rmw_node_security_options_t
    enforce_security::rmw_security_enforcement_policy_t
    security_root_path::Cstring
end

@cenum rmw_qos_reliability_policy_t::UInt32 begin
    RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT = 0
    RMW_QOS_POLICY_RELIABILITY_RELIABLE = 1
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT = 2
    RMW_QOS_POLICY_RELIABILITY_UNKNOWN = 3
end

@cenum rmw_qos_history_policy_t::UInt32 begin
    RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT = 0
    RMW_QOS_POLICY_HISTORY_KEEP_LAST = 1
    RMW_QOS_POLICY_HISTORY_KEEP_ALL = 2
    RMW_QOS_POLICY_HISTORY_UNKNOWN = 3
end

@cenum rmw_qos_durability_policy_t::UInt32 begin
    RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT = 0
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL = 1
    RMW_QOS_POLICY_DURABILITY_VOLATILE = 2
    RMW_QOS_POLICY_DURABILITY_UNKNOWN = 3
end

@cenum rmw_qos_liveliness_policy_t::UInt32 begin
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT = 0
    RMW_QOS_POLICY_LIVELINESS_AUTOMATIC = 1
    RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE = 2
    RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC = 3
    RMW_QOS_POLICY_LIVELINESS_UNKNOWN = 4
end


struct rmw_qos_profile_t
    history::rmw_qos_history_policy_t
    depth::Csize_t
    reliability::rmw_qos_reliability_policy_t
    durability::rmw_qos_durability_policy_t
    deadline::rmw_time_t
    lifespan::rmw_time_t
    liveliness::rmw_qos_liveliness_policy_t
    liveliness_lease_duration::rmw_time_t
    avoid_ros_namespace_conventions::Bool
end

struct rmw_gid_t
    implementation_identifier::Cstring
    data::NTuple{24, UInt8}
end

struct rmw_message_info_t
    publisher_gid::rmw_gid_t
    from_intra_process::Bool
end

struct rmw_message_info_sequence_t
    message_info_sequence::Ptr{rmw_message_info_t}
    size::Csize_t
    capacity::Csize_t
end

@cenum rmw_log_severity_t::UInt32 begin
    RMW_LOG_SEVERITY_DEBUG = 10
    RMW_LOG_SEVERITY_INFO = 20
    RMW_LOG_SEVERITY_WARN = 30
    RMW_LOG_SEVERITY_ERROR = 40
    RMW_LOG_SEVERITY_FATAL = 50
end


struct rmw_liveliness_changed_status_t
    alive_count::Int32
    not_alive_count::Int32
    alive_count_change::Int32
    not_alive_count_change::Int32
end

struct rmw_requested_deadline_missed_status_t
    total_count::Int32
    total_count_change::Int32
end

struct rmw_liveliness_lost_status_t
    total_count::Int32
    total_count_change::Int32
end

struct rmw_offered_deadline_missed_status_t
    total_count::Int32
    total_count_change::Int32
end

const RMW_TOPIC_VALID = 0
const RMW_TOPIC_INVALID_IS_EMPTY_STRING = 1
const RMW_TOPIC_INVALID_NOT_ABSOLUTE = 2
const RMW_TOPIC_INVALID_ENDS_WITH_FORWARD_SLASH = 3
const RMW_TOPIC_INVALID_CONTAINS_UNALLOWED_CHARACTERS = 4
const RMW_TOPIC_INVALID_CONTAINS_REPEATED_FORWARD_SLASH = 5
const RMW_TOPIC_INVALID_NAME_TOKEN_STARTS_WITH_NUMBER = 6
const RMW_TOPIC_INVALID_TOO_LONG = 7

#TODO
# Skipping MacroDefinition: RMW_TOPIC_MAX_NAME_LENGTH 255U /* impl constraint */ - 8U
const RMW_TOPIC_MAX_NAME_LENGTH = 255

const RMW_NAMESPACE_VALID = 0
const RMW_NAMESPACE_INVALID_IS_EMPTY_STRING = 1
const RMW_NAMESPACE_INVALID_NOT_ABSOLUTE = 2
const RMW_NAMESPACE_INVALID_ENDS_WITH_FORWARD_SLASH = 3
const RMW_NAMESPACE_INVALID_CONTAINS_UNALLOWED_CHARACTERS = 4
const RMW_NAMESPACE_INVALID_CONTAINS_REPEATED_FORWARD_SLASH = 5
const RMW_NAMESPACE_INVALID_NAME_TOKEN_STARTS_WITH_NUMBER = 6
const RMW_NAMESPACE_INVALID_TOO_LONG = 7
const RMW_NAMESPACE_MAX_LENGTH = RMW_TOPIC_MAX_NAME_LENGTH - UInt32(2)
const RMW_NODE_NAME_VALID = 0
const RMW_NODE_NAME_INVALID_IS_EMPTY_STRING = 1
const RMW_NODE_NAME_INVALID_CONTAINS_UNALLOWED_CHARACTERS = 2
const RMW_NODE_NAME_INVALID_STARTS_WITH_NUMBER = 3
const RMW_NODE_NAME_INVALID_TOO_LONG = 4
const RMW_NODE_NAME_MAX_NAME_LENGTH = 255

# Skipping MacroDefinition: RMW_EXPORT __attribute__ ( ( visibility ( "default" ) ) )
# Skipping MacroDefinition: RMW_PUBLIC __attribute__ ( ( visibility ( "default" ) ) )
# Skipping MacroDefinition: RMW_LOCAL __attribute__ ( ( visibility ( "hidden" ) ) )

const RMW_AVOID_MEMORY_ALLOCATION = 0
