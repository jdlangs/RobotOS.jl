# Julia wrapper for ROS package 'rosidl_typesupport_introspection_c' common definitions
# Automatically generated using Clang.jl, do not edit manually
# Use `wrap_rospkg` in `clang_wrap.jl` for regeneration

struct rosidl_typesupport_introspection_c__MessageMember
    name_::Cstring
    type_id_::UInt8
    string_upper_bound_::Csize_t
    members_::Ptr{rosidl_message_type_support_t}
    is_array_::Bool
    array_size_::Csize_t
    is_upper_bound_::Bool
    offset_::UInt32
    default_value_::Ptr{Cvoid}
    size_function::Ptr{Cvoid}
    get_const_function::Ptr{Cvoid}
    get_function::Ptr{Cvoid}
    resize_function::Ptr{Cvoid}
end

struct rosidl_typesupport_introspection_c__MessageMembers
    message_namespace_::Cstring
    message_name_::Cstring
    member_count_::UInt32
    size_of_::Csize_t
    members_::Ptr{rosidl_typesupport_introspection_c__MessageMember}
    init_function::Ptr{Cvoid}
    fini_function::Ptr{Cvoid}
end

struct rosidl_typesupport_introspection_c__ServiceMembers
    service_namespace_::Cstring
    service_name_::Cstring
    request_members_::Ptr{rosidl_typesupport_introspection_c__MessageMembers}
    response_members_::Ptr{rosidl_typesupport_introspection_c__MessageMembers}
end

# Skipping MacroDefinition: ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT __attribute__ ( ( visibility ( "default" ) ) )
# Skipping MacroDefinition: ROSIDL_TYPESUPPORT_INTROSPECTION_C_PUBLIC __attribute__ ( ( visibility ( "default" ) ) )
# Skipping MacroDefinition: ROSIDL_TYPESUPPORT_INTROSPECTION_C_LOCAL __attribute__ ( ( visibility ( "hidden" ) ) )
