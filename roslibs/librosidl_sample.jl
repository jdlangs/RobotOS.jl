# Julia wrapper for ROS package 'rosidl_generator_c' common definitions
# Automatically generated using Clang.jl, do not edit manually
# Use `wrap_rospkg` in `clang_wrap.jl` for regeneration

# Skipping MacroDefinition: ROSIDL_GET_ACTION_TYPE_SUPPORT ( PkgName , Name ) ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME ( rosidl_typesupport_c , PkgName , action , Name ) ( )

const rosidl_service_typesupport_handle_function = Ptr{Cvoid}

struct rosidl_service_type_support_t
    typesupport_identifier::Cstring
    data::Ptr{Cvoid}
    func::rosidl_service_typesupport_handle_function
end

const rosidl_message_typesupport_handle_function = Ptr{Cvoid}

struct rosidl_message_type_support_t
    typesupport_identifier::Cstring
    data::Ptr{Cvoid}
    func::rosidl_message_typesupport_handle_function
end

struct rosidl_action_type_support_t
    goal_service_type_support::Ptr{rosidl_service_type_support_t}
    result_service_type_support::Ptr{rosidl_service_type_support_t}
    cancel_service_type_support::Ptr{rosidl_service_type_support_t}
    feedback_message_type_support::Ptr{rosidl_message_type_support_t}
    status_message_type_support::Ptr{rosidl_message_type_support_t}
end

# Skipping MacroDefinition: ROSIDL_GET_MSG_BOUNDS ( PkgName , MsgSubfolder , MsgName ) ROSIDL_BOUNDS_INTERFACE__MESSAGE_SYMBOL_NAME ( rosidl_typesupport_c , PkgName , MsgSubfolder , MsgName ) ( )

const rosidl_message_bounds_handle_function = Ptr{Cvoid}

struct rosidl_message_bounds_t
    typesupport_identifier::Cstring
    data::Ptr{Cvoid}
    func::rosidl_message_bounds_handle_function
end

@cenum rosidl_enum::UInt32 begin
    VAL0 = 0
    VAL1 = 1
    VAL2 = 2
    VAL3 = 3
end


# Skipping MacroDefinition: ROSIDL_GET_MSG_TYPE_SUPPORT ( PkgName , MsgSubfolder , MsgName ) ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME ( rosidl_typesupport_c , PkgName , MsgSubfolder , MsgName ) ( )
# Skipping MacroDefinition: ROSIDL_GENERATOR_C__PRIMITIVE_SEQUENCE ( STRUCT_NAME , TYPE_NAME ) typedef struct rosidl_generator_c__ ## STRUCT_NAME ## __Sequence { TYPE_NAME * data ; size_t size ; /*!< The number of valid items in data */ size_t capacity ; /*!< The number of allocated items in data */ } rosidl_generator_c__ ## STRUCT_NAME ## __Sequence ;

struct rosidl_generator_c__float__Sequence
    data::Ptr{Cfloat}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__double__Sequence
    data::Ptr{Cdouble}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__long_double__Sequence
    data::Ptr{Float64}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__char__Sequence
    data::Ptr{UInt8}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__wchar__Sequence
    data::Ptr{UInt16}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__boolean__Sequence
    data::Ptr{Bool}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__octet__Sequence
    data::Ptr{UInt8}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__uint8__Sequence
    data::Ptr{UInt8}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__int8__Sequence
    data::Ptr{Int8}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__uint16__Sequence
    data::Ptr{UInt16}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__int16__Sequence
    data::Ptr{Int16}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__uint32__Sequence
    data::Ptr{UInt32}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__int32__Sequence
    data::Ptr{Int32}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__uint64__Sequence
    data::Ptr{UInt64}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__int64__Sequence
    data::Ptr{Int64}
    size::Csize_t
    capacity::Csize_t
end

const rosidl_generator_c__bool__Sequence = rosidl_generator_c__boolean__Sequence
const rosidl_generator_c__byte__Sequence = rosidl_generator_c__octet__Sequence
const rosidl_generator_c__float32__Sequence = rosidl_generator_c__float__Sequence
const rosidl_generator_c__float64__Sequence = rosidl_generator_c__double__Sequence

# Skipping MacroDefinition: ROSIDL_GENERATOR_C__DECLARE_PRIMITIVE_SEQUENCE_FUNCTIONS ( STRUCT_NAME , TYPE_NAME ) ROSIDL_GENERATOR_C_PUBLIC bool rosidl_generator_c__ ## STRUCT_NAME ## __Sequence__init ( rosidl_generator_c__ ## STRUCT_NAME ## __Sequence * sequence , size_t size ) ; ROSIDL_GENERATOR_C_PUBLIC void rosidl_generator_c__ ## STRUCT_NAME ## __Sequence__fini ( rosidl_generator_c__ ## STRUCT_NAME ## __Sequence * sequence ) ;
# Skipping MacroDefinition: ROSIDL_GET_SRV_TYPE_SUPPORT ( PkgName , SrvSubfolder , SrvName ) ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME ( rosidl_typesupport_c , PkgName , SrvSubfolder , SrvName ) ( )

struct rosidl_generator_c__String
    data::Cstring
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__String__Sequence
    data::Ptr{rosidl_generator_c__String}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__String__bounds
    bounds::Csize_t
end

struct rosidl_generator_c__U16String
    data::Ptr{uint_least16_t}
    size::Csize_t
    capacity::Csize_t
end

struct rosidl_generator_c__U16String__Sequence
    data::Ptr{rosidl_generator_c__U16String}
    size::Csize_t
    capacity::Csize_t
end

# Skipping MacroDefinition: ROSIDL_GENERATOR_C_EXPORT __attribute__ ( ( visibility ( "default" ) ) )
# Skipping MacroDefinition: ROSIDL_GENERATOR_C_PUBLIC __attribute__ ( ( visibility ( "default" ) ) )
# Skipping MacroDefinition: ROSIDL_GENERATOR_C_LOCAL __attribute__ ( ( visibility ( "hidden" ) ) )
