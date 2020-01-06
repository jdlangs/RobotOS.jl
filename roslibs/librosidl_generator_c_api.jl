# Julia wrapper for ROS package 'rosidl_generator_c' API
# Automatically generated using Clang.jl, do not edit manually
# Use `wrap_rospkg` in `clang_wrap.jl` for regeneration

# -----------------------------------------
# Generated wrapper for action_type_support_struct.h
# -----------------------------------------

# -----------------------
# end action_type_support_struct.h
# -----------------------

# -----------------------------------------
# Generated wrapper for message_bounds_struct.h
# -----------------------------------------

function get_message_bounds_handle(handle, identifier)
    ccall((:get_message_bounds_handle, librosidl_generator_c), Ptr{rosidl_message_bounds_t}, (Ptr{rosidl_message_bounds_t}, Cstring), handle, identifier)
end

function get_message_bounds_handle_function(handle, identifier)
    ccall((:get_message_bounds_handle_function, librosidl_generator_c), Ptr{rosidl_message_bounds_t}, (Ptr{rosidl_message_bounds_t}, Cstring), handle, identifier)
end

# -----------------------
# end message_bounds_struct.h
# -----------------------

# -----------------------------------------
# Generated wrapper for message_initialization.h
# -----------------------------------------

# -----------------------
# end message_initialization.h
# -----------------------

# -----------------------------------------
# Generated wrapper for message_type_support_struct.h
# -----------------------------------------

function get_message_typesupport_handle(handle, identifier)
    ccall((:get_message_typesupport_handle, librosidl_generator_c), Ptr{rosidl_message_type_support_t}, (Ptr{rosidl_message_type_support_t}, Cstring), handle, identifier)
end

function get_message_typesupport_handle_function(handle, identifier)
    ccall((:get_message_typesupport_handle_function, librosidl_generator_c), Ptr{rosidl_message_type_support_t}, (Ptr{rosidl_message_type_support_t}, Cstring), handle, identifier)
end

# -----------------------
# end message_type_support_struct.h
# -----------------------

# -----------------------------------------
# Generated wrapper for primitives_sequence.h
# -----------------------------------------

# -----------------------
# end primitives_sequence.h
# -----------------------

# -----------------------------------------
# Generated wrapper for primitives_sequence_functions.h
# -----------------------------------------

function rosidl_generator_c__float__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__float__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__float__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__float__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__float__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__float__Sequence},), sequence)
end

function rosidl_generator_c__double__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__double__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__double__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__double__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__double__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__double__Sequence},), sequence)
end

function rosidl_generator_c__long_double__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__long_double__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__long_double__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__long_double__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__long_double__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__long_double__Sequence},), sequence)
end

function rosidl_generator_c__char__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__char__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__char__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__char__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__char__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__char__Sequence},), sequence)
end

function rosidl_generator_c__wchar__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__wchar__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__wchar__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__wchar__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__wchar__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__wchar__Sequence},), sequence)
end

function rosidl_generator_c__boolean__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__boolean__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__boolean__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__boolean__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__boolean__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__boolean__Sequence},), sequence)
end

function rosidl_generator_c__octet__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__octet__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__octet__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__octet__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__octet__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__octet__Sequence},), sequence)
end

function rosidl_generator_c__uint8__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__uint8__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__uint8__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__uint8__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__uint8__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__uint8__Sequence},), sequence)
end

function rosidl_generator_c__int8__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__int8__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__int8__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__int8__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__int8__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__int8__Sequence},), sequence)
end

function rosidl_generator_c__uint16__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__uint16__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__uint16__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__uint16__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__uint16__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__uint16__Sequence},), sequence)
end

function rosidl_generator_c__int16__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__int16__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__int16__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__int16__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__int16__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__int16__Sequence},), sequence)
end

function rosidl_generator_c__uint32__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__uint32__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__uint32__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__uint32__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__uint32__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__uint32__Sequence},), sequence)
end

function rosidl_generator_c__int32__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__int32__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__int32__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__int32__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__int32__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__int32__Sequence},), sequence)
end

function rosidl_generator_c__uint64__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__uint64__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__uint64__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__uint64__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__uint64__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__uint64__Sequence},), sequence)
end

function rosidl_generator_c__int64__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__int64__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__int64__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__int64__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__int64__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__int64__Sequence},), sequence)
end

function rosidl_generator_c__bool__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__bool__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__boolean__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__bool__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__bool__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__boolean__Sequence},), sequence)
end

function rosidl_generator_c__byte__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__byte__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__octet__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__byte__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__byte__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__octet__Sequence},), sequence)
end

function rosidl_generator_c__float32__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__float32__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__float__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__float32__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__float32__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__float__Sequence},), sequence)
end

function rosidl_generator_c__float64__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__float64__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__double__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__float64__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__float64__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__double__Sequence},), sequence)
end

# -----------------------
# end primitives_sequence_functions.h
# -----------------------

# -----------------------------------------
# Generated wrapper for service_type_support_struct.h
# -----------------------------------------

function get_service_typesupport_handle(handle, identifier)
    ccall((:get_service_typesupport_handle, librosidl_generator_c), Ptr{rosidl_service_type_support_t}, (Ptr{rosidl_service_type_support_t}, Cstring), handle, identifier)
end

function get_service_typesupport_handle_function(handle, identifier)
    ccall((:get_service_typesupport_handle_function, librosidl_generator_c), Ptr{rosidl_service_type_support_t}, (Ptr{rosidl_service_type_support_t}, Cstring), handle, identifier)
end

# -----------------------
# end service_type_support_struct.h
# -----------------------

# -----------------------------------------
# Generated wrapper for string.h
# -----------------------------------------

# -----------------------
# end string.h
# -----------------------

# -----------------------------------------
# Generated wrapper for string_bounds.h
# -----------------------------------------

# -----------------------
# end string_bounds.h
# -----------------------

# -----------------------------------------
# Generated wrapper for string_functions.h
# -----------------------------------------

function rosidl_generator_c__String__init(str)
    ccall((:rosidl_generator_c__String__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__String},), str)
end

function rosidl_generator_c__String__fini(str)
    ccall((:rosidl_generator_c__String__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__String},), str)
end

function rosidl_generator_c__String__assignn(str, value, n::Csize_t)
    ccall((:rosidl_generator_c__String__assignn, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__String}, Cstring, Csize_t), str, value, n)
end

function rosidl_generator_c__String__assign(str, value)
    ccall((:rosidl_generator_c__String__assign, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__String}, Cstring), str, value)
end

function rosidl_generator_c__String__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__String__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__String__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__String__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__String__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__String__Sequence},), sequence)
end

function rosidl_generator_c__String__Sequence__create(size::Csize_t)
    ccall((:rosidl_generator_c__String__Sequence__create, librosidl_generator_c), Ptr{rosidl_generator_c__String__Sequence}, (Csize_t,), size)
end

function rosidl_generator_c__String__Sequence__destroy(sequence)
    ccall((:rosidl_generator_c__String__Sequence__destroy, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__String__Sequence},), sequence)
end

# -----------------------
# end string_functions.h
# -----------------------

# -----------------------------------------
# Generated wrapper for u16string.h
# -----------------------------------------

# -----------------------
# end u16string.h
# -----------------------

# -----------------------------------------
# Generated wrapper for u16string_functions.h
# -----------------------------------------

function rosidl_generator_c__U16String__init(str)
    ccall((:rosidl_generator_c__U16String__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__U16String},), str)
end

function rosidl_generator_c__U16String__fini(str)
    ccall((:rosidl_generator_c__U16String__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__U16String},), str)
end

function rosidl_generator_c__U16String__assignn(str, value, n::Csize_t)
    ccall((:rosidl_generator_c__U16String__assignn, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__U16String}, Ptr{UInt16}, Csize_t), str, value, n)
end

function rosidl_generator_c__U16String__assignn_from_char(str, value, n::Csize_t)
    ccall((:rosidl_generator_c__U16String__assignn_from_char, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__U16String}, Cstring, Csize_t), str, value, n)
end

function rosidl_generator_c__U16String__assign(str, value)
    ccall((:rosidl_generator_c__U16String__assign, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__U16String}, Ptr{UInt16}), str, value)
end

function rosidl_generator_c__U16String__len(value)
    ccall((:rosidl_generator_c__U16String__len, librosidl_generator_c), Csize_t, (Ptr{UInt16},), value)
end

function rosidl_generator_c__U16String__resize(str, n::Csize_t)
    ccall((:rosidl_generator_c__U16String__resize, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__U16String}, Csize_t), str, n)
end

function rosidl_generator_c__U16String__Sequence__init(sequence, size::Csize_t)
    ccall((:rosidl_generator_c__U16String__Sequence__init, librosidl_generator_c), Bool, (Ptr{rosidl_generator_c__U16String__Sequence}, Csize_t), sequence, size)
end

function rosidl_generator_c__U16String__Sequence__fini(sequence)
    ccall((:rosidl_generator_c__U16String__Sequence__fini, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__U16String__Sequence},), sequence)
end

function rosidl_generator_c__U16String__Sequence__create(size::Csize_t)
    ccall((:rosidl_generator_c__U16String__Sequence__create, librosidl_generator_c), Ptr{rosidl_generator_c__U16String__Sequence}, (Csize_t,), size)
end

function rosidl_generator_c__U16String__Sequence__destroy(sequence)
    ccall((:rosidl_generator_c__U16String__Sequence__destroy, librosidl_generator_c), Cvoid, (Ptr{rosidl_generator_c__U16String__Sequence},), sequence)
end

# -----------------------
# end u16string_functions.h
# -----------------------

# -----------------------------------------
# Generated wrapper for visibility_control.h
# -----------------------------------------

# -----------------------
# end visibility_control.h
# -----------------------
