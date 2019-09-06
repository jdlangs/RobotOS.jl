#typegen.jl
using Libdl

struct ROSIDLTypeSupport
    identifer::Cstring
    data::Ptr{Nothing}
    func::Ptr{Nothing}
end

struct ROSIDLMessageMember
    name::Cstring
    type_id::Cuchar
    string_upper_bound::Cuint
    members::Ptr{Nothing}
    is_array::Bool
    array_size::Cuint
    is_upper_bound::Bool
    offset::Cuint
    default_value::Ptr{Nothing}
    size_function::Ptr{Nothing}
    get_const_function::Ptr{Nothing}
    get_function::Ptr{Nothing}
    resize_function::Ptr{Nothing}
end

struct ROSIDLMessageMembers
    namespace::Cstring
    name::Cstring
    member_count::Cuint
    size_of::Cuint
    members::Ptr{ROSIDLMessageMember}
end


function get_message_info(pkg, msg)
    func_sym = Symbol("rosidl_typesupport_introspection_c__", "get_message_type_support_handle__", pkg, "__msg__", msg)
    lib = Libdl.dlopen(string("lib", pkg, "__rosidl_typesupport_introspection_c"))
    func_handle = Libdl.dlsym(lib, func_sym)

    typesupport = unsafe_load(ccall(func_handle, Ptr{ROSIDLTypeSupport}, ()))
    return unsafe_load(convert(Ptr{ROSIDLMessageMembers}, typesupport.data))
end

function print_msg_info(info::ROSIDLMessageMembers)
    println(unsafe_string(info.namespace), "/", unsafe_string(info.name), ": ", info.member_count, " members")
end
