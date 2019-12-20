#typegen.jl
using Libdl

const ROSIDLMessageTypesupport = rosidl_message_type_support_t
const ROSIDLMessageMember = rosidl_typesupport_introspection_c__MessageMember
const ROSIDLMessageMembers = rosidl_typesupport_introspection_c__MessageMembers

const TYPEID_TO_STRING = Dict(
    1 => "float",
    2 => "double",
    3 => "long double",
    4 => "char",
    5 => "wchar",
    6 => "bool",
    7 => "octet",
    8 => "uint8",
    9 => "int8",
    10 => "uint16",
    11 => "int16",
    12 => "uint32",
    13 => "int32",
    14 => "uint64",
    15 => "int64",
    16 => "string",
    17 => "wstring",
    18 => "ros message",
)

function get_message_info(pkg, msg)
    func_sym = Symbol("rosidl_typesupport_introspection_c__", "get_message_type_support_handle__", pkg, "__msg__", msg)
    lib = Libdl.dlopen(string("lib", pkg, "__rosidl_typesupport_introspection_c"))
    func_handle = Libdl.dlsym(lib, func_sym)

    typesupport = unsafe_load(ccall(func_handle, Ptr{ROSIDLMessageTypesupport}, ()))
    return unsafe_load(convert(Ptr{ROSIDLMessageMembers}, typesupport.data))
end

function print_msg_info(info::ROSIDLMessageMembers, indent=0)
    println("  "^indent, unsafe_string(info.message_namespace_), "/", unsafe_string(info.message_name_), ": ", info.member_count_, " members")
    for i=1:info.member_count_
        member = unsafe_load(info.members_, i)
        print_msg_member_info(member, indent+1)
        if member.type_id_ == 18
            member_typesupport = unsafe_load(member.members_, 1)
            print_msg_info(unsafe_load(convert(Ptr{ROSIDLMessageMembers}, member_typesupport.data)), indent+1)
        end
    end
end

function print_msg_member_info(member::ROSIDLMessageMember, indent=0)
    println("  "^indent, unsafe_string(member.name_), " :: ", TYPEID_TO_STRING[member.type_id_])
end
