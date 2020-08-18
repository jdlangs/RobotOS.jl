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

const TYPEID_TO_JLTYPE = Dict(
    1 => Float32,
    2 => Float64,
    3 => Float64,
    4 => Char,
    5 => Char,
    6 => Bool,
    7 => Int8,
    8 => UInt8,
    9 => Int8,
    10 => UInt16,
    11 => Int16,
    12 => UInt32,
    13 => Int32,
    14 => UInt64,
    15 => Int64,
    16 => String,
    17 => String,
    18 => Type,
)

function get_message_info(pkg, msg)
    func_sym = Symbol("rosidl_typesupport_introspection_c__", "get_message_type_support_handle__", pkg, "__msg__", msg)
    lib = Libdl.dlopen(string("lib", pkg, "__rosidl_typesupport_introspection_c"))
    func_handle = Libdl.dlsym(lib, func_sym)

    typesupport = unsafe_load(ccall(func_handle, Ptr{ROSIDLMessageTypesupport}, ()))
    return unsafe_load(convert(Ptr{ROSIDLMessageMembers}, typesupport.data))
end

function members(info::ROSIDLMessageMembers)
    (unsafe_load(info.members_, i) for i=1:info.member_count_)
end

function print_msg_info(info::ROSIDLMessageMembers, indent=0)
    println("  "^indent, unsafe_string(info.message_namespace_), "/", unsafe_string(info.message_name_), ": ", info.member_count_, " members")
    for member in members(info)
        print_msg_member_info(member, indent+1)
        if member.type_id_ == 18
            member_typesupport = unsafe_load(member.members_)
            print_msg_info(unsafe_load(convert(Ptr{ROSIDLMessageMembers}, member_typesupport.data)), indent+1)
        end
    end
end

function print_msg_member_info(member::ROSIDLMessageMember, indent=0)
    println("  "^indent, unsafe_string(member.name_), " :: ", TYPEID_TO_STRING[member.type_id_])
end

function msg_member_code(member::ROSIDLMessageMember)
    name = Symbol(unsafe_string(member.name_))
    jltype = TYPEID_TO_JLTYPE[member.type_id_]
    :($name::$jltype)
end

function msg_code(pkg, msg)
    message_info = get_message_info(pkg, msg)
    message_name = Symbol(msg)
    message_members = map(msg_member_code, members(message_info))
    quote
        struct $message_name
            $(message_members...)
        end
    end
end

function create_message(pkg, msg)
    mod_name = Symbol(pkg)
    @__MODULE__().eval(
        :(
            module $mod_name
                $(msg_code(pkg, msg))
            end
        )
    )
end
