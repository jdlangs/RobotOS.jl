#Generate Julia composite types for ROS messages

msg_instances = Dict{String, PyObject}()
msg_builtin_types = Dict{String, Symbol} (
    "bool"    => :Bool,
    "int8"    => :Int8,
    "int16"   => :Int16,
    "int32"   => :Int32,
    "int64"   => :Int64,
    "uint8"   => :Uint8,
    "uint16"  => :Uint16,
    "uint32"  => :Uint32,
    "uint64"  => :Uint64,
    "float32" => :Float32,
    "float64" => :Float64,
    "string"  => :ASCIIString,
    "time"    => :Float64,
    )
msg_typ_defaults = Dict{String, Any} (
    "bool"    => false,
    "int8"    => zero(Int8),
    "int16"   => zero(Int16),
    "int32"   => zero(Int32),
    "int64"   => zero(Int64),
    "uint8"   => zero(Uint8),
    "uint16"  => zero(Uint16),
    "uint32"  => zero(Uint32),
    "uint64"  => zero(Uint64),
    "float32" => zero(Float32),
    "float64" => zero(Float64),
    "string"  => "",
    "time"    => 0.0,
    )
msg_modules = Set{String}()

type Time
    secs::Int
    nsecs::Int
end

function pkg_msg_strs(msgtype::String)
    if ! ismsg(msgtype)
        error("Incorrect message type '$msgtype', use 'package_name/message'")
    end
    split(msgtype, '/')
end
ismsg(s::String) = ismatch(r"^\w+/\w+$", s)

#TODO:
#  check if type defined
#  add export names in module execution
function jltype(msgtype::String)
    importmsg(msgtype)
    pkg, name = pkg_msg_strs(msgtype)
    if eval(:(isdefined(Main.ROS.$(symbol(pkg)), symbol(name))))
        return nothing
    end
    
    memnames = msg_instances[msgtype]["__slots__"]
    memtypes = msg_instances[msgtype]["_slot_types"]
    members = [zip(memnames, memtypes)...]
    for (name,typ) in members
        if ismsg(typ)
            jltype(typ)
        end
    end
    typeexprs = buildtype(name, members) 
    mod = eval(symbol(pkg)) #msg_modules[pkg]
    println("New type: $pkg/$name")

    for ex in typeexprs 
        eval(mod, ex)
    end
    nothing
end

#Recursively import all needed messages for a given message
#TODO: error checking
function importmsg(msgtype::String)
    if ! haskey(msg_instances,msgtype)
        println("msg import: ", msgtype)
        pkg, msg = pkg_msg_strs(msgtype)
        pkgi = symbol(string("py_",pkg))
        pkgsym = symbol(pkg)
        msgsym = symbol(msg)
        @eval @pyimport $pkgsym.msg as $pkgi
        if ! (pkg in msg_modules)
            println("Creating new module: ", pkg)
            eval(Expr(:toplevel, :(module ($pkgsym) end)))
            push!(msg_modules, pkg)
        else
            println("$pkg module exists")
        end

        msg_instances[msgtype] = @eval ($pkgi.$msgsym)()
        subtypes = pycall(msg_instances[msgtype]["_get_types"], PyAny)
        for t in subtypes
            if ismsg(t)
                importmsg(t)
            end
        end
    end
    nothing
end

#TODO:
#  default constructors
#  array types
#  find way to do non-absolute references to the message modules
function buildtype(name::String, members::Vector)
    typeexpr = :(type $(symbol(name)) end)
    construct = :($(symbol(name))() = $(symbol(name))())
    for (n,typ) in members
        namesym = symbol(n)
        memexpr, defexpr =
            if ismsg(typ)
                pkg, msg = map(symbol, pkg_msg_strs(typ))
                (:($namesym::(Main.ROS.$pkg).$msg),
                 Expr(:call, :((Main.ROS.$pkg).$msg)))
            elseif haskey(msg_builtin_types, typ)
                (:($namesym::$(msg_builtin_types[typ])),
                 :($(msg_typ_defaults[typ])))
            else
                error("Unknown type: $typ")
            end
        push!(typeexpr.args[3].args, memexpr)
        push!(construct.args[2].args, defexpr)
    end
    [typeexpr, construct]
end 
