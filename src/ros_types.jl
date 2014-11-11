#Generate Julia composite types for ROS messages

msg_instances = Dict{String, PyObject}()
msg_builtin_types = Dict{String, (Symbol, Any)} (
    "bool"    => (:Bool,false),
    "int8"    => (:Int8,zero(Int8)),
    "int16"   => (:Int16,zero(Int16)),
    "int32"   => (:Int32,zero(Int32)),
    "int64"   => (:Int64,zero(Int64)),
    "uint8"   => (:Uint8,zero(Uint8)),
    "uint16"  => (:Uint16,zero(Uint16)),
    "uint32"  => (:Uint32,zero(Uint32)),
    "uint64"  => (:Uint64,zero(Uint64)),
    "float32" => (:Float32,zero(Float32)),
    "float64" => (:Float64,zero(Float64)),
    "string"  => (:ASCIIString,""),
    "time"    => (:Float64,0.0),
    )
msg_modules = String[]

function genmsgs(m::Dict{String, Vector{String}})
    dependencies = Dict{String, Set{String}}()
    for (pkg, names) in m
        println("Module import: ", pkg)
        for n in names
            msgtype = pkg * "/" * n
            dependencies[msgtype] = importmsg(msgtype)
        end
    end
    dependencies
    #typelist = create_order(dependencies)
    #modlist = mod_order(typelist)
    #for mod in modlist
        #buildmodule(mod, m[mod])
    #end
end

function modulelist(d::Dict{String, Set{String}})
    mlrecurse!(currlist, d, pkg) = begin
        if ! (pkg in currlist)
            if haskey(d, pkg) #do dependencies first
                for dpkg in d[pkg]
                    mlrecurse!(currlist, d, dpkg)
                end
            end
            #Now it's ok to add it
            push!(currlist, pkg)
        end
    end
    mlist = String[]
    for p in keys(d)
        mlrecurse!(mlist, d, p)
    end
    mlist
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
    
    memnames = msg_instances[msgtype]["__slots__"]
    memtypes = msg_instances[msgtype]["_slot_types"]
    members = [zip(memnames, memtypes)...]
    for (n,typ) in members
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
#Returns a list of messages this one is (fully) dependent on
function importmsg(msgtype::String)
    mdepends = Set{String}()
    if ! haskey(msg_instances,msgtype)
        println("msg import: ", msgtype)
        pkg, msg = pkg_msg_strs(msgtype)
        pkgi = symbol(string("py_",pkg))
        pkgsym = symbol(pkg)
        msgsym = symbol(msg)

        @eval @pyimport $pkgsym.msg as $pkgi
        msg_instances[msgtype] = @eval ($pkgi.$msgsym)()
        subtypes = pycall(msg_instances[msgtype]["_get_types"], PyAny)
        for t in subtypes
            if ismsg(t)
                push!(mdepends, t)

                subdepends = importmsg(t)
                union!(mdepends, subdepends)
            end
        end
    end
    mdepends
end

function buildmodule(modname::String, types::Vector{String})
    modsym = symbol(modname)
    println("Creating new module: ", pkg)
    eval(Expr(:toplevel, :(module ($modsym) end)))
    push!(msg_modules, modname)

    #Message may have dependencies within the same module
    #Need to build the type creation list in the proper order
    typerecurse(typelist::Vector{(String, Vector{Expr})}, mod, msgtype) = begin
        memnames = msg_instances[msgtype]["__slots__"]
        memtypes = msg_instances[msgtype]["_slot_types"]
        members = [zip(memnames, memtypes)...]
        for (mname,mtyp) in members
            if ismsg(mtyp)
                typerecurse(typelist, mod, mtyp)
            end
        end
        typeexprs = buildtype(msgtype, members) 
        push!(typelist, (msgtype, typeexprs))
    end

    tlist = (String,Vector{Expr})[]
    for typ in types
        msgtype = modname * "/" * typ

        mod = eval(symbol(pkg)) #msg_modules[pkg]
        println("New type: $pkg/$name")

        for ex in typeexprs 
            eval(mod, ex)
        end
    end
end

#Generate code for a native Julia message type
#  - type/member declarations
#  - default constructor
#  - convert from PyObject
#
#TODO:
#  array types
function buildtype(name::String, members::Vector)
    println("Building: $name")

    #Empty expressions
    typedecl = :(
        type $(symbol(name)) 
        end
    )
    construct = :(
        $(symbol(name))() = $(symbol(name))()
    )
    pyconvert = :(
        convert(::Type{$(symbol(name))}, o::PyObject) = begin
            jmsg = $(symbol(name))() 
            jmsg 
        end
    )
    typeargs = typedecl.args[3].args
    consargs = construct.args[2].args
    convargs = pyconvert.args[2].args

    #Now add the type fields and their implications
    for (n,typ) in members
        if typ == "time"
            continue
        end
        println("\tmember: $n :: $typ")
        namesym = symbol(n)
        if ismsg(typ)
            pkg, msg = map(symbol, pkg_msg_strs(typ))
            memexpr = :($namesym::(Main.ROS.$pkg).$msg)
            defexpr = Expr(:call, :((Main.ROS.$pkg).$msg))
            convexpr = :(jmsg.$namesym = convert((Main.ROS.$pkg).$msg, o[$n]))
        elseif haskey(msg_builtin_types, typ)
            j_typ, j_def = msg_builtin_types[typ]
            memexpr = Expr(:(::), namesym, j_typ)
            defexpr = j_def
            convexpr = :(jmsg.$namesym = convert($j_typ, o[$n]))
        else
            error("Unknown type: $typ, something wrong in members?\n  $members")
        end
        push!(typeargs, memexpr)
        push!(consargs, defexpr)
        insert!(convargs, length(convargs), convexpr)
    end
    [typedecl, construct, pyconvert]
end 
