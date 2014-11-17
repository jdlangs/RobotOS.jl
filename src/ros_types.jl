#Generate Julia composite types for ROS messages

msg_classes = Dict{String, PyObject}()
msg_deps = Dict{String, Set{String}}()
msg_builtin_types = Dict{String, (Symbol, Any)} (
    "bool"    => (:Bool,        false),
    "int8"    => (:Int8,        zero(Int8)),
    "int16"   => (:Int16,       zero(Int16)),
    "int32"   => (:Int32,       zero(Int32)),
    "int64"   => (:Int64,       zero(Int64)),
    "uint8"   => (:Uint8,       zero(Uint8)),
    "uint16"  => (:Uint16,      zero(Uint16)),
    "uint32"  => (:Uint32,      zero(Uint32)),
    "uint64"  => (:Uint64,      zero(Uint64)),
    "float32" => (:Float32,     zero(Float32)),
    "float64" => (:Float64,     zero(Float64)),
    "string"  => (:ASCIIString, ""),
    "time"    => (:Float64,     0.0),
    )
msg_modules = String[]
mod_deps = Dict{String, Set{String}}()

function genmsgs(m::Dict{String, Vector{String}})
    for (pkg, names) in m
        for n in names
            msgtype = pkg * "/" * n
            importmsg(msgtype)
        end
    end
    modlist = order(mod_deps)
    println("modules: $modlist")
    for mod in modlist
        modtypes = filter(Regex("^$mod/\\w+\$"), msg_deps)
        mtypelist = order(modtypes)
        println(mod)
        println(mtypelist)
        buildmodule(mod, mtypelist)
    end
end

#Produce an order of the keys of d that respect their dependencies
function order(d::Dict{String, Set{String}})
    trecurse!(currlist, d, t) = begin
        if ! (t in currlist)
            if haskey(d, t) #do dependencies first
                for dt in d[t]
                    trecurse!(currlist, d, dt)
                end
                #Now it's ok to add it
                push!(currlist, t)
            end
        end
    end
    tlist = ASCIIString[]
    for t in keys(d)
        trecurse!(tlist, d, t)
    end
    tlist
end

function pkg_msg_strs(msgtype::String)
    if ! ismsg(msgtype)
        error("Incorrect message type '$msgtype', use 'package_name/message'")
    end
    split(msgtype, '/')
end
ismsg(s::String) = ismatch(r"^\w+/\w+$", s)

#Recursively import all needed messages for a given message
function importmsg(msgtype::String)
    if ! haskey(msg_classes,msgtype)
        println("msg import: ", msgtype)
        pkg, msg = pkg_msg_strs(msgtype)
        pkgi = symbol(string("py_",pkg))
        pkgsym = symbol(pkg)

        #Import python ROS module, no effect if already there
        @eval @pyimport $pkgsym.msg as $pkgi
        if ! haskey(mod_deps, pkg)
            mod_deps[pkg] = Set{String}()
        end
        #Store a reference to the message class definition
        msg_classes[msgtype] = @eval $pkgi.pymember($msg)
        msg_deps[msgtype] = Set{String}()
        subtypes = msg_classes[msgtype][:_slot_types]
        for t in subtypes
            if ismsg(t)
                if pkg != pkg_msg_strs(t)[1]
                    push!(mod_deps[pkg], pkg_msg_strs(t)[1])
                end
                push!(msg_deps[msgtype], t)

                importmsg(t)
            end
        end
    end
end

function buildmodule(modname::String, types::Vector)
    println("Creating new module: ", modname)
    modsym = symbol(modname)
    eval(Expr(:toplevel, :(module ($modsym) end)))
    push!(msg_modules, modname)

    mod = eval(modsym)

    #Import/exports
    eval(mod, Expr(:using, :PyCall))
    eval(mod, Expr(:import, :Base, :convert))
    for m in mod_deps[modname]
        eval(mod, Expr(:using, :., :., symbol(m)))
    end
    exports = Expr(:export)
    for typ in types
        pkg, msg = pkg_msg_strs(typ)
        push!(exports.args, symbol(msg))
    end
    eval(mod, exports)

    for typ in types
        pkg, msg = pkg_msg_strs(typ)
        memnames = msg_classes[typ]["__slots__"]
        memtypes = msg_classes[typ]["_slot_types"]
        members = [zip(memnames, memtypes)...]
        println(members)
        typeexprs = buildtype(msg, members)

        println("New type: $pkg/$msg")
        for ex in typeexprs 
            println(ex)
            eval(mod, ex)
        end
    end
end

#Generate code for a native Julia message type
#  - type/member declarations
#  - default constructor
#  - convert from PyObject
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
            memexpr = :($namesym::$msg)
            defexpr = Expr(:call, msg)
            convexpr = :(jmsg.$namesym = convert($msg, o[$n]))
        else
            btype, arraylen = isbuiltin(typ)
            j_typ, j_def = msg_builtin_types[btype]
            if arraylen >= 0
                memexpr = Expr(:(::), namesym, :(Array{$j_typ, 1}))
                defexpr = :($j_typ[])
                convexpr = :(jmsg.$namesym = convert(Array{$j_typ,1}, o[$n]))
            else
                memexpr = Expr(:(::), namesym, j_typ)
                defexpr = j_def
                convexpr = :(jmsg.$namesym = convert($j_typ, o[$n]))
            end
        end
        push!(typeargs, memexpr)
        push!(consargs, defexpr)
        insert!(convargs, length(convargs), convexpr)
    end
    [typedecl, construct, pyconvert]
end 

function isbuiltin(typ::String)
    arraylen = -1
    arrtest = r"^(\w+)\[(\d*)\]$"
    m = match(arrtest, typ)
    if m != nothing
        btype = m.captures[1]
        if isempty(m.captures[2])
            arraylen = 0
        else
            arraylen = int(m.captures[2])
        end
    else
        btype = typ
    end

    if ! haskey(msg_builtin_types, btype)
        error("ROS message generation, Unknown type: $typ")
    end
    btype, arraylen
end
