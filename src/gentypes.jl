#Generate Julia composite types for ROS messages

_msg_classes = Dict{String, PyObject}()
_msg_builtin_types = Dict{String, Symbol} (
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
    "time"    => :Time,
    )
msgzero{T<:Real}(::Type{T}) = zero(T)
msgzero(::Type{ASCIIString}) = ""
msgzero(::Type{Time}) = Time()

function genmsgs(m::Dict{String, Vector{String}})
    mod_deps = Dict{String, Set{String}}()
    msg_deps = Dict{String, Set{String}}()
    for (pkg, names) in m
        for n in names
            msgtype = pkg * "/" * n
            importmsg(msgtype, mod_deps, msg_deps)
        end
    end
    modlist = _order(mod_deps)
    for mod in modlist
        modtypes = filter(Regex("^$mod/\\w+\$"), msg_deps)
        mtypelist = _order(modtypes)
        buildmodule(mod, mod_deps[mod], mtypelist)
    end
end

#Recursively import all needed messages for a given message
function importmsg(msgtype::String, mod_deps, msg_deps)
    if ! haskey(_msg_classes,msgtype)
        pkg, msg = _pkg_msg_strs(msgtype)
        pkgi = symbol(string("py_",pkg))
        pkgsym = symbol(pkg)

        #Import python ROS module, no effect if already there
        @eval @pyimport $pkgsym.msg as $pkgi
        if ! haskey(mod_deps, pkg)
            mod_deps[pkg] = Set{String}()
        end
        #Store a reference to the message class definition
        _msg_classes[msgtype] = @eval $pkgi.pymember($msg)
        msg_deps[msgtype] = Set{String}()
        subtypes = _msg_classes[msgtype][:_slot_types]
        for t in subtypes
            if _ismsg(t)
                t = _check_array_type(t)[1]
                tpkg = _pkg_msg_strs(t)[1]
                if tpkg != pkg #Possibly a new module dependency
                    push!(mod_deps[pkg], tpkg)
                end
                #pushing to a set does not create duplicates
                push!(msg_deps[msgtype], t)

                importmsg(t, mod_deps, msg_deps)
            end
        end
    end
end

#Create the new module and build the new types inside
function buildmodule(modname::String, deps::Set, types::Vector)
    #Create module
    modsym = symbol(modname)
    eval(Expr(:toplevel, :(module ($modsym) end)))
    mod = eval(modsym)

    #Import/exports so the generated code can use external names
    eval(mod, Expr(:using, :PyCall))
    eval(mod, Expr(:block, 
        Expr(:using, :., :., :ROS, :Time),
        Expr(:using, :., :., :ROS, :msgzero),
    ))
    eval(mod, Expr(:import, :Base, :convert))
    for m in deps
        eval(mod, Expr(:using, :., :., symbol(m)))
    end
    exports = Expr(:export)
    for typ in types
        pkg, msg = _pkg_msg_strs(typ)
        push!(exports.args, symbol(msg))
    end
    eval(mod, exports)

    #Type creation
    for typ in types
        pkg, msg = _pkg_msg_strs(typ)
        memnames = _msg_classes[typ]["__slots__"]
        memtypes = _msg_classes[typ]["_slot_types"]
        members = [zip(memnames, memtypes)...]
        typeexprs = buildtype(msg, members)

        for ex in typeexprs 
            eval(mod, ex)
        end
    end
    nothing
end

#Generate code for a native Julia message type
#  - type/member declarations
#  - default constructor
#  - convert to/from PyObject
function buildtype(name::String, members::Vector)
    println("Type: $name")

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
        println("\t$n :: $typ")

        typ, arraylen = _check_array_type(typ)
        if _ismsg(typ)
            j_typ = symbol(_pkg_msg_strs(typ)[2])
            j_def = Expr(:call, j_typ)
        else
            if ! haskey(_msg_builtin_types, typ)
                error("Message generation; unknown type '$typ' in:\n$members")
            end
            j_typ = _msg_builtin_types[typ]
            j_def = Expr(:call, :msgzero, j_typ)
        end

        namesym = symbol(n)
        if arraylen >= 0
            memexpr = :($namesym::Array{$j_typ,1})
            defexpr = Expr(:call, :fill, j_def, arraylen)
            convexpr = :(jmsg.$namesym = convert(Array{$j_typ,1}, o[$n]))
        else
            memexpr = :($namesym::$j_typ)
            defexpr = j_def
            convexpr = :(jmsg.$namesym = convert($j_typ, o[$n]))
        end
        push!(typeargs, memexpr)
        push!(consargs, defexpr)
        insert!(convargs, length(convargs), convexpr)
    end
    [typedecl, construct, pyconvert]
end 

#Produce an order of the keys of d that respect their dependencies
function _order(d::Dict{String, Set{String}})
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
    tlist = String[]
    for t in keys(d)
        trecurse!(tlist, d, t)
    end
    tlist
end

function _pkg_msg_strs(msgtype::String)
    if ! _ismsg(msgtype)
        error("Incorrect message type '$msgtype', use 'package_name/message'")
    end
    split(msgtype, '/')
end
#Valid message string is all word chars split by a single forward slash, with
#optional square brackets for array types
_ismsg(s::String) = ismatch(r"^\w+/\w+(?:\[\d*\])?$", s)

#Sanitize a string by checking for and removing brackets if they are present
#Return the sanitized type and the number inside the brackets if it is a fixed
#size type. Returns -1 if variable size (no number)
function _check_array_type(typ::String)
    arraylen = -1
    arrtest = r"^([\w/]+)\[(\d*)\]$"
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
    btype, arraylen
end
