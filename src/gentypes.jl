#Generate Julia composite types for ROS messages
using Compat

_rostypes = Dict{ASCIIString, Vector{ASCIIString}}()
_rospy_classes = Dict{ASCIIString, PyObject}()
_jltype_strs = Dict{DataType, ASCIIString}()
_ros_builtin_types = @compat Dict{ASCIIString, Symbol}(
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
    "duration"=> :Duration,
    )
typezero{T<:Real}(::Type{T}) = zero(T)
typezero(::Type{ASCIIString}) = ""
typezero(::Type{Time}) = Time(0,0)
typezero(::Type{Duration}) = Duration(0,0)

abstract MsgT

function usetypes(types::Dict)
    for (pkg,names) in types
        for n in names
            _addtype(pkg,n)
        end
    end
end
function usetypes(names::String...)
    for n in names
        if _isrostype(n)
            _addtype(_pkg_name_strs(n)...)
        else
            error("Invalid ros type: $n")
        end
    end
end
function usepkg(pkg::String, names::String...)
    for n in names
        _addtype(pkg, n)
    end
end
function _addtype(pkg::String, name::String)
    if ! haskey(_rostypes, pkg)
        _rostypes[pkg] = Vector{ASCIIString}[]
    end
    if ! (name in _rostypes[pkg])
        push!(_rostypes[pkg], name)
    end
end

function gentypes()
    pkg_deps = Dict{ASCIIString, Set{ASCIIString}}()
    typ_deps = Dict{ASCIIString, Set{ASCIIString}}()
    for (pkg, names) in _rostypes
        for n in names
            typestr = pkg * "/" * n
            importtype(typestr, pkg_deps, typ_deps)
        end
    end
    pkglist = _order(pkg_deps)
    for pkg in pkglist
        pkgtypes = filter(Regex("^$pkg/\\w+\$"), typ_deps)
        mtypelist::Vector{ASCIIString} = _order(pkgtypes)
        buildmodule(pkg, pkg_deps[pkg], mtypelist)
    end
end

#Recursively import all needed messages for a given message
function importtype(typestr::String, pkg_deps::Dict, typ_deps::Dict)
    if ! haskey(_rospy_classes,typestr)
        pkg, name = _pkg_name_strs(typestr)
        pkgi = symbol(string("py_",pkg))
        pkgsym = symbol(pkg)

        #Import python ROS module, no effect if already there
        try
            @eval @pyimport $pkgsym.msg as $pkgi
        catch ex
            error("python import error: $(ex.val[:args][1])")
        end
        if ! haskey(pkg_deps, pkg)
            pkg_deps[pkg] = Set{ASCIIString}()
        end
        #Store a reference to the message class definition
        _rospy_classes[typestr] = try
            @eval $pkgi.pymember($name)
        catch KeyError
            error("$name not found in package: $pkg")
        end
        typ_deps[typestr] = Set{ASCIIString}()
        subtypes = _rospy_classes[typestr][:_slot_types]
        for t in subtypes
            if _isrostype(t)
                t = _check_array_type(t)[1]
                tpkg = _pkg_name_strs(t)[1]
                if tpkg != pkg #Possibly a new module dependency
                    push!(pkg_deps[pkg], tpkg)
                end
                #pushing to a set does not create duplicates
                push!(typ_deps[typestr], t)

                importtype(t, pkg_deps, typ_deps)
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
    modexprs = Expr[]

    #Import/exports so the generated code can use external names
    pymod = symbol(string("py_",modname))
    push!(modexprs, 
        quote
            import Base.convert
            using PyCall
            import RobotOS.pymod
            using  RobotOS.MsgT
            using  RobotOS.Time
            using  RobotOS.Duration
            using  RobotOS.typezero
        end
    )
    for m in deps
        push!(modexprs, Expr(:using, symbol(m)))
    end
    exports = Expr(:export)
    for typ in types
        push!(exports.args, symbol(_pkg_name_strs(typ)[2]))
    end
    push!(modexprs, exports)

    #Type creation
    for typ in types
        pkg, msg = _pkg_name_strs(typ)
        memnames = _rospy_classes[typ]["__slots__"]
        memtypes = _rospy_classes[typ]["_slot_types"]
        members = [zip(memnames, memtypes)...]
        typeexprs = buildtype(typ, members)

        for ex in typeexprs 
            push!(modexprs, ex)
        end
        @eval _jltype_strs[$mod.$(symbol(msg))] = $typ
    end
    nothing
end

#Generate code for a native Julia message type
#  - type/member declarations
#  - default constructor
#  - convert to/from PyObject
function buildtype(typ::String, members::Vector)
    pkg, name = _pkg_name_strs(typ)
    pymod = symbol(string("py_",pkg))
    nsym = symbol(name)
    println("Type: $name")

    #Empty expressions
    exprs = Array(Expr, 4)
    #Type declaration
    exprs[1] = :(
        type $nsym <: MsgT
        end
    )
    #Default constructor
    exprs[2] = :(
        $nsym() = $nsym()
    )
    #Convert from PyObject
    exprs[3] = :(
        convert(::Type{$nsym}, o::PyObject) = begin
            jl = $nsym()
            jl
        end
    )
    #Convert to PyObject
    exprs[4] = :(
        convert(::Type{PyObject}, o::$nsym) = begin
            py = ($pymod.$nsym)()
            py
        end
    )
    typeargs  = exprs[1].args[3].args
    consargs  = exprs[2].args[2].args
    jlconargs = exprs[3].args[2].args
    pyconargs = exprs[4].args[2].args

    #Now process the type members
    for (namestr,typ) in members
        println("\t$namestr :: $typ")

        typ, arraylen = _check_array_type(typ)
        if _isrostype(typ)
            j_typ = symbol(_pkg_name_strs(typ)[2])
            j_def = Expr(:call, j_typ)
        else
            if ! haskey(_ros_builtin_types, typ)
                error("Message generation; unknown type '$typ' in:\n$members")
            end
            j_typ = _ros_builtin_types[typ]
            j_def = Expr(:call, :typezero, j_typ)
        end

        namesym = symbol(namestr)
        if arraylen >= 0
            memexpr = :($namesym::Array{$j_typ,1})
            defexpr = Expr(:call, :fill, j_def, arraylen)
            jlconexpr = :(jl.$namesym = convert(Array{$j_typ,1}, o[$namestr]))

            #uint8[] is string in rospy and PyCall's conversion to bytearray is
            #rejected by ROS
            if j_typ == :Uint8
                pyconexpr = :(py[$namestr] =
                    pycall(pybuiltin("str"), PyObject, PyObject(o.$namesym))
                )
            elseif _isrostype(typ)
                pyconexpr = :(py[$namestr] =
                    convert(Array{PyObject,1}, o.$namesym))
            else
                pyconexpr = :(py[$namestr] = o.$namesym)
            end
        else
            memexpr = :($namesym::$j_typ)
            defexpr = j_def
            jlconexpr = :(jl.$namesym = convert($j_typ, o[$namestr]))
            pyconexpr = :(py[$namestr] = convert(PyObject, o.$namesym))
        end
        push!(typeargs, memexpr)
        push!(consargs, defexpr)
        insert!(jlconargs, length(jlconargs), jlconexpr)
        insert!(pyconargs, length(pyconargs), pyconexpr)
    end
    return exprs
end 

#Produce an order of the keys of d that respect their dependencies
function _order(d::Dict)
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

function _pkg_name_strs(typestr::String)
    if ! _isrostype(typestr)
        error("Invalid message type '$typestr', use 'package_name/type_name'")
    end
    split(typestr, '/')
end
#Valid ROS type string is all word chars split by a single forward slash, with
#optional square brackets for array types
_isrostype(s::String) = ismatch(r"^\w+/\w+(?:\[\d*\])?$", s)

#Sanitize a string by checking for and removing brackets if they are present
#Return the sanitized type and the number inside the brackets if it is a fixed
#size type. Returns 0 if variable size (no number), -1 if no brackets
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
