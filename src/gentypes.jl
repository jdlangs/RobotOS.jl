#Generate Julia composite types for ROS messages
using Compat

const _rospy_classes = Dict{ASCIIString, PyObject}()
const _ros_typ_deps = Dict{ASCIIString, Set{ASCIIString}}()
const _ros_builtin_types = @compat Dict{ASCIIString, Symbol}(
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
    #Deprecated but supported
    "char"    => :Uint8,
    "byte"    => :Int8,
    )
typezero{T<:Real}(::Type{T}) = zero(T)
typezero(::Type{ASCIIString}) = ""
typezero(::Type{Time}) = Time(0,0)
typezero(::Type{Duration}) = Duration(0,0)

abstract MsgT

#Rearranges the expression into a RobotOS._usepkg call. Input comes in as a
#single package qualified expression, or as a tuple expression where the first
#element is the same as the single expression case. Most of the code is just
#error checking that the input takes that form.
macro rosimport(input)
    @assert input.head in [:tuple, :(.), :(:)] "Improper @rosimport input"
    if input.head == :tuple
        @assert isa(input.args[1], Expr) "Improper @rosimport input"
        @assert input.args[1].head == :(:) "First argument needs ':' following"
        types = ASCIIString[]
        pkg, typ = _pkgtype_import(input.args[1])
        push!(types, typ)
        for t in input.args[2:end]
            @assert isa(t, Symbol) "Type name ($(string(t))) not a symbol"
            push!(types, string(t))
        end
        return :(_usepkg($pkg, $types...))
    else
        pkg, typ = _pkgtype_import(input)
        return :(_usepkg($pkg, $typ))
    end
end

#Return the pkg and types strings for a single expression of form:
#  pkg.msg.type or pkg.msg:type
function _pkgtype_import(input::Expr)
    @assert input.head in [:(.), :(:)]
    @assert isa(input.args[1], Expr) "Improper @rosimport input"
    @assert input.args[1].head == :(.) "Improper @rosimport input"
    @assert input.args[1].args[2].args[1] == :msg "Improper @rosimport input"
    p = input.args[1].args[1]
    @assert isa(p, Symbol) "Package name ($(string(p))) not a symbol"
    ps = string(p)
    ts = ""
    if isa(input.args[2], Symbol)
        ts = string(input.args[2])
    elseif isa(input.args[2], Expr)
        tsym = input.args[2].args[1]
        @assert isa(tsym, Symbol) "Type name ($(string(tsym))) not a symbol"
        ts = string(tsym)
    end
    return ps,ts
end
#Import a set of types from a single package
function _usepkg(pkg::String, names::String...)
    for n in names
        typestr = pkg * "/" * n
        importtype(typestr, _ros_typ_deps)
    end
end

#Do the Julia type generation. This function is needed because we want to
#create the modules in one go, rather than anytime @rosimport gets called
function gentypes()
    pkg_deps = _typ_to_pkg_deps(_ros_typ_deps)
    pkglist = _order(pkg_deps)
    for pkg in pkglist
        pkgtypes = filter(Regex("^$pkg/\\w+\$"), _ros_typ_deps)
        mtypelist::Vector{ASCIIString} = _order(pkgtypes)
        buildmodule(pkg, pkg_deps[pkg], mtypelist)
    end
end

#Reset type generation process to start over with @rosimport. Does not remove
#already generated modules! They will be replaced when gentypes is called
#again.
function cleartypes()
    empty!(_ros_typ_deps)
    empty!(_rospy_classes)
    nothing
end

#Recursively import all needed messages for a given message
function importtype(typestr::String, typ_deps::Dict)
    if ! haskey(_rospy_classes,typestr)
        if _verbose
            println("Importing: ", typestr)
        end
        pkg, name = _pkg_name_strs(typestr)
        pkgi = symbol(string("py_",pkg))
        pkgsym = symbol(pkg)

        #Import python ROS module, no effect if already there
        try
            @eval @pyimport $pkgsym.msg as $pkgi
        catch ex
            error("python import error: $(ex.val[:args][1])")
        end
        #Store a reference to the message class definition
        _rospy_classes[typestr] = try
            @eval $pkgi.pymember($name)
        catch ex
            if isa(ex, KeyError)
                error("$name not found in package: $pkg")
            else
                rethrow(ex)
            end
        end
        typ_deps[typestr] = Set{ASCIIString}()
        subtypes = _rospy_classes[typestr][:_slot_types]
        for t in subtypes
            if _isrostype(t)
                #Don't want to include the brackets in the string if present
                t = _check_array_type(t)[1]
                #pushing to a set does not create duplicates
                push!(typ_deps[typestr], t)

                importtype(t, typ_deps)
            end
        end
    end
end

#Create the new module and build the new types inside
function buildmodule(modname::String, deps::Set, types::Vector)
    #Module interior code expressions
    modexprs = Expr[]

    #Import/exports so the generated code can use external names
    pymod = symbol(string("py_",modname))
    push!(modexprs, 
        quote
            import Base.convert
            using  PyCall
            import RobotOS
            using  RobotOS.MsgT
            using  RobotOS.Time
            using  RobotOS.Duration
            using  RobotOS.typezero
            import RobotOS._typerepr
        end
    )
    push!(modexprs, Expr(:import, :RobotOS, pymod))
    for m in deps
        push!(modexprs, Expr(:using, symbol(m), :msg))
    end
    exports = Expr(:export)
    for typ in types
        push!(exports.args, symbol(_pkg_name_strs(typ)[2]))
    end
    push!(modexprs, exports)

    #Type creation
    for typ in types
        memnames = _rospy_classes[typ]["__slots__"]
        memtypes = _rospy_classes[typ]["_slot_types"]
        members = [zip(memnames, memtypes)...]
        typeexprs = buildtype(typ, members)

        for ex in typeexprs 
            push!(modexprs, ex)
        end
    end
    #Create module
    msgmod = :(module msg end)
    append!(msgmod.args[3].args, modexprs)

    modsym = symbol(modname)
    modexp = Expr(:toplevel, :(module ($modsym) end))
    push!(modexp.args[1].args[3].args, msgmod)

    eval(Main, modexp)
    mod = eval(Main, modsym)
    mod
end

#Generate code for a native Julia message type
#  - type/member declarations
#  - default constructor
#  - convert to/from PyObject
function buildtype(typename::String, members::Vector)
    pkg, name = _pkg_name_strs(typename)
    pymod = symbol(string("py_",pkg))
    nsym = symbol(name)
    if _verbose
        println("Type: $name")
    end

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
        convert(jlt::Type{$nsym}, o::PyObject) = begin
            if o[:_type] != _typerepr(jlt)
                throw(InexactError())
            end
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

    #Now add the meat to the empty expressions above
    for (namestr,typ) in members
        if _verbose
            println("\t$namestr :: $typ")
        end
        if typ == "char" || typ == "byte"
            warn("Use of type '$typ' is deprecated in message definitions, ",
            "use '$(lowercase(string(_ros_builtin_types[typ])))' instead.")
        end

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
    push!(exprs, :(_typerepr(::Type{$nsym}) = $typename))
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

#Produce a dict of package dependencies from the dependencies of the
#fully-qualified type strings
function _typ_to_pkg_deps(typ_deps::Dict)
    pkg_deps = Dict{ASCIIString, Set{ASCIIString}}()
    for (typ, deps) in collect(typ_deps)
        tpkg = _pkg_name_strs(typ)[1]
        if ! haskey(pkg_deps, tpkg)
            pkg_deps[tpkg] = Set{ASCIIString}()
        end
        for d in deps
            dpkg = _pkg_name_strs(d)[1]
            if dpkg != tpkg
                push!(pkg_deps[tpkg], dpkg)
            end
        end
    end
    pkg_deps
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

#Default method to get the "pkg/type" string from a generated DataType.
#Extended by the generated modules.
_typerepr{T}(::Type{T}) = error("Not a ROS type")
