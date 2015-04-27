#Generate Julia composite types for ROS messages
using Compat

abstract ROSModule
type ROSMsgModule <: ROSModule
    name::ASCIIString
    members::Vector{ASCIIString}
    pyobjs::Dict{ASCIIString,PyObject}
    deps::Set{ASCIIString}
    function ROSMsgModule(mname::String)
        new(mname, 
            ASCIIString[], 
            Dict{ASCIIString,PyObject}(), 
            Set{ASCIIString}()
        )
    end
end
type ROSSrvModule <: ROSModule
    name::ASCIIString
    members::Vector{ASCIIString}
    pyobjs::Dict{ASCIIString,Vector{PyObject}}
    deps::Set{ASCIIString}
    function ROSSrvModule(mname::String)
        new(mname, 
            ASCIIString[], 
            Dict{ASCIIString,Vector{PyObject}}(), 
            Set{ASCIIString}()
        )
    end
end
type ROSPackage
    name::ASCIIString
    msg::ROSMsgModule
    srv::ROSSrvModule
    function ROSPackage(pkgname::String)
        new(pkgname, ROSMsgModule(pkgname), ROSSrvModule(pkgname))
    end
end

const _rospy_imports = Dict{ASCIIString,ROSPackage}()
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

#Abstract supertypes of all generated types
abstract MsgT
abstract SrvT

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
        pkg, ismsg, typ = _pkgtype_import(input.args[1])
        push!(types, typ)
        for t in input.args[2:end]
            @assert isa(t, Symbol) "Type name ($(string(t))) not a symbol"
            push!(types, string(t))
        end
        return :(_usepkg($pkg, $ismsg, $types...))
    else
        pkg, ismsg, typ = _pkgtype_import(input)
        return :(_usepkg($pkg, $ismsg, $typ))
    end
end

#Return the pkg and types strings for a single expression of form:
#  pkg.[msg|srv].type or pkg.[msg|srv]:type
function _pkgtype_import(input::Expr)
    @assert input.head in (:(.), :(:)) "Improper @rosimport input"
    @assert isa(input.args[1], Expr) "Improper @rosimport input"
    @assert input.args[1].head == :(.) "Improper @rosimport input"
    p = input.args[1].args[1]
    @assert isa(p, Symbol) "Package name ($(string(p))) not a symbol"
    m_or_s = input.args[1].args[2].args[1]
    @assert m_or_s in (:msg,:srv) "Improper @rosimport input"
    ps = string(p)
    msb = m_or_s == :msg
    ts = ""
    if isa(input.args[2], Symbol)
        ts = string(input.args[2])
    elseif isa(input.args[2], Expr)
        tsym = input.args[2].args[1]
        @assert isa(tsym, Symbol) "Type name ($(string(tsym))) not a symbol"
        ts = string(tsym)
    end
    return ps,msb,ts
end
#Import a set of types from a single package
function _usepkg(package::String, ismsg::Bool, names::String...)
    if ! haskey(_rospy_imports, package)
        @debug("Creating new package: ",package,".", ismsg ? "msg" : "srv") 
        _rospy_imports[package] = ROSPackage(package)
    end
    rospypkg = _rospy_imports[package]
    for n in names
        addtype!(ismsg ? rospypkg.msg : rospypkg.srv, n)
    end
end

#Do the Julia type generation. This function is needed because we want to
#create the modules in one go, rather than anytime @rosimport gets called
function gentypes()
    pkgdeps = _collectdeps(_rospy_imports)
    pkglist = _order(pkgdeps)
    for pkg in pkglist
        buildpackage(_rospy_imports[pkg])
    end
end

#Reset type generation process to start over with @rosimport. Does not remove
#already generated modules! They will be replaced when gentypes is called
#again.
function cleartypes()
    empty!(_rospy_imports)
    nothing
end

function addtype!(mod::ROSSrvModule, typ::String)
    if ! (typ in mod.members)
        fullmodname = string(mod.name, ".srv")
        @debug("Service type import: ", fullmodname, ".", typ)
        pymod, pyobj = _pyvars(fullmodname, typ)

        if ! haskey(pyobj, "_request_class")
            error(string("Incorrect service name: ", typ))
        end

        #Immediately import dependencies from the Request/Response classes
        #Repeats are OK
        req_obj = pymod.pymember(string(typ,"Request"))
        resp_obj = pymod.pymember(string(typ,"Response"))
        deptypes = [req_obj[:_slot_types]; resp_obj[:_slot_types]]
        _importdeps!(mod, deptypes)

        push!(mod.members, typ)
        mod.pyobjs[typ] = PyObject[pyobj, req_obj, resp_obj]
    end
end

function addtype!(mod::ROSMsgModule, typ::String)
    if ! (typ in mod.members)
        fullmodname = string(mod.name, ".msg")
        @debug("Message type import: ", fullmodname, ".", typ)
        pymod, pyobj = _pyvars(fullmodname, typ)

        deptypes = pyobj[:_slot_types]
        _importdeps!(mod, deptypes)

        push!(mod.members, typ)
        mod.pyobjs[typ] = pyobj
    end
end

function _pyvars(modname::String, typ::String)
    pymod = _import_rospy_pkg(modname)
    pyobj =
        try pymod.pymember(typ)
        catch ex
            if isa(ex, KeyError)
                error("$typ not found in package: $modname")
            else
                rethrow(ex)
            end
        end
    pymod, pyobj
end

function _importdeps!(mod::ROSModule, deps::Vector)
    for d in deps
        if ! (d in keys(_ros_builtin_types))
            @debug("Dependency: ", d)
            pkgname, typename = _splittypestr(d)

            @debug_addindent
            #Create a new ROSPackage if needed
            if ! haskey(_rospy_imports, pkgname)
                @debug("Creating new package: ", pkgname)
                _rospy_imports[pkgname] = ROSPackage(pkgname)
            end

            #Dependencies will always be messages only
            depmod = _rospy_imports[pkgname].msg
            if depmod.name != mod.name
                #pushing to a set does not create duplicates
                push!(mod.deps, depmod.name)
            end

            #Don't want to include the brackets in the string if present
            typeclean = _check_array_type(typename)[1]
            addtype!(depmod, typeclean)
            @debug_subindent
        end
    end
end

function _import_rospy_pkg(package::String)
    pkg, ptype = split(package, '.')
    if ptype != "msg" && ptype != "srv" 
        throw(ArgumentError("Improper import call for package: $package"))
    end
    pypkg = symbol(string("py_",pkg,"_",ptype))
    newimport = try
        eval(pypkg)
        false
    catch
        true
    end
    if newimport
        @debug("Importing python package: ", package)
        pkgsym, mssym = symbol(pkg), symbol(ptype)
        try
            @eval @pyimport $(pkgsym).$(mssym) as $pypkg
        catch ex
            show(ex)
            error("python import error: $(ex.val[:args][1])")
        end
    end
    eval(pypkg)
end

function buildpackage(pkg::ROSPackage)
    @debug("Building package: ", pkg.name)
end

function generate(pkg::ROSPackage)
    msgcode = modulecode(pkg.msg)
    srvcode = modulecode(pkg.srv)

    pkgcode = quote
        module $(pkg.name)
        end
    end
    eval(Main, pkgcode)
    pkgmod = eval(Main, symbol(pkg.name))
    msgmod = quote
        module msg
            $(msgcode...)
        end
    end
    srvmod = quote
        module srv
            $(srvcode...)
        end
    end
    eval(pkgmod, msgmod)
    eval(pkgmod, srvmod)
end

function modulecode(mod::ROSModule)
    #Module interior code expressions
    modexprs = Expr[]

    #Import/exports so the generated code can use external names
    push!(modexprs,
        quote
            import Base.convert
            import PyCall
            import RobotOS
            import RobotOS.MsgT
            import RobotOS.Time
            import RobotOS.Duration
            import RobotOS.typezero
            import RobotOS._typerepr
        end
    )
    #push!(modexprs, Expr(:import, :RobotOS, pymod))
    #Dependencies are always msg modules only
    for m in mod.deps
        msym = m.ismsg ? (:msg) : (:srv)
        push!(modexprs, Expr(:using, symbol(m.mod), msym))
    end

    types = keys(mod.objs)
    exports = Expr(:export)
    for typ in types
        push!(exports.args, symbol(typ))
    end
    push!(modexprs, exports)

    #Type creation
    for typ in types
        memnames = mod.objs[typ]["__slots__"]
        memtypes = mod.objs[typ]["_slot_types"]
        members = [zip(memnames, memtypes)...]
        typeexprs = buildtype(mod.name, typ, members)

        for ex in typeexprs
            push!(modexprs, ex)
        end
    end
    modexprs
end

#Create the new module and build the new types inside
function buildmodule(modname::String, deps::Set, types::Vector)
    #Module interior code expressions
    modexprs = Expr[]

    #Import/exports so the generated code can use external names
    pkg, m_or_s = split(modname, '.')
    pymod = symbol(string("py_",modname,"_",m_or_s))
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
    #Dependencies are always msg modules only
    for m in deps
        push!(modexprs, Expr(:using, symbol(m), :msg))
    end
    exports = Expr(:export)
    for typ in types
        push!(exports.args, symbol(_splittypestr(typ)[3]))
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
    m_or_s_sym = symbol(m_or_s)
    msgmod = :(module $m_or_s_sym end)
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
#  - convert to/from PyObject
#  - default constructor
function buildtype(modinfo, typename::String, members::Vector)
    pkg = modinfo.mod
    m_or_s = modinfo.ismsg ? (:msg) : (:srv)
    parentsym = modinfo.ismsg ? (:MsgT) : (:SrvT)
    nsym = symbol(typename)
    @debug("Type: ", typename)

    exprs = Expr[]
    #Type declaration
    push!(exprs, :(
        type $nsym <: $parentsym
            #Generated code here
        end
    ))
    #Convert from PyObject
    push!(exprs, :(
        function convert(jlt::Type{$nsym}, o::PyObject)
            if o[:_type] != _typerepr(jlt)
                throw(InexactError())
            end
            jl = $nsym()
            #Generated code here
            jl
        end
    ))
    #Convert to PyObject
    push!(exprs, :(
        function convert(::Type{PyObject}, o::$nsym)
            py = (RobotOS._rospy_imports[$pkg].$m_or_s).objs[$typename]()
            #Generated code here
            py
        end
    ))
    #Default constructor, but only if the type has members
    defconstr = :(
        function $nsym()
            $nsym() #Generated code inside parens here
        end
    )
    if length(members) > 0
        push!(exprs, defconstr)
    end

    typeargs  =  exprs[1].args[3].args
    jlconargs =  exprs[2].args[2].args
    pyconargs =  exprs[3].args[2].args
    consargs  = defconstr.args[2].args[2].args

    #Now add the meat to the empty expressions above
    for (namestr,typ) in members
        @debug("\t$namestr :: $typ")
        if typ == "char" || typ == "byte"
            warn("Use of type '$typ' is deprecated in message definitions, ",
            "use '$(lowercase(string(_ros_builtin_types[typ])))' instead.")
        end

        typ, arraylen = _check_array_type(typ)
        if _isrostype(typ)
            j_typ = symbol(_splittypestr(typ)[2])
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
        insert!(jlconargs, length(jlconargs), jlconexpr)
        insert!(pyconargs, length(pyconargs), pyconexpr)
        push!(consargs, defexpr)
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
        tpkg = _splittypestr(typ)[1]
        if ! haskey(pkg_deps, tpkg)
            pkg_deps[tpkg] = Set{ASCIIString}()
        end
        for d in deps
            dpkg = _splittypestr(d)[1]
            if dpkg != tpkg
                push!(pkg_deps[tpkg], dpkg)
            end
        end
    end
    pkg_deps
end

function _splittypestr(typestr::String)
    if ! _isrostype(typestr)
        error(string("Invalid message type '$typestr', ",
                     "use 'package_name/type_name'"))
    end
    rospkg, typ = split(typestr, '/')
    rospkg, typ
end
#Valid ROS type string is all word chars split by a single forward slash, with
#optional square brackets for array types
_isrostype(s::String) = ismatch(r"^\w+/\w+(?:\[\d*\])?$", s)

#Sanitize a string by checking for and removing brackets if they are present
#Return the sanitized type and the number inside the brackets if it is a fixed
#size type. Returns 0 if variable size (no number), -1 if no brackets
function _check_array_type(typ::String)
    arraylen = -1
    m = match(r"^([\w/]+)\[(\d*)\]$", typ)
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
