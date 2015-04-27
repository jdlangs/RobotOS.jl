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

function addtype!(mod::ROSMsgModule, typ::String)
    if ! (typ in mod.members)
        @debug("Message type import: ", _modname(mod), ".", typ)
        pymod, pyobj = _pyvars(_modname(mod), typ)

        deptypes = pyobj[:_slot_types]
        _importdeps!(mod, deptypes)

        push!(mod.members, typ)
        mod.pyobjs[typ] = pyobj
    end
end

function addtype!(mod::ROSSrvModule, typ::String)
    if ! (typ in mod.members)
        @debug("Service type import: ", _modname(mod), ".", typ)
        pymod, pyobj = _pyvars(_modname(mod), typ)

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

    #Create the top-level module for the package in Main
    pkgsym = symbol(pkg.name)
    pkgcode = Expr(:toplevel, :(module $pkgsym end))
    Main.eval(pkgcode)
    pkgmod = Main.eval(pkgsym)

    #Add msg and srv submodules if needed
    @debug_addindent
    if length(pkg.msg.members) > 0
        msgcode = modulecode(pkg.msg)
        msgmod = :(module msg end)
        for expr in msgcode
            println(expr)
            push!(msgmod.args[3].args, expr)
        end
        eval(pkgmod, msgmod)
    end
    if length(pkg.srv.members) > 0
        srvcode = modulecode(pkg.srv)
        srvmod = :(module srv end)
        for expr in srvcode
            push!(srvmod.args[3].args, expr)
        end
        eval(pkgmod, srvmod)
    end
    @debug_subindent
end

function modulecode(mod::ROSModule)
    @debug("submodule: ", _modname(mod))
    modcode = Expr[]

    #Generic imports
    push!(modcode,
        quote
            using PyCall
            import Base.convert
            import RobotOS
            import RobotOS.MsgT
            import RobotOS.Time
            import RobotOS.Duration
            import RobotOS.typezero
            import RobotOS._typerepr
        end
    )
    #Import the dependant message modules
    for m in mod.deps
        push!(modcode, Expr(:using, symbol(m), :msg))
    end
    exprs

    #The exported names
    push!(modcode, _exportexpr(mod))

    #The actual type definition
    @debug_addindent
    for typ in mod.members
        typecode = buildtype(mod, typ)
        append!(modcode, typecode)
    end
    @debug_subindent
    modcode
end

function _exportexpr(mod::ROSMsgModule)
    exports = [symbol(m) for m in mod.members]
    Expr(:export, exports...)
end

function _exportexpr(mod::ROSSrvModule)
    exportexpr = Expr(:export)
    for typ in mod.members
        push!(exports.args,
            symbol(typ),
            symbol(string(typ,"Request")),
            symbol(string(typ,"Response"))
        )
    end
    exportexpr
end

function buildtype(mod::ROSMsgModule, typename::String)
    pyobj = mod.pyobjs[typename]
    memnames = pyobj[:__slots__]
    memtypes = pyobj[:_slot_types]
    members = collect(zip(memnames, memtypes))

    pyexpr = :(RobotOS._rospy_imports[$(mod.name)].msg.pyobjs[$typename])
    typecode(string(mod.name,"/",typename), :MsgT, pyexpr, members)
end

function buildtype(mod::ROSSrvModule, typename::String)
    baseexpr = :(type $typename <: SrvT end)

    reqobj = mod.pyobjs[typename][2]
    memnames = reqobj[:__slots__]
    memtypes = reqobj[:_slot_types]
    reqmems = collect(zip(memnames, memtypes))
    pyreq  = :(RobotOS._rospy_imports[$(mod.name)].srv.pyobjs[$typename][2])
    reqexprs  = typecode(
        string(mod.name,"/",typename,"Request"), :SrvT, pyreq, reqmems)

    respobj = mod.pyobjs[typename][3]
    memnames = respobj[:__slots__]
    memtypes = respobj[:_slot_types]
    respmems = collect(zip(memnames, memtypes))
    pyresp = :(RobotOS._rospy_imports[$(mod.name)].srv.pyobjs[$typename][3])
    respexprs = typecode(
        string(mod.name,"/",typename,"Response"), :SrvT, pyresp, respmems)

    [baseexpr; reqexprs; respexprs]
end

#Create the core generated expressions for a native Julia message type that has
#data fields and interchanges with a python counterpart: 
# (1) the 'type ... end' block
# (2) No param outer constructer
# (3) convert(PyObject, ...)
# (4) convert(..., o::PyObject)
function typecode(rosname::String, super::Symbol, pyexpr::Expr, members::Vector)
    pkg, tname = _splittypestr(rosname)
    @debug("Type: ", tname)
    tsym = symbol(tname)

    exprs = Expr[]
    #First the empty expressions
    #(1) Type declaration
    push!(exprs, :(
        type $tsym <: $super
            #Generated code here
        end
    ))
    #(2) Default constructor, but only if the type has members
    if length(members) > 0
        push!(exprs, :(
            function $tsym()
                $tsym() #Generated code inside parens here
            end
        ))
    else
        push!(exprs, :())
    end
    #(3) Convert to PyObject
    push!(exprs, :(
        function convert(::Type{PyObject}, o::$tsym)
            py = pycall($pyexpr, PyObject)
            #Generated code here
            py
        end
    ))
    #(4) Convert from PyObject
    push!(exprs, :(
        function convert(jlt::Type{$tsym}, o::PyObject)
            if o[:_type] != _typerepr(jlt)
                throw(InexactError())
            end
            jl = $tsym()
            #Generated code here
            jl
        end
    ))

    #Now add the meat to the empty expressions above
    for (namestr,typ) in members
        @debug_addindent
        _addtypemember!(exprs, namestr, typ)
        @debug_subindent
    end
    push!(exprs, :(_typerepr(::Type{$tsym}) = $rosname))
    exprs
end

function _addtypemember!(exprs, namestr, typestr)
    @debug("$namestr :: $typestr")
    typeargs  = exprs[1].args[3].args
    pyconargs = exprs[3].args[2].args
    jlconargs = exprs[4].args[2].args
    consargs  = length(exprs[2].args) >= 2 ? 
        exprs[2].args[2].args[2].args : 
        nothing

    if typestr == "char" || typestr == "byte"
        warn("Use of type '$typestr' is deprecated in message definitions, ",
        "use '$(lowercase(string(_ros_builtin_types[typestr])))' instead.")
    end

    typestr, arraylen = _check_array_type(typestr)
    if _isrostype(typestr)
        j_typ = symbol(_splittypestr(typestr)[2])
        j_def = Expr(:call, j_typ)
    else
        if ! haskey(_ros_builtin_types, typestr)
            error("Message generation; unknown type '$typestr'")
        end
        j_typ = _ros_builtin_types[typestr]
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

function _collectdeps(pkgs::Dict)
    deps = Dict{ASCIIString, Set{ASCIIString}}()
    for pname in keys(pkgs)
        if ! haskey(deps, pname)
            deps[pname] = Set{ASCIIString}()
        end
        union!(deps[pname], pkgs[pname].msg.deps)
        union!(deps[pname], pkgs[pname].srv.deps)
    end
    deps
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

_modname(m::ROSMsgModule) = string(m.name, ".msg")
_modname(m::ROSSrvModule) = string(m.name, ".srv")
