#Generate Julia composite types for ROS messages
using Compat

export @rosimport, rostypegen, rostypereset, gentypes, cleartypes

#Type definitions
#Composite types for internal use. Keeps track of the imported types and helps
#keep code generation orderly.
abstract ROSModule
type ROSPackage
    name::ASCIIString
    msg::ROSModule
    srv::ROSModule
    function ROSPackage(pkgname::ASCIIString)
        pkg = new(pkgname)
        pkg.msg = ROSMsgModule(pkg)
        pkg.srv = ROSSrvModule(pkg)
        pkg
    end
end
type ROSMsgModule <: ROSModule
    pkg::ROSPackage
    members::Vector{ASCIIString}
    deps::Set{ASCIIString}
    ROSMsgModule(pkg) = new(pkg, ASCIIString[], Set{ASCIIString}())
end
type ROSSrvModule <: ROSModule
    pkg::ROSPackage
    members::Vector{ASCIIString}
    deps::Set{ASCIIString}
    ROSSrvModule(pkg) = new(pkg, ASCIIString[], Set{ASCIIString}())
end

#These two global objects maintain the hierarchy from multiple calls to
#`@rosimport` and keep the link to the Python objects whenever communication
#goes between RobotOS and rospy.
const _rospy_imports = Dict{ASCIIString,ROSPackage}()
const _rospy_objects = Dict{ASCIIString,PyObject}()
const _rospy_modules = Dict{ASCIIString,PyObject}()

const _ros_builtin_types = @compat Dict{ASCIIString, Symbol}(
    "bool"    => :Bool,
    "int8"    => :Int8,
    "int16"   => :Int16,
    "int32"   => :Int32,
    "int64"   => :Int64,
    "uint8"   => :UInt8,
    "uint16"  => :UInt16,
    "uint32"  => :UInt32,
    "uint64"  => :UInt64,
    "float32" => :Float32,
    "float64" => :Float64,
    "string"  => :ASCIIString,
    "time"    => :Time,
    "duration"=> :Duration,
    #Deprecated but supported
    "char"    => :UInt8,
    "byte"    => :Int8,
    )

#Abstract supertypes of all generated types
abstract MsgT
abstract SrvT
abstract ServiceDefinition

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
        @assert length(input.args[2].args) == 1 "Type name not a symbol"
        tsym = input.args[2].args[1]
        @assert isa(tsym, Symbol) "Type name ($(string(tsym))) not a symbol"
        ts = string(tsym)
    end
    return ps,msb,ts
end
#Import a set of types from a single package
function _usepkg(package::ASCIIString, ismsg::Bool, names::ASCIIString...)
    global _rospy_imports
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
function rostypegen()
    global _rospy_imports
    pkgdeps = _collectdeps(_rospy_imports)
    pkglist = _order(pkgdeps)
    for pkg in pkglist
        buildpackage(_rospy_imports[pkg])
    end
end

#Reset type generation process to start over with @rosimport. Does not remove
#already generated modules! They will be replaced when rostypegen is called
#again.
function rostypereset()
    global _rospy_imports
    global _rospy_objects
    empty!(_rospy_imports)
    empty!(_rospy_objects)
    nothing
end

#Populate the module with a new message type. Import and add dependencies first
#so they will appear first in the generated code.
function addtype!(mod::ROSMsgModule, typ::ASCIIString)
    global _rospy_objects
    if !(typ in mod.members)
        @debug("Message type import: ", _fullname(mod), ".", typ)
        pymod, pyobj = _pyvars(_fullname(mod), typ)

        deptypes = pyobj[:_slot_types]
        _importdeps!(mod, deptypes)

        push!(mod.members, typ)
        _rospy_objects[_rostypestr(mod, typ)] = pyobj
    end
end

#Populate the module with a new service type. Import and add dependencies
#first.
function addtype!(mod::ROSSrvModule, typ::ASCIIString)
    global _rospy_objects
    if !(typ in mod.members)
        @debug("Service type import: ", _fullname(mod), ".", typ)
        pymod, pyobj = _pyvars(_fullname(mod), typ)

        if ! haskey(pyobj, "_request_class")
            error(string("Incorrect service name: ", typ))
        end

        #Immediately import dependencies from the Request/Response classes
        #Repeats are OK
        req_obj = pymod[string(typ,"Request")]
        resp_obj = pymod[string(typ,"Response")]
        deptypes = [req_obj[:_slot_types]; resp_obj[:_slot_types]]
        _importdeps!(mod, deptypes)

        push!(mod.members, typ)
        fulltypestr = _rostypestr(mod, typ)
        _rospy_objects[fulltypestr] = pyobj
        _rospy_objects[string(fulltypestr,"Request")] = req_obj
        _rospy_objects[string(fulltypestr,"Response")] = resp_obj
    end
end

#Return the python module and python object for a particular type
function _pyvars(modname::ASCIIString, typ::ASCIIString)
    pymod = _import_rospy_pkg(modname)
    pyobj =
        try pymod[typ]
        catch ex
            isa(ex, KeyError) || rethrow(ex)
            throw(KeyError("'$typ' in package '$modname'"))
        end
    pymod, pyobj
end

#Continue the import process on a list of dependencies. Called by `addtype!`
#and calls `addtype!` to complete the dependency recursion.
function _importdeps!(mod::ROSModule, deps::Vector)
    global _rospy_imports
    for d in deps
        #We don't care about array types when doing dependency resolution
        dclean = _check_array_type(d)[1]
        if ! haskey(_ros_builtin_types, dclean)
            @debug("Dependency: ", d)
            pkgname, typename = _splittypestr(dclean)

            @debug_addindent
            #Create a new ROSPackage if needed
            if ! haskey(_rospy_imports, pkgname)
                @debug("Creating new package: ", pkgname)
                _rospy_imports[pkgname] = ROSPackage(pkgname)
            end

            #Dependencies will always be messages only
            depmod = _rospy_imports[pkgname].msg
            #pushing to a set does not create duplicates
            push!(mod.deps, _name(depmod))

            addtype!(depmod, typename)
            @debug_subindent
        end
    end
end

#Bring in the python modules as needed
function _import_rospy_pkg(package::ASCIIString)
    global _rospy_modules
    if ! haskey(_rospy_modules, package)
        @debug("Importing python package: ", package)
        try
            _rospy_modules[package] = pyimport(package)
        catch ex
            show(ex)
            error("python import error: $(ex.val[:args][1])")
        end
    end
    _rospy_modules[package]
end

#The function that creates and fills the generated top-level modules
function buildpackage(pkg::ROSPackage)
    @debug("Building package: ", _name(pkg))

    #Create the top-level module for the package in Main
    pkgsym = symbol(_name(pkg))
    pkgcode = Expr(:toplevel, :(module ($pkgsym) end))
    Main.eval(pkgcode)
    pkgmod = Main.eval(pkgsym)

    #Add msg and srv submodules if needed
    @debug_addindent
    if length(pkg.msg.members) > 0
        msgmod = :(module msg end)
        msgcode = modulecode(pkg.msg)
        for expr in msgcode
            push!(msgmod.args[3].args, expr)
        end
        eval(pkgmod, msgmod)
    end
    if length(pkg.srv.members) > 0
        srvmod = :(module srv end)
        srvcode = modulecode(pkg.srv)
        for expr in srvcode
            push!(srvmod.args[3].args, expr)
        end
        eval(pkgmod, srvmod)
    end
    @debug_subindent
end

#Generate all code for a .msg or .srv module
function modulecode(mod::ROSModule)
    @debug("submodule: ", _fullname(mod))
    modcode = Expr[]

    #Common imports
    if VERSION < v"0.4-"
        push!(modcode,
            quote
                using Compat
            end)
    end
    push!(modcode,
        quote
            using PyCall
            import Base.convert
            import RobotOS
            import RobotOS.Time
            import RobotOS.Duration
            import RobotOS._typedefault
            import RobotOS._typerepr
        end
    )
    #Import statement specific to the module
    append!(modcode, _importexprs(mod))
    #The exported names
    push!(modcode, _exportexpr(mod))

    #The generated type codes
    @debug_addindent
    for typ in mod.members
        typecode = buildtype(mod, typ)
        append!(modcode, typecode)
    end
    @debug_subindent
    modcode
end

#The imports specific to each module, including dependant packages
function _importexprs(mod::ROSMsgModule)
    imports = Expr[Expr(:import, :RobotOS, :MsgT)]
    othermods = filter(d -> d != _name(mod), mod.deps)
    append!(imports, [Expr(:using,symbol(m),:msg) for m in othermods])
    imports
end
function _importexprs(mod::ROSSrvModule)
    imports = Expr[
        Expr(:import, :RobotOS, :SrvT),
        Expr(:import, :RobotOS, :ServiceDefinition),
        Expr(:import, :RobotOS, :_srv_reqtype),
        Expr(:import, :RobotOS, :_srv_resptype),
    ]
    append!(imports, [Expr(:using,symbol(m),:msg) for m in mod.deps])
    imports
end

#The exported names for each module
function _exportexpr(mod::ROSMsgModule)
    exportexpr = Expr(:export)
    for m in mod.members
        push!(exportexpr.args, symbol(_jl_safe_name(m,"Msg")))
    end
    exportexpr
end
function _exportexpr(mod::ROSSrvModule)
    exportexpr = Expr(:export)
    for typ in mod.members
        push!(exportexpr.args,
            symbol(typ),
            symbol(string(typ,"Request")),
            symbol(string(typ,"Response"))
        )
    end
    exportexpr
end

#All the generated code for a generated message type
function buildtype(mod::ROSMsgModule, typename::ASCIIString)
    global _rospy_objects
    fulltypestr = _rostypestr(mod, typename)
    pyobj = _rospy_objects[fulltypestr]
    memnames = pyobj[:__slots__]
    memtypes = pyobj[:_slot_types]
    members = collect(zip(memnames, memtypes))

    typecode(fulltypestr, :MsgT, members)
end

#All the generated code for a generated service type
#Will create 3 different composite types.
function buildtype(mod::ROSSrvModule, typename::ASCIIString)
    global _rospy_objects

    req_typestr = _rostypestr(mod, string(typename,"Request"))
    reqobj = _rospy_objects[req_typestr]
    memnames = reqobj[:__slots__]
    memtypes = reqobj[:_slot_types]
    reqmems = collect(zip(memnames, memtypes))
    pyreq  = :(RobotOS._rospy_objects[$req_typestr])
    reqexprs  = typecode(req_typestr, :SrvT, reqmems)

    resp_typestr = _rostypestr(mod, string(typename,"Response"))
    respobj = _rospy_objects[resp_typestr]
    memnames = respobj[:__slots__]
    memtypes = respobj[:_slot_types]
    respmems = collect(zip(memnames, memtypes))
    pyresp = :(RobotOS._rospy_objects[$resp_typestr])
    respexprs = typecode(resp_typestr, :SrvT, respmems)

    defsym = symbol(typename)
    reqsym = symbol(string(typename,"Request"))
    respsym = symbol(string(typename,"Response"))
    srvexprs = Expr[
        :(immutable $defsym <: ServiceDefinition end),
        :(_typerepr(::Type{$defsym}) = $(_rostypestr(mod,typename))),
        :(_srv_reqtype(::Type{$defsym}) = $reqsym),
        :(_srv_resptype(::Type{$defsym}) = $respsym),
    ]
    [reqexprs; respexprs; srvexprs]
end

#Create the core generated expressions for a native Julia message type that has
#data fields and interchanges with a python counterpart:
# (1) the 'type ... end' block
# (2) Default outer constructer with no arguments
# (3) convert(PyObject, ...)
# (4) convert(..., o::PyObject)
function typecode(rosname::ASCIIString, super::Symbol, members::Vector)
    tname = _splittypestr(rosname)[2]
    @debug("Type: ", tname)

    #generated code should not conflict with julia built-ins
    #some messages need renaming
    suffix = if super == :MsgT; "Msg"
         elseif super == :SrvT; "Srv"
         else; "ROS" end
    jlsym = symbol(_jl_safe_name(tname,suffix))

    exprs = Expr[]
    #First the empty expressions
    #(1) Type declaration
    push!(exprs, :(
        type $jlsym <: $super
            #Generated code here
        end
    ))
    #(2) Default constructor, but only if the type has members
    if length(members) > 0
        push!(exprs, :(
            function $jlsym()
                $jlsym() #Generated code inside parens here
            end
        ))
    else
        push!(exprs, :())
    end
    #(3) Convert to PyObject
    push!(exprs, :(
        function convert(::Type{PyObject}, o::$jlsym)
            py = pycall(RobotOS._rospy_objects[$rosname], PyObject)
            #Generated code here
            py
        end
    ))
    #(4) Convert from PyObject
    push!(exprs, :(
        function convert(jlt::Type{$jlsym}, o::PyObject)
            if convert(ASCIIString, o["_type"]) != _typerepr(jlt)
                throw(InexactError())
            end
            jl = $jlsym()
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
    push!(exprs, :(_typerepr(::Type{$jlsym}) = $rosname))
    exprs
end


#Add the generated expression from a single member of a type, either built-in
#or ROS type. `exprs` is the Expr objects of the items created in `typecode`.
#Maybe this can be factored into something nicer.
function _addtypemember!(exprs, namestr, typestr)
    @debug("$namestr :: $typestr")
    typeargs  = exprs[1].args[3].args
    consargs  = exprs[2].args[2].args[2].args
    pyconargs = exprs[3].args[2].args
    jlconargs = exprs[4].args[2].args

    if typestr == "char" || typestr == "byte"
        warn("Use of type '$typestr' is deprecated in message definitions, ",
        "use '$(lowercase(string(_ros_builtin_types[typestr])))' instead.")
    end

    typestr, arraylen = _check_array_type(typestr)
    if _isrostype(typestr)
        j_typ = symbol(_splittypestr(typestr)[2])
        #Default has to be deferred until the types exist
        j_def = Expr(:call, j_typ)
    else
        if ! haskey(_ros_builtin_types, typestr)
            error("Message generation; unknown type '$typestr'")
        end
        j_typ = _ros_builtin_types[typestr]
        #Compute the default value now
        j_def = @eval _typedefault($j_typ)
    end

    namesym = symbol(namestr)
    if arraylen >= 0
        memexpr = :($namesym::Array{$j_typ,1})
        defexpr = :([$j_def for i = 1:$arraylen])
        jlconexpr = :(jl.$namesym = convert(Array{$j_typ,1}, o[$namestr]))

        #uint8[] is string in rospy and PyCall's conversion to bytearray is
        #rejected by ROS
        if j_typ == :UInt8
            pyconexpr = :(py[$namestr] =
                pycall(pybuiltin("str"), PyObject, PyObject(o.$namesym))
            )
        elseif _isrostype(typestr)
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

#Build a String => Iterable{String} object from the individual package
#dependencies.
function _collectdeps{S<:AbstractString}(pkgs::Dict{S, ROSPackage})
    deps = Dict{S, Set{S}}()
    for pname in keys(pkgs)
        if ! haskey(deps, pname)
            deps[pname] = Set{S}()
        end
        union!(deps[pname], pkgs[pname].msg.deps)
        union!(deps[pname], pkgs[pname].srv.deps)
    end
    deps
end

#Produce an order of the keys of d that respect their dependencies.
#Assumed to be Dict(String => Iterable{String})
function _order(d::Dict)
    trecurse!(currlist, d, t) = begin
        if !(t in currlist)
            if haskey(d, t) #do dependencies first
                for dt in d[t]
                    if dt != t
                        trecurse!(currlist, d, dt)
                    end
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

_rostypestr(mod::ROSModule, name::ASCIIString) = string(_name(mod),"/",name)
function _splittypestr(typestr::ASCIIString)
    if ! _isrostype(typestr)
        error(string("Invalid message type '$typestr', ",
                     "use 'package_name/type_name'"))
    end
    rospkg, typ = map(ascii, split(typestr, '/'))
    rospkg, typ
end
#Valid ROS type string is all word chars split by a single forward slash, with
#optional square brackets for array types
_isrostype(s::ASCIIString) = ismatch(r"^\w+/\w+(?:\[\d*\])?$", s)

#Sanitize a string by checking for and removing brackets if they are present
#Return the sanitized type and the number inside the brackets if it is a fixed
#size type. Returns 0 if variable size (no number), -1 if no brackets
function _check_array_type(typ::ASCIIString)
    arraylen = -1
    m = match(r"^([\w/]+)\[(\d*)\]$", typ)
    if m != nothing
        btype = m.captures[1]
        if isempty(m.captures[2])
            arraylen = 0
        else
            arraylen = parse(Int, m.captures[2])
        end
    else
        btype = typ
    end
    ascii(btype), arraylen
end

#Get the rospy PyObject corresponding to a generated type
function _get_rospy_class(typ::DataType)
    global _rospy_objects
    rospycls =
        try
            _rospy_objects[_typerepr(typ)]
        catch ex
            if isa(ex, KeyError)
                error("Type ($typ) is not generated")
            else
                error("Type ($typ) is not a valid message type")
            end
        end
    rospycls
end

_jl_safe_name(name::AbstractString, suffix) = _nameconflicts(name) ?
    string(name,suffix) :
    name

#Check if the type name conflicts with a Julia builtin. Currently this is only
#some of the messages from the std_msgs.msg package
_nameconflicts(typename::ASCIIString) = isdefined(Base, symbol(typename))

#Get a default value for any builtin ROS type
_typedefault{T<:Real}(::Type{T}) = zero(T)
_typedefault(::Type{ASCIIString}) = ""
_typedefault(::Type{Time}) = Time(0,0)
_typedefault(::Type{Duration}) = Duration(0,0)

#Default method to get the "pkg/type" string from a generated DataType.
#Extended by the generated modules.
function _typerepr end

#Default methods to get the request/response datatypes for a generated service
function _srv_reqtype end
function _srv_resptype end

#Accessors for the package name
_name(p::ROSPackage) = p.name
_name(m::ROSModule) = _name(m.pkg)

#Get the full ROS name for a module (e.g., 'std_msgs.msg' or nav_msgs.srv')
_fullname(m::ROSMsgModule) = string(_name(m), ".msg")
_fullname(m::ROSSrvModule) = string(_name(m), ".srv")
