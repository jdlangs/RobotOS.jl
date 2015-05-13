#Generate Julia composite types for ROS messages
using Compat

#Composite types for internal use. Keeps track of the imported types and helps
#keep code generation orderly.
abstract ROSModule
type ROSMsgModule <: ROSModule
    name::ASCIIString
    members::Vector{ASCIIString}
    deps::Set{ASCIIString}
    function ROSMsgModule(mname::String)
        new(mname, 
            ASCIIString[],
            Set{ASCIIString}()
        )
    end
end
type ROSSrvModule <: ROSModule
    name::ASCIIString
    members::Vector{ASCIIString}
    deps::Set{ASCIIString}
    function ROSSrvModule(mname::String)
        new(mname, 
            ASCIIString[],
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

#These two global objects maintain the hierarchy from multiple calls to
#`@rosimport` and keep the link to the Python objects whenever communication
#goes between RobotOS and rospy.
const _rospy_imports = Dict{ASCIIString,ROSPackage}()
const _rospy_objects = Dict{ASCIIString,PyObject}()

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
        tsym = input.args[2].args[1]
        @assert isa(tsym, Symbol) "Type name ($(string(tsym))) not a symbol"
        ts = string(tsym)
    end
    return ps,msb,ts
end
#Import a set of types from a single package
function _usepkg(package::String, ismsg::Bool, names::String...)
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
function gentypes()
    global _rospy_imports
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
    global _rospy_imports
    global _rospy_objects
    empty!(_rospy_imports)
    empty!(_rospy_objects)
    nothing
end

#Populate the module with a new message type. Import and add dependencies first
#so they will appear first in the generated code.
function addtype!(mod::ROSMsgModule, typ::String)
    global _rospy_objects
    if ! (typ in mod.members)
        @debug("Message type import: ", _modname(mod), ".", typ)
        pymod, pyobj = _pyvars(_modname(mod), typ)

        deptypes = pyobj[:_slot_types]
        _importdeps!(mod, deptypes)

        push!(mod.members, typ)
        _rospy_objects[_rostypestr(mod, typ)] = pyobj
    end
end

#Populate the module with a new service type. Import and add dependencies
#first.
function addtype!(mod::ROSSrvModule, typ::String)
    global _rospy_objects
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
        fulltypestr = _rostypestr(mod, typ)
        _rospy_objects[fulltypestr] = pyobj
        _rospy_objects[string(fulltypestr,"Request")] = req_obj
        _rospy_objects[string(fulltypestr,"Response")] = resp_obj
    end
end

#Return the python module and python object for a particular type
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

#Continue the import process on a list of dependencies. Called by `addtype!`
#and calls `addtype!` to complete the dependency recursion.
function _importdeps!(mod::ROSModule, deps::Vector)
    global _rospy_imports
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
            #pushing to a set does not create duplicates
            push!(mod.deps, depmod.name)

            #Don't want to include the brackets in the string if present
            typeclean = _check_array_type(typename)[1]
            addtype!(depmod, typeclean)
            @debug_subindent
        end
    end
end

#Bring in the python modules as needed
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

#The function that creates and fills the generated top-level modules
function buildpackage(pkg::ROSPackage)
    @debug("Building package: ", pkg.name)

    #Create the top-level module for the package in Main
    pkgsym = symbol(pkg.name)
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
    @debug("submodule: ", _modname(mod))
    modcode = Expr[]

    #Common imports
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
    othermods = filter(d -> d != mod.name, mod.deps)
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
    exports = [symbol(m) for m in mod.members]
    Expr(:export, exports...)
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
function buildtype(mod::ROSMsgModule, typename::String)
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
function buildtype(mod::ROSSrvModule, typename::String)
    global _rospy_objects
    fulltypestr = _rostypestr(mod, typename)

    req_str = string(fulltypestr,"Request")
    reqobj = _rospy_objects[req_str]
    memnames = reqobj[:__slots__]
    memtypes = reqobj[:_slot_types]
    reqmems = collect(zip(memnames, memtypes))
    pyreq  = :(RobotOS._rospy_objects[$req_str])
    reqexprs  = typecode(req_str, :SrvT, reqmems)

    resp_str = string(fulltypestr,"Response")
    respobj = _rospy_objects[resp_str]
    memnames = respobj[:__slots__]
    memtypes = respobj[:_slot_types]
    respmems = collect(zip(memnames, memtypes))
    pyresp = :(RobotOS._rospy_objects[$resp_str])
    respexprs = typecode(resp_str, :SrvT, respmems)

    defsym = symbol(typename)
    reqsym = symbol(string(typename,"Request"))
    respsym = symbol(string(typename,"Response"))
    srvexprs = Expr[
        :(type $defsym <: ServiceDefinition end),
        :(_typerepr(::Type{$defsym}) = $(_rostypestr(mod,typename))),
        :(_srv_reqtype(::Type{$defsym}) = $reqsym),
        :(_srv_resptype(::Type{$defsym}) = $respsym),
    ]
    [reqexprs; respexprs; srvexprs]
end

#Create the core generated expressions for a native Julia message type that has
#data fields and interchanges with a python counterpart: 
# (1) the 'type ... end' block
# (2) No param outer constructer
# (3) convert(PyObject, ...)
# (4) convert(..., o::PyObject)
function typecode(rosname::String, super::Symbol, members::Vector)
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
            py = pycall(RobotOS._rospy_objects[$rosname], PyObject)
            #Generated code here
            py
        end
    ))
    #(4) Convert from PyObject
    push!(exprs, :(
        function convert(jlt::Type{$tsym}, o::PyObject)
            if convert(ASCIIString, o["_type"]) != _typerepr(jlt)
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
        j_def = Expr(:call, j_typ)
    else
        if ! haskey(_ros_builtin_types, typestr)
            error("Message generation; unknown type '$typestr'")
        end
        j_typ = _ros_builtin_types[typestr]
        j_def = Expr(:call, :_typedefault, j_typ)
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
function _collectdeps{S<:String}(pkgs::Dict{S, ROSPackage})
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

#Produce an order of the keys of d that respect their dependencies.
#Assumed to be Dict(String => Iterable{String})
function _order(d::Dict)
    trecurse!(currlist, d, t) = begin
        if ! (t in currlist)
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

_rostypestr(mod::ROSModule, name::String) = string(mod.name,"/",name)
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

#Get the ROS name string corresponding to a generated type
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

#Get a default value for any builtin ROS type
_typedefault{T<:Real}(::Type{T}) = zero(T)
_typedefault(::Type{ASCIIString}) = ""
_typedefault(::Type{Time}) = Time(0,0)
_typedefault(::Type{Duration}) = Duration(0,0)

#Default method to get the "pkg/type" string from a generated DataType.
#Extended by the generated modules.
_typerepr{T}(::Type{T}) = error("Not a ROS type")

#Default method to get the request/response datatypes for a generated service
_srv_reqtype{T} (::Type{T}) = error("Not a ROS Service type")
_srv_resptype{T}(::Type{T}) = error("Not a ROS Service type")

#Get the full ROS name for a module (e.g., 'std_msgs.msg' or nav_msgs.srv')
_modname(m::ROSMsgModule) = string(m.name, ".msg")
_modname(m::ROSSrvModule) = string(m.name, ".srv")
