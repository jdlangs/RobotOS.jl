#API for calling/creating services. Syntax is practically identical to rospy.
export Service, ServiceProxy, wait_for_service

if VERSION < v"0.4-"
    export call
else
    import Base.call
end

type ServiceProxy{SrvType <: ServiceDefinition}
    o::PyObject

    function ServiceProxy(name::AbstractString; kwargs...)
        @debug("Creating <$SrvType> service proxy for '$name'")
        rospycls = _get_rospy_class(SrvType)
        new(__rospy__[:ServiceProxy](ascii(name), rospycls; kwargs...))
    end
end
function ServiceProxy{SrvType<:ServiceDefinition}(
    name::AbstractString,
    srv::Type{SrvType};
    kwargs...
)
    ServiceProxy{SrvType}(ascii(name); kwargs...)
end

function call{SrvType <: ServiceDefinition}(
    srv::ServiceProxy{SrvType},
    req::SrvT
)
    if ! isa(req, _srv_reqtype(SrvType))
        throw(ArgumentError(
            string("Incorrect service request type: ",typeof(req))))
    end
    pyresp = pycall(srv.o, PyObject, convert(PyObject, req))
    resp = convert(_srv_resptype(SrvType), pyresp)
    resp
end

type Service{SrvType <: ServiceDefinition}
    o::PyObject
    jl_handler

    function Service(name::AbstractString, handler; kwargs...)
        @debug("Providing <$SrvType> service at '$name'")
        rospycls = _get_rospy_class(SrvType)
        ReqType = _srv_reqtype(SrvType)
        jl_hndl(req::PyObject) =
            convert(PyObject, handler(convert(ReqType,req)))
        try
            new(__rospy__[:Service](ascii(name), rospycls, jl_hndl; kwargs...),
                jl_hndl
            )
        catch err
            if isa(err, PyCall.PyError)
                error("Problem during service creation: $(err.val[:args][1])")
            else
                rethrow(err)
            end
        end
    end
end
function Service{SrvType<:ServiceDefinition}(
    name::AbstractString,
    srv::Type{SrvType},
    handler;
    kwargs...
)
    Service{SrvType}(ascii(name), handler; kwargs...)
end

function wait_for_service(service::AbstractString; kwargs...)
    try
        __rospy__[:wait_for_service](ascii(service); kwargs...)
    catch ex
        error("Timeout exceeded waiting for service '$service'")
    end
end
