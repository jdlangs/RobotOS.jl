#API for calling/creating services. Syntax is practically identical to rospy.
export Service, ServiceProxy, wait_for_service

using Compat

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

@compat function (srv::ServiceProxy{SrvType}){SrvType <: ServiceDefinition}(
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
    handler
    srv_obj::PyObject
    cb_interface::PyObject
    async_loop::Task

    function Service(name::AbstractString, handler; kwargs...)
        @debug("Providing <$SrvType> service at '$name'")
        rospycls = _get_rospy_class(SrvType)

        cond = Compat.AsyncCondition()
        pysrv = _py_ros_callbacks["ServiceCallback"](CB_NOTIFY_PTR, cond.handle)

        srvobj = try
            __rospy__[:Service](ascii(name), rospycls, pysrv["srv_cb"]; kwargs...)
        catch err
            if isa(err, PyCall.PyError)
                error("Problem during service creation: $(err.val[:args][1])")
            else
                rethrow(err)
            end
        end

        rosobj = new(handler, srvobj, pysrv)

        cbloop = Task(() -> _callback_async_loop(rosobj, cond))
        schedule(cbloop)

        rosobj.async_loop = cbloop
        return rosobj
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
