#API for calling/creating services. Syntax is practically identical to rospy.
export Service, ServiceProxy, wait_for_service

"""
    ServiceProxy{T}(name; kwargs...)
    ServiceProxy(name, T; kwargs...)

Create a proxy object used to invoke a remote service. Use `srv_proxy(msg_request)` with the object
to invoke the service call. Keyword arguments are directly passed to rospy.
"""
struct ServiceProxy{SrvType <: AbstractService}
    o::PyObject

    function ServiceProxy{ST}(name::AbstractString; kwargs...) where ST <: AbstractService
        @debug("Creating <$ST> service proxy for '$name'")
        rospycls = _get_rospy_class(ST)
        new{ST}(__rospy__.ServiceProxy(ascii(name), rospycls; kwargs...))
    end
end

function ServiceProxy(name::AbstractString, srv::Type{ST}; kwargs...) where ST <: AbstractService
    ServiceProxy{ST}(ascii(name); kwargs...)
end

function (srv::ServiceProxy{ST})(req::AbstractSrv) where ST <: AbstractService
    if ! isa(req, _srv_reqtype(ST))
        throw(ArgumentError(
            string("Incorrect service request type: ", typeof(req),
                   ", expected: ", _srv_reqtype(ST))))
    end
    pyresp = pycall(srv.o, PyObject, convert(PyObject, req))
    resp = convert(_srv_resptype(ST), pyresp)
    resp
end

"""
    Service{T}(name, callback; kwargs...)
    Service(name, T, callback; kwargs...)

Create a service object that can receive requests and provide responses. The callback can be of
any callable type. Keyword arguments are directly passed to rospy.
"""
mutable struct Service{SrvType <: AbstractService}
    handler
    srv_obj::PyObject
    cb_interface::PyObject
    async_loop::Task

    function Service{ST}(name::AbstractString, handler; kwargs...) where ST <: AbstractService
        @debug("Providing <$ST> service at '$name'")
        rospycls = _get_rospy_class(ST)

        cond = Base.AsyncCondition()
        pysrv = _py_ros_callbacks."ServiceCallback"(CB_NOTIFY_PTR[], cond.handle)

        srvobj = try
            __rospy__.Service(ascii(name), rospycls, pysrv."srv_cb"; kwargs...)
        catch err
            if isa(err, PyCall.PyError)
                error("Problem during service creation: $(err.val.args[1])")
            else
                rethrow(err)
            end
        end

        rosobj = new{ST}(handler, srvobj, pysrv)

        cbloop = Task(() -> _callback_async_loop(rosobj, cond))
        schedule(cbloop)

        rosobj.async_loop = cbloop
        return rosobj
    end
end

function Service(name::AbstractString, srv::Type{ST}, handler; kwargs...) where ST <: AbstractService
    Service{ST}(ascii(name), handler; kwargs...)
end

"""
    wait_for_service(srv_name; kwargs...)

Block until the specified service is available. Keyword arguments are directly passed to rospy.
Throws an exception if the waiting timeout period is exceeded.
"""
function wait_for_service(service::AbstractString; kwargs...)
    try
        __rospy__.wait_for_service(ascii(service); kwargs...)
    catch ex
        error("Timeout exceeded waiting for service '$service'")
    end
end
