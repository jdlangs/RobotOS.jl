#API for publishing and subscribing to message topics
export Publisher, Subscriber, publish

using Compat

type Publisher{MsgType<:MsgT}
    o::PyObject

    function Publisher(topic::AbstractString; kwargs...)
        @debug("Creating <$(string(MsgType))> publisher on topic: '$topic'")
        rospycls = _get_rospy_class(MsgType)
        return new(__rospy__[:Publisher](ascii(topic), rospycls; kwargs...))
    end
end
Publisher{MsgType<:MsgT}(topic::AbstractString, ::Type{MsgType}; kwargs...) =
    Publisher{MsgType}(ascii(topic); kwargs...)

function publish{MsgType<:MsgT}(p::Publisher{MsgType}, msg::MsgType)
    pycall(p.o["publish"], PyAny, convert(PyObject, msg))
end

type Subscriber{MsgType<:MsgT}
    callback
    callback_args::Tuple
    sub_obj::PyObject
    queue::PyObject
    async_loop::Task

    function Subscriber(topic::AbstractString, cb, cb_args::Tuple=(); kwargs...)
        @debug("Creating <$(string(MsgType))> subscriber on topic: '$topic'")
        rospycls = _get_rospy_class(MsgType)

        cond = Compat.AsyncCondition()
        mqueue = _py_ros_callbacks["MessageQueue"](CB_NOTIFY_PTR, cond.handle)
        subobj = __rospy__[:Subscriber](ascii(topic), rospycls, mqueue["storemsg"]; kwargs...)

        rosobj = new(cb, cb_args, subobj, mqueue)
        cbloop = Task(() -> _callback_async_loop(rosobj, cond))
        schedule(cbloop)

        rosobj.async_loop = cbloop
        return rosobj
    end
end

function Subscriber{MsgType<:MsgT}(topic, ::Type{MsgType}, cb, cb_args::Tuple=(); kwargs...)
    Subscriber{MsgType}(topic, cb, cb_args; kwargs...)
end

