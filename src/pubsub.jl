#API for publishing and subscribing to message topics
export Publisher, Subscriber, publish

type Publisher{MsgType<:MsgT}
    o::PyObject

    function Publisher(topic::String; kwargs...)
        @debug("Creating <$(string(MsgType))> publisher on topic: '$topic'")
        rospycls = _get_rospy_class(MsgType)
        return new(__rospy__.Publisher(topic, rospycls; kwargs...))
    end
end
Publisher{MsgType<:MsgT}(topic::String, ::Type{MsgType}; kwargs...) =
    Publisher{MsgType}(topic; kwargs...)

function publish{MsgType<:MsgT}(p::Publisher{MsgType}, msg::MsgType)
    pycall(p.o["publish"], PyAny, convert(PyObject, msg))
end

type Subscriber{MsgType<:MsgT}
    o::PyObject
    callback

    function Subscriber(
        topic::String, cb, cb_args::Tuple=(); kwargs...
    )
        @debug("Creating <$(string(MsgType))> subscriber on topic: '$topic'")
        rospycls = _get_rospy_class(MsgType)
        jl_callback(msg::PyObject) = cb(convert(MsgType, msg), cb_args...)
        return new(
            __rospy__.Subscriber(topic, rospycls, jl_callback; kwargs...),
            jl_callback
        )
    end
end
Subscriber{MsgType<:MsgT}(
    topic::String,
    ::Type{MsgType},
    cb,
    cb_args::Tuple=();
    kwargs...
) = Subscriber{MsgType}(topic, cb, cb_args; kwargs...)
