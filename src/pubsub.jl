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
    callback::Function

    function Subscriber(
        topic::String, cb::Function, cb_args = (); kwargs...
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
    cb::Function,
    cb_args = ();
    kwargs...
) = Subscriber{MsgType}(topic, cb, cb_args; kwargs...)

function _get_rospy_class(typ::DataType)
    rospycls =
        try
            _rospy_classes[RobotOS._typerepr(typ)]
        catch ex
            if isa(ex, KeyError)
                error("Type ($typ) is not generated")
            else
                error("Type ($typ) is not a valid message type")
            end
        end
    rospycls
end
