#API for publishing and subscribing to message topics
export Publisher, Subscriber, publish

using Compat

"""
    Publisher{T}(topic; kwargs...)
    Publisher(topic, T; kwargs...)

Create an object to publish messages of type `T` on a topic. Keyword arguments are directly passed
to rospy.
"""
type Publisher{MsgType<:AbstractMsg}
    o::PyObject

    @compat function (::Type{Publisher{MT}}){MT <: AbstractMsg}(topic::AbstractString; kwargs...)
        @debug("Creating <$(string(MT))> publisher on topic: '$topic'")
        rospycls = _get_rospy_class(MT)
        return new{MT}(__rospy__[:Publisher](ascii(topic), rospycls; kwargs...))
    end
end

Publisher{MT<:AbstractMsg}(topic::AbstractString, ::Type{MT}; kwargs...) =
    Publisher{MT}(ascii(topic); kwargs...)

"""
    publish(p::Publisher{T}, msg::T)

Publish `msg` on `p`, a `Publisher` with matching message type.
"""
function publish{MT<:AbstractMsg}(p::Publisher{MT}, msg::MT)
    pycall(p.o["publish"], PyAny, convert(PyObject, msg))
end

"""
    Subscriber{T}(topic, callback, cb_args=(); kwargs...)
    Subscriber(topic, T, callback, cb_args=(); kwargs...)

Create a subscription to a topic with message type `T` with a callback to use when a message is
received, which can be any callable type. Extra arguments provided to the callback when invoked
can be provided in the `cb_args` tuple. Keyword arguments are directly passed to rospy.
"""
type Subscriber{MsgType<:AbstractMsg}
    callback
    callback_args::Tuple
    sub_obj::PyObject
    queue::PyObject
    async_loop::Task

    @compat function (::Type{Subscriber{MT}}){MT <: AbstractMsg}(
        topic::AbstractString, cb, cb_args::Tuple=(); kwargs...
    )
        @debug("Creating <$(string(MT))> subscriber on topic: '$topic'")
        rospycls = _get_rospy_class(MT)

        cond = Compat.AsyncCondition()
        mqueue = _py_ros_callbacks["MessageQueue"](CB_NOTIFY_PTR, cond.handle)
        subobj = __rospy__[:Subscriber](ascii(topic), rospycls, mqueue["storemsg"]; kwargs...)

        rosobj = new{MT}(cb, cb_args, subobj, mqueue)
        cbloop = Task(() -> _callback_async_loop(rosobj, cond))
        schedule(cbloop)

        rosobj.async_loop = cbloop
        return rosobj
    end
end

function Subscriber{MT<:AbstractMsg}(topic, ::Type{MT}, cb, cb_args::Tuple=(); kwargs...)
    Subscriber{MT}(topic, cb, cb_args; kwargs...)
end
