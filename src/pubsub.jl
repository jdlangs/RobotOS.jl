#API for publishing and subscribing to message topics

type Publisher{MsgType<:MsgT}
    o::PyObject

    function Publisher(topic::String; kwargs...)
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

    function Subscriber(topic::String, cb::Function, cb_args = (); kwargs...)
        rospycls = _get_rospy_class(MsgType)
        jl_callback(msg::PyObject) = cb(
            convert(MsgType, msg),
            cb_args...
        )
        return new(
            __rospy__.Subscriber(
                topic,
                rospycls,
                jl_callback;
                kwargs...
            ),
            jl_callback
        )
    end
end
Subscriber{MsgType<:MsgT}(
    topic::String,
    ::Type{MsgType},
    cb::Function,
    cb_args=();
    kwargs...
) = Subscriber{MsgType}(topic, cb, cb_args; kwargs...)

#Utility func to form a full string of the type including which module it's in
#This works in v0.4 and avoids the need to keep a Dict in the ROS module
#function _type_to_string(typ::DataType)
    #mod_str = split(string(Base.function_module(typ)), '.')[end]
    #"$mod_str/$typ"
#end

function _get_rospy_class(typ::DataType)
    rospycls =
        try
            _rospy_classes[_jltype_strs[typ]] #_type_to_string(typ)
        catch KeyError
            error("Type ($typ) is not generated or not publishable")
        end
    rospycls
end
