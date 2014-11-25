#Message Publishing API

type Publisher{MsgType}
    o::PyObject
end
function Publisher{MsgType}(
    topic::String, 
    typ::Type{MsgType}; 
    kwargs...
)
    msgtype = _type_to_string(typ)
    if ! haskey(_msg_classes, msgtype)
        error("Type ($msgtype) has not been generated!")
    else
        Publisher{MsgType}(
            __rospy__.Publisher(topic, _msg_classes[msgtype]; kwargs...)
        )
    end
end

function publish{MsgType}(p::Publisher{MsgType}, msg::MsgType)
    pycall(p.o["publish"], PyAny, convert(PyObject, msg))
end

type Subscriber{MsgType}
    o::PyObject
end
function Subscriber{MsgType}(
    topic::String,
    typ::Type{MsgType},
    callback::Function,
    callback_args = ();
    kwargs...
)
    msgtype = _type_to_string(typ)
    jl_callback(msg::PyObject) = callback(convert(typ, msg), callback_args...)
    if ! haskey(_msg_classes, msgtype)
        error("Type ($msgtype) has not been generated!")
    else
        Subscriber{MsgType}(
            __rospy__.Subscriber(
                topic,
                _msg_classes[msgtype],
                jl_callback;
                kwargs...
            )
        )
    end
end

function _type_to_string(typ::DataType)
    mod_str = split(string(Base.function_module(typ)), '.')[end]
    "$mod_str/$typ"
end
