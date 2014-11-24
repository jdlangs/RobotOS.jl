#Message Publishing API

type Publisher{MsgType}
    o::PyObject
end
function Publisher{MsgType}(
    topic::String, 
    typ::Type{MsgType}, 
    qs::Integer=10,
)
    msg = type_to_string(typ)
    if ! haskey(_msg_classes, msg)
        error("Message ($msg) has not been generated!")
    else
        Publisher{MsgType}(
            __rospy__.Publisher(topic, _msg_classes[msg]))
    end
end

function publish{MsgType}(p::Publisher{MsgType}, msg::MsgType)
    pycall(p.o["publish"], PyAny, convert(PyObject, msg))
end

function type_to_string(typ::DataType)
    mod_str = split(string(Base.function_module(typ)), '.')[end]
    "$mod_str/$typ"
end
