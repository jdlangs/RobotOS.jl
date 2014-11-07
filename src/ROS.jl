module ROS

using PyCall
@pyimport rospy

include("ros_types.jl")

function init_node(name::String)
    rospy.init_node(name)
end

function subscribe(topic::String, msgtype::String, callback::Function)
    class = message_class(msgtype)
    sub = rospy.Subscriber(topic, class, callback)
    Subscriber(sub)
end

function advertise(topic::String, msgtype::String)
    class = message_class(msgtype)
    pub = rospy.Publisher(topic, class)
    Publisher(pub)
end

function spin(t::Real)
    rospy.sleep(t)
end

function message_class(msgtype::String)
    if ! ismatch(r"^\w+/\w+$", msgtype)
        error("Incorrect message type '$msgtype', use 'package_name/message'")
    end
    package, message = split(msgtype, '/')
    pkg_sym = symbol(package)
    eval(:(@pyimport $pkg_sym.msg as $pkg_sym))
    class = eval(:($pkg_sym.pymember($message)))
    class
end

end
