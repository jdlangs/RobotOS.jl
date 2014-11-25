#General rospy functions

function init_node(node_name::String; args...)
    __rospy__.init_node(node_name; args...)
    pygui_start(:tk) #Need to figure out what's going on behind the scenes here
    nothing
end

spin()                 = __rospy__.spin()
is_shutdown()          = __rospy__.is_shutdown()
get_caller_id()        = __rospy__.get_caller_id()
get_name()             = __rospy__.get_name()
get_namespace()        = __rospy__.get_namespace()
get_node_uri()         = __rospy__.get_node_uri()
get_param_names()      = __rospy__.get_param_names()
get_published_topics() = __rospy__.get_published_topics()
get_ros_root()         = __rospy__.get_ros_root()
