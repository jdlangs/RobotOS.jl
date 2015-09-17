#Wrappers for functions directly in the rospy namespace
export init_node, is_shutdown, spin,
       get_param, has_param, set_param, delete_param,
       logdebug, loginfo, logwarn, logerr, logfatal

#General rospy functions
init_node(name::AbstractString; args...) =
    __rospy__[:init_node](ascii(name); args...)
spin()                 = __rospy__[:spin]()
is_shutdown()          = __rospy__[:is_shutdown]()
get_published_topics() = __rospy__[:get_published_topics]()
get_ros_root()         = __rospy__[:get_ros_root]()

#Parameter server API
function get_param(param_name::AbstractString, def=nothing)
    try
        if def == nothing
            __rospy__[:get_param](ascii(param_name))
        else
            __rospy__[:get_param](ascii(param_name), def)
        end
    catch ex
        throw(KeyError(pycall(pybuiltin("str"), PyAny, ex.val)[2:end-1]))
    end
end
set_param(param_name::AbstractString, val) =
    __rospy__[:set_param](ascii(param_name), val)
has_param(param_name::AbstractString) =
    __rospy__[:has_param](ascii(param_name))
function delete_param(param_name::AbstractString)
    try
        __rospy__[:delete_param](ascii(param_name))
    catch ex
        throw(KeyError(pycall(pybuiltin("str"), PyAny, ex.val)[2:end-1]))
    end
end
#Doesn't work for some reason
#rospy_search_param(param_name::AbstractString) =
#    __rospy__[:rospy_search_param](ascii(param_name))
get_param_names() = __rospy__[:get_param_names]()

#Logging API
logdebug(msg, args...) = __rospy__[:logdebug](msg, args...)
loginfo(msg, args...)  = __rospy__[:loginfo](msg, args...)
logwarn(msg, args...)  = __rospy__[:logwarn](msg, args...)
logerr(msg, args...)   = __rospy__[:logerr](msg, args...)
logfatal(msg, args...) = __rospy__[:logfatal](msg, args...)

#Node information
get_name()             = __rospy__[:get_name]()
get_namespace()        = __rospy__[:get_namespace]()
get_node_uri()         = __rospy__[:get_node_uri]()
get_caller_id()        = __rospy__[:get_caller_id]()
