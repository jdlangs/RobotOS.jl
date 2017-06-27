#Wrappers for functions directly in the rospy namespace
export init_node, is_shutdown, spin,
       get_param, has_param, set_param, delete_param,
       logdebug, loginfo, logwarn, logerr, logfatal

"""
    init_node(name; args...)

Initialize this node, registering it with the ROS master. All arguments are passed on directly to
the rospy init_node function.
"""
init_node(name::AbstractString; args...) =
    __rospy__[:init_node](ascii(name); args...)

"""
    is_shutdown()

Return the shutdown status of the node.
"""
is_shutdown()          = __rospy__[:is_shutdown]()

get_published_topics() = __rospy__[:get_published_topics]()
get_ros_root()         = __rospy__[:get_ros_root]()

"""
    spin()

Block execution and process callbacks/service calls until the node is shut down.
"""
function spin()
    #Have to make sure both Julia tasks and python threads can wake up so
    #can't just call rospy's spin
    while ! is_shutdown()
        rossleep(Duration(0.001))
    end
end

#Parameter server API
"""
    get_param(param_name, default=nothing)

Request the value of a parameter from the parameter server, with optional default value. If no
default is given, throws a `KeyError` if the parameter cannot be found.
"""
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

"""
    set_param(param_name, val)

Set the value of a parameter on the parameter server.
"""
set_param(param_name::AbstractString, val) =
    __rospy__[:set_param](ascii(param_name), val)

"""
    has_param(param_name)

Return a boolean specifying if a parameter exists on the parameter server.
"""
has_param(param_name::AbstractString) =
    __rospy__[:has_param](ascii(param_name))

"""
    delete_param(param_name)

Delete a parameter from the parameter server. Throws a `KeyError` if no such parameter exists.
"""
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

"""
    logdebug, loginfo, logwarn, logerr, logfatal

Call the rospy logging system at the corresponding message level, passing a message and other
arguments directly.
"""
logdebug, loginfo, logwarn, logerr, logfatal

#Node information
get_name()             = __rospy__[:get_name]()
get_namespace()        = __rospy__[:get_namespace]()
get_node_uri()         = __rospy__[:get_node_uri]()
get_caller_id()        = __rospy__[:get_caller_id]()
