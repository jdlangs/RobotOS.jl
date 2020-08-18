module ROS2

# include generated roslib wrappers
using CEnum

include("extra_c_defs.jl")

function include_roslib(libname::String)
    include("../../roslibs/lib$(libname)_common.jl")
    include("../../roslibs/lib$(libname)_api.jl")
end

#include_roslib("rosidl_generator_c")
#include_roslib("rosidl_typesupport_introspection_c")
#include_roslib("rcutils")
#include_roslib("rmw")
#include_roslib("rcl")

# Circular definitions removed to here (TODO)
#const RCUTILS_STEADY_TIME = rcutils_steady_time_now #from rcutils_common

# this package's code
include("clang_wrap.jl")
include("typegen.jl")

# Create a full Clang of all the ROS2 packages we need in the correct order
function wrap_ros2()
    wrap_rospkg("rcutils")
    wrap_rospkg("rosidl_generator_c")
    wrap_rospkg("rosidl_typesupport_introspection_c")
    wrap_rospkg("rmw")
    wrap_rospkg("rcl")
end

const LIBRCL = :librcl

function load()
    librcl[] = Libdl.dlopen("librcl")
end

function unload()
    Libdl.dlclose(librcl[])
end

function init()
    argc = 0
    argv = C_NULL
    context = rcl_get_zero_initialized_context()
    options = rcl_get_zero_initialized_options()
    rcl_init(argc, argv, options, context)
end

function shutdown(context)
    rcl_shutdown(context)
end

function create_node(context)
    node = rcl_get_zero_initialized_node()
    rcl_node_init(node, name, namespace, context, options)
end

function spin(node)
end

function spin_once(node)
end

function create_publisher(node, msgtype, topic, qos_profile)
    publisher = rcl_get_zero_initialized_publisher()
    rcl_publisher_init(publisher, node, typesupport, topic_name, options)
end

function create_client(node, srvtype, srv_name)
end

function publish(pub, msg)
    rcl_publish(pub, msg, C_NULL)
end

function main()
    init()
    node = create_node("rosjl_test")
    pub = create_publisher(node)

    for i=1:10
        pt = geometry_msgs.Point(1,2,3)
        publish(pub, pt)
        sleep(1.0)
    end

    shutdown(context)
end

end
