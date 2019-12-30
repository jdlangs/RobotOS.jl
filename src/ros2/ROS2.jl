module ROS2

# include generated roslib wrappers
using CEnum

include("extra_c_defs.jl")

function include_roslib(libname::String)
    include("../../roslibs/lib$(libname)_common.jl")
    include("../../roslibs/lib$(libname)_api.jl")
end

include_roslib("rosidl_generator_c")
include_roslib("rosidl_typesupport_introspection_c")

# this package's code
include("clang_wrap.jl")
include("typegen.jl")

function init()
    rcl_init(argc, argv, options, context)
    context
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
