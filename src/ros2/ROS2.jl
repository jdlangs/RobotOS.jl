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

struct Point
    x::Float64
    y::Float64
    z::Float64
end

function init()
    rcl_init(argc, argv, options, context)
    context
end

function shutdown(context)
    rcl_shutdown(context)
end

function mknode(context)
    node = rcl_get_zero_initialized_node()
    rcl_node_init(node, name, namespace, context, options)
end

function mkpub(node)
    publisher = rcl_get_zero_initialized_publisher()
    rcl_publisher_init(publisher, node, typesupport, topic_name, options)
end

function to_msg(pt::Point)
end

function publish(pub, msg)
    rcl_publish(pub, msg, C_NULL)
end

function main()
    context = init()
    node = mknode()
    pub = mkpub(node)

    for i=1:10
        pt = Point(1,2,3)
        publish(pub, to_msg(pt))
        sleep(1.0)
    end

    shutdown(context)
end

end
