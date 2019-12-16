module ROS2

include("clang_wrap.jl")

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
