module ROS

export genmsgs, init_node

using PyCall
const __rospy__ = try
    pywrap(pyimport("rospy"))
catch ex
    if ex.val[:args][1] == "No module named rospy"
        error("rospy not found!\nHas an environment setup script been run?")
    end
end

include("time.jl")
include("gentypes.jl")

function init_node(node_name::String, args...)
    __rospy__.init_node(node_name)
end

end
