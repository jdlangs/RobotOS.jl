module ROS

export genmsgs

using PyCall
try
    @pyimport rospy
catch ex
    if ex.val[:args][1] == "No module named rospy"
        error("rospy not found!\nHas an environment setup script been run?")
    end
end

include("time.jl")
include("gentypes.jl")

end
