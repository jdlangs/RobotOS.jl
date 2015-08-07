module RobotOS

using PyCall
const __rospy__ = try
    pyimport("rospy")
catch ex
    if ex.val[:args][1] == "No module named rospy"
        error("rospy not found!\nHas an environment setup script been run?")
    end
end

#Put julia's ARGS into python's so remappings will work
py_sys = pyimport("sys")
py_sys["argv"] = ARGS

include("debug.jl")
include("time.jl")
include("gentypes.jl")
include("rospy.jl")
include("pubsub.jl")
include("services.jl")

end
