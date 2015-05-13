module RobotOS

#Interior code can use this macro for debugging output
macro debug(expr, other...)
    :(if _debug_output println($expr,$(other...)) end)
end
_debug_output = false
debug(d::Bool) = global _debug_output = d

using PyCall
const __rospy__ = try
    pywrap(pyimport("rospy"))
catch ex
    if ex.val[:args][1] == "No module named rospy"
        error("rospy not found!\nHas an environment setup script been run?")
    end
end

#Put julia's ARGS into python's so remappings will work
py_sys = pyimport("sys")
py_sys["argv"] = ARGS

include("time.jl")
include("gentypes.jl")
include("rospy.jl")
include("pubsub.jl")

end
