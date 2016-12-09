module RobotOS

using PyCall

#Empty imported modules for valid precompilation
const __sys__ = PyCall.PyNULL()
const __rospy__ = PyCall.PyNULL()

function __init__()
    #Put julia's ARGS into python's so remappings will work
    copy!(__sys__, pyimport("sys"))
    __sys__["argv"] = ARGS

    try
        copy!(__rospy__, pyimport("rospy"))
    catch ex
        if (isa(ex, PyCall.PyError) &&
            pycall(pybuiltin("str"), PyAny, ex.val) == "No module named rospy")
            error("rospy not found!\nHas an environment setup script been run?")
        else
            rethrow(ex)
        end
    end
end

_threads_enabled() = ccall(:jl_threading_enabled, Cint, ()) != 0

include("debug.jl")
include("time.jl")
include("gentypes.jl")
include("rospy.jl")
include("pubsub.jl")
include("services.jl")

end
