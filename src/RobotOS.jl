module RobotOS

using PyCall

#Empty imported modules for valid precompilation
const _py_sys = PyCall.PyNULL()
const _py_ros_callbacks = PyCall.PyNULL()
const __rospy__ = PyCall.PyNULL()

include("debug.jl")
include("time.jl")
include("gentypes.jl")
include("rospy.jl")
include("pubsub.jl")
include("services.jl")
include("callbacks.jl")

function __init__()
    #Put julia's ARGS into python's so remappings will work
    copy!(_py_sys, pyimport("sys"))
    _py_sys["argv"] = ARGS

    #Fill in empty PyObjects
    if ! (dirname(@__FILE__) in _py_sys["path"])
        pushfirst!(_py_sys["path"], dirname(@__FILE__))
    end
    copy!(_py_ros_callbacks, pyimport("ros_callbacks"))

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

    #Compile the callback notify function, see callbacks.jl
    CB_NOTIFY_PTR[] = @cfunction(_callback_notify, Cint, (Ptr{Cvoid},))
end

end
