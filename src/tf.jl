module TF

using RobotOS
using PyCall
import Base.==

const __tf__ = PyCall.PyNULL()

function __init__()
    copy!(__tf__, pyimport("tf"))
end

export Transform, TransformListener, lookupTransform, waitForTransform

struct Transform
    trans::Array{Float64}
    rot::Array{Float64}
end

Base.:(==)(tf1::Transform, tf2::Transform) = (tf1.trans == tf2.trans && tf1.rot == tf2.rot)


"""
    TransformListener()
Create a transform listener object.
"""
struct TransformListener
    o::PyObject
    function TransformListener()
        new(__tf__.TransformListener())
    end
end

"""
    generate_error_message(err) 
Retrun error message string which includes both exception type and error massage information.
"""
function generate_error_message(err)
    exception_type = err.T.__name__
    error_massage = exception_type * ": $(err.val.args[1])"
end

"""
    lookupTransform(tf_listener_obj, target, source, time) 
Return tuple of (position, quaternion).
"""
function lookupTransform(tl::TransformListener,
                         target_frame::AbstractString,
                         source_frame::AbstractString,
                         pytime::Time)
    time = convert(PyObject, pytime)
    try
        trans, rot = pycall(tl.o.lookupTransform, PyAny, target_frame, source_frame, time)
        return Transform(trans, rot)
    catch err
        if isa(err, PyCall.PyError)
            error_massage = generate_error_message(err)
            error(error_massage)
        else
            rethrow(err)
        end
    end
end

"""
    waitForTransform(tf_listener_obj, target, source, time, timeout, pypolling_sleep_duration) 
Waits for the given transformation to become available. If the timeout occurs before the transformation becomes available, raises an exception.
"""
function waitForTransform(tl::TransformListener,
                          target_frame::AbstractString, 
                          source_frame::AbstractString,
                          pytime::Time,
                          pytimeout::Duration;
                          pypolling_sleep_duration = Duration(0.01))
    time = convert(PyObject, pytime)
    timeout = convert(PyObject, pytimeout)
    polling_sleep_duration = convert(PyObject, pypolling_sleep_duration)
    try
        pycall(tl.o.waitForTransform, PyAny, target_frame, source_frame,
               time, timeout, polling_sleep_duration)
    catch err
        if isa(err, PyCall.PyError)
            error_massage = generate_error_message(err)
            error(error_massage)
        else
            rethrow(err)
        end
    end
end

end
