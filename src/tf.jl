export TransformListener, lookupTransform, waitForTransform

"""
    TransformListener()
Create a transform listener object 
"""
struct TransformListener
    o::PyObject
    function TransformListener()
        new(__tf__.TransformListener())
    end
end

"""
    lookupTransform(tf_listener_obj, target, source, time) 
Return tuple of (position, quaternion)
"""
function lookupTransform(tl::TransformListener,
                         target_frame::AbstractString,
                         source_frame::AbstractString,
                         pytime::Time)
    time = convert(PyObject, pytime)
    pycall(tl.o.lookupTransform, PyAny, target_frame, source_frame, time)
end

"""
    waitForTransform(tf_listener_obj, target, source, time, timeout, pypolling_sleep_duration) 
Waits for the given transformation to become available.  
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
    pycall(tl.o.waitForTransform, PyAny, target_frame, source_frame,
           time, timeout, polling_sleep_duration)
end
