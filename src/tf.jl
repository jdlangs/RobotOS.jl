export TransformListener, lookupTransform, waitForTransform

struct TransformListener
    o::PyObject
    function TransformListener()
        new(__tf__.TransformListener())
    end
end

function lookupTransform(tl::TransformListener,
                         target_frame::AbstractString,
                         source_frame::AbstractString,
                         pytime::Time)
    time = convert(PyObject, pytime)
    pycall(tl.o.lookupTransform, PyAny, target_frame, source_frame, time)
end

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
