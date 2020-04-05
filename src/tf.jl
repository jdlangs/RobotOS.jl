export TransformListener, lookupTransform

struct TransformListener
    o::PyObject
    function TransformListener()
        new(__tf__.TransformListener())
    end
end

function lookupTransform(tl::TransformListener,
                         target_frame::AbstractString,
                         source_frame::AbstractString,
                         pyt::Time)
    t = convert(PyObject, pyt)
    tl.o.lookupTransform(target_frame, source_frame, t)
    pycall(tl.o.lookupTransform, PyAny, target_frame, source_frame, t)
end
