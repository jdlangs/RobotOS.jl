#This function will run in a new python thread created by rospy.
#No julia allocation allowed.
function _callback_notify(handle::Ptr{Cvoid})
    ccall(:uv_async_send, Cint, (Ptr{Cvoid},), handle)
end

#The pointer to the compiled notify function. This can't be precompiled so it gets initialized in
#the module __init__ function.
const CB_NOTIFY_PTR = Ref{Ptr{Cvoid}}()

function _callback_async_loop(rosobj, cond)
    @debug("Spinning up callback loop...")
    while ! is_shutdown()
        wait(cond)
        _run_callbacks(rosobj)
    end
    @debug("Exiting callback loop")
end

function _run_callbacks(sub::Subscriber{M}) where M
    while pycall(sub.queue["size"], PyAny) > 0
        msg = pycall(sub.queue["get"], PyObject)
        sub.callback(convert(M, msg), sub.callback_args...)
    end
end

function _run_callbacks(srv::Service{T}) where T
    ReqType = _srv_reqtype(T)

    req = pycall(srv.cb_interface["get_request"], PyObject)
    response = srv.handler(convert(ReqType, req))

    #Python callback is blocking until the response is ready
    pycall(srv.cb_interface["set_response"], PyAny, convert(PyObject, response))
end
