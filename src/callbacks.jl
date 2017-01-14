#This function will run in a new python thread created by rospy.
#No julia allocation allowed.
function _callback_notify(handle::Ptr{Void})
    ccall(:uv_async_send, Cint, (Ptr{Void},), handle)
end

const CB_NOTIFY_PTR = cfunction(_callback_notify, Cint, (Ptr{Void},))

function _callback_async_loop(rosobj, cond)
    @debug("Spinning up callback loop...")
    while ! is_shutdown()
        wait(cond)
        _run_callbacks(rosobj)
    end
    @debug("Exiting callback loop")
end

function _run_callbacks{M}(sub::Subscriber{M})
    while pycall(sub.queue["size"], PyAny) > 0
        msg = pycall(sub.queue["get"], PyObject)
        sub.callback(convert(M, msg), sub.callback_args...)
    end
end

function _run_callbacks{T}(srv::Service{T})
    ReqType = _srv_reqtype(T)

    req = pycall(srv.cb_interface["get_request"], PyObject)
    response = srv.handler(convert(ReqType, req))

    #Python callback is blocking until the response is ready
    pycall(srv.cb_interface["set_response"], PyAny, convert(PyObject, response))
end
