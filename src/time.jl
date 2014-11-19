type Time
    secs::Int32
    nsecs::Int32
end
Time() = Time(0,0)

type Duration
    secs::Int32
    nsecs::Int32
end

type Rate
    r::PyObject
end

type Timer
    t::PyObject
end

type TimerEvent
    last_expected::Time
    last_real::Time
    current_expected::Time
    current_real::Time
    last_duration::Duration
end

function now()
    t = pycall(rospy.pymember("Time")["now"], PyObject)
    Time(t[:secs], t[:nsecs])
end
