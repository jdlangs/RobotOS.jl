import Base.convert

abstract TVal

#Time definition
type Time <: TVal
    secs::Int32
    nsecs::Int32
end
Time() = Time(0,0)
Time(t::FloatingPoint) =
    Time(ifloor(Int32, t), iround(Int32, mod(t,1)*1e9))

#PyObject conversions
convert(::Type{Time}, o::PyObject) = Time(o[:secs],o[:nsecs])
convert(::Type{PyObject}, t::Time) = begin
    pyt = __rospy__.Time()
    pyt["secs"] = t.secs
    pyt["nsecs"] = t.nsecs
    pyt
end

#Duration
type Duration <: TVal
    secs::Int32
    nsecs::Int32
end
Duration() = Duration(0,0)
Duration(t::FloatingPoint) =
    Duration(ifloor(Int32,t), iround(Int32,mod(t,1)*1e9))

convert(::Type{Duration}, o::PyObject) = Duration(o[:secs], o[:nsecs])
convert(::Type{PyObject}, t::Duration) = begin
    pyt = __rospy__.Duration()
    pyt["secs"] = t.secs
    pyt["nsecs"] = t.nsecs
    pyt
end

#Temporal arithmetic
+(t1::Time, t2::Duration) =
    _canon_time(Time,     t1.secs + t2.secs, t1.nsecs + t2.nsecs)
+(t1::Duration, t2::Time) =
    _canon_time(Time,     t1.secs + t2.secs, t1.nsecs + t2.nsecs)
+(t1::Duration, t2::Duration) =
    _canon_time(Duration, t1.secs + t2.secs, t1.nsecs + t2.nsecs)
-(t1::Time, t2::Duration) =
    _canon_time(Time,     t1.secs - t2.secs, t1.nsecs - t2.nsecs)
-(t1::Duration, t2::Duration) =
    _canon_time(Duration, t1.secs - t2.secs, t1.nsecs - t2.nsecs)
-(t1::Time, t2::Time) =
    _canon_time(Duration, t1.secs - t2.secs, t1.nsecs - t2.nsecs)
function _canon_time{T<:TVal}(typ::Type{T}, secs::Integer, nsecs::Integer)
    nt =
        if nsecs >= 1000000000
            T(secs + 1, nsecs - 1000000000)
        elseif nsecs < 0
            T(secs - 1, nsecs + 1000000000)
        else
            T(secs, nsecs)
        end
    nt
end

type Rate
    o::PyObject
end
Rate(hz::FloatingPoint) = Rate(__rospy__.Rate(hz))

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

function get_rostime()
    t = try
        __rospy__.get_rostime()
    catch ex
        error(pycall(pybuiltin("str"), PyAny, ex.val))
    end
    convert(Time, t)
end
now() = get_rostime()

to_sec{T<:TVal}(t::T) = float64(t.secs) + 1e-9*float64(t.nsecs)
to_nsec{T<:TVal}(t::T) = 1000000000*t.secs + t.nsecs

sleep(t::Duration) = __rospy__.sleep(convert(PyObject, t))
sleep(t::FloatingPoint) = __rospy__.sleep(t)
sleep(r::Rate) = pycall(r.o["sleep"], PyAny)
