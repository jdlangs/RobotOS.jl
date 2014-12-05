import Base: convert, isless

#Time type definitions
abstract TVal

type Time <: TVal
    secs::Int32
    nsecs::Int32
end
Time() = Time(0,0)
Time(t::FloatingPoint) =
    Time(floor(Int32, t), round(Int32, mod(t,1)*1e9))

type Duration <: TVal
    secs::Int32
    nsecs::Int32
end
Duration() = Duration(0,0)
Duration(t::FloatingPoint) =
    Duration(floor(Int32,t), round(Int32, mod(t,1)*1e9))

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

#Enforce 0 <= nsecs < 1e9
function _canon_time{T<:TVal}(::Type{T}, secs::Integer, nsecs::Integer)
    nsec_conv = int32(1_000_000_000)
    dsecs, rnsecs  = convert((Int32, Int32), divrem(nsecs, nsec_conv))
    if rnsecs < 0
        dsecs = dsecs - one(Int32)
        rnsecs = rnsecs + nsec_conv
    end
    T(secs + dsecs, rnsecs)
end

#PyObject conversions
convert(::Type{Time},     o::PyObject) = Time    (o[:secs],o[:nsecs])
convert(::Type{Duration}, o::PyObject) = Duration(o[:secs],o[:nsecs])
convert(::Type{PyObject}, t::Time)     = __rospy__.Time    (t.secs,t.nsecs)
convert(::Type{PyObject}, t::Duration) = __rospy__.Duration(t.secs,t.nsecs)

#Real number conversions
to_sec{T<:TVal}(t::T) = float64(t.secs) + 1e-9*float64(t.nsecs)
to_nsec{T<:TVal}(t::T) = 1_000_000_000*t.secs + t.nsecs
convert{T<:TVal}(::Type{Float64}, t::T) = to_sec(t)

#Comparisons
=={T<:TVal}(t1::T, t2::T)     = (t1.secs == t2.secs) && (t1.nsecs == t2.nsecs)
isless{T<:TVal}(t1::T, t2::T) = to_nsec(t1) < to_nsec(t2)

#----------------------------
#Extra time-related utilities
#----------------------------

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

sleep(t::Duration) = __rospy__.sleep(convert(PyObject, t))
sleep(t::FloatingPoint) = __rospy__.sleep(t)
sleep(r::Rate) = pycall(r.o["sleep"], PyAny)
