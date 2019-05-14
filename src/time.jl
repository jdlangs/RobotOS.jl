#All time related types and functions

import Base: convert, isless, sleep, +, -, *, ==
export Time, Duration, Rate, to_sec, to_nsec, get_rostime, rossleep

#Time type definitions
abstract type AbstractTime end

"""
    Time(secs, nsecs), Time(), Time(t::Real)

Object representing an absolute time from a fixed past reference point at nanosecond precision.

Basic arithmetic can be performed on combinations of `Time` and `Duration` objects that make sense.
For example, if `t::Time` and `d::Duration`, `t+d` will be a `Time`, `d+d` a `Duration`, `t-d` a
`Time`, `d-d` a `Duration`, and `t-t` a `Duration`.
"""
struct Time <: AbstractTime
    secs::Int32
    nsecs::Int32
    function Time(s::Real,n::Real)
        cs, cns = _canonical_time(s,n)
        new(cs, cns)
    end
end
Time() = Time(0,0)
Time(t::Real) = Time(t,0)

"""
    Duration(secs, nsecs), Duration(), Duration(t::Real)

Object representing a relative period of time at nanosecond precision.

Basic arithmetic can be performed on combinations of `Time` and `Duration` objects that make sense.
For example, if `t::Time` and `d::Duration`, `t+d` will be a `Time`, `d+d` a `Duration`, `t-d` a
`Time`, `d-d` a `Duration`, and `t-t` a `Duration`.
"""
struct Duration <: AbstractTime
    secs::Int32
    nsecs::Int32
    function Duration(s::Real,n::Real)
        cs, cns = _canonical_time(s,n)
        new(cs, cns)
    end
end
Duration() = Duration(0,0)
Duration(t::Real) = Duration(t,0)

#Enforce 0 <= nsecs < 1e9
function _canonical_time(secs, nsecs)
    nsec_conv = convert(Int32, 1_000_000_000)
    secs32  = floor(Int32, secs)
    nsecs32 = floor(Int32, mod(secs,1)*1e9 + nsecs)

    addsecs = div(nsecs32, nsec_conv)
    crnsecs = rem(nsecs32, nsec_conv)
    if crnsecs < 0
        addsecs -= one(Int32)
        crnsecs += nsec_conv
    end
    (secs32 + addsecs, crnsecs)
end

+(t1::Time,     t2::Duration) = Time(    t1.secs+t2.secs, t1.nsecs+t2.nsecs)
+(t1::Duration, t2::Time)     = Time(    t1.secs+t2.secs, t1.nsecs+t2.nsecs)
+(t1::Duration, t2::Duration) = Duration(t1.secs+t2.secs, t1.nsecs+t2.nsecs)
-(t1::Time,     t2::Duration) = Time(    t1.secs-t2.secs, t1.nsecs-t2.nsecs)
-(t1::Duration, t2::Duration) = Duration(t1.secs-t2.secs, t1.nsecs-t2.nsecs)
-(t1::Time,     t2::Time)     = Duration(t1.secs-t2.secs, t1.nsecs-t2.nsecs)
*(td::Duration, tf::Real)     = Duration(tf*td.secs     , tf*td.nsecs)
*(tf::Real, td::Duration)     = Duration(tf*td.secs     , tf*td.nsecs)

#PyObject conversions
convert(::Type{Time},     o::PyObject) = Time(    o.secs,o.nsecs)
convert(::Type{Duration}, o::PyObject) = Duration(o.secs,o.nsecs)
convert(::Type{PyObject}, t::Time)     = __rospy__.Time(    t.secs,t.nsecs)
convert(::Type{PyObject}, t::Duration) = __rospy__.Duration(t.secs,t.nsecs)

#Real number conversions
"""
    to_sec(t)

Return the value of a ROS time object in absolute seconds (with nanosecond precision)
"""
to_sec(t::T) where {T <: AbstractTime} = t.secs + 1e-9*t.nsecs

"""
    to_nsec(t)

Return the value of a ROS time object in nanoseconds as an integer.
"""
to_nsec(t::T) where {T <: AbstractTime} = 1_000_000_000*t.secs + t.nsecs
convert(::Type{Float64}, t::T) where {T <: AbstractTime} = to_sec(t)

#Comparisons
==(t1::T, t2::T) where {T <: AbstractTime} = (t1.secs == t2.secs) && (t1.nsecs == t2.nsecs)
isless(t1::T, t2::T) where {T <: AbstractTime} = to_nsec(t1) < to_nsec(t2)

"""
    Rate(hz::Real), Rate(d::Duration)

Used to allow a loop to run at a fixed rate. Construct with a frequency or `Duration` and use with
`rossleep` or `sleep`. The rate object will record execution time of other work in the loop and
modify the sleep time to compensate, keeping the loop rate as consistent as possible.
"""
mutable struct Rate
    duration::Duration
    last_time::Time
end
Rate(d::Duration) = Rate(d, get_rostime())
Rate(hz::Real) = Rate(Duration(1.0/hz), get_rostime())

"""
    get_rostime()

Return the current ROS time as a `Time` object.
"""
function get_rostime()
    t = try
        __rospy__.get_rostime()
    catch ex
        error(pycall(pybuiltin("str"), PyAny, ex.val))
    end
    convert(Time, t)
end

"""
    RobotOS.now()

Return the current ROS time as a `Time` object.
"""
now() = get_rostime()

"""
    rossleep(t)

Sleep and process callbacks for a number of seconds implied by the type and value of `t`, which may
be a real-value, a `Duration` object, or a `Rate` object.
"""
function rossleep(td::Duration)
    #Busy sleep loop needed to allow both julia and python async activity
    tnsecs = to_nsec(td)
    t0 = time_ns()
    while time_ns()-t0 < tnsecs
        yield()                   #Allow julia callback loops to run
        __rospy__.sleep(0.001)  #Allow rospy comm threads to run
    end
end
rossleep(t::Real) = rossleep(Duration(t))

function rossleep(r::Rate)
    ctime = get_rostime()
    if r.last_time > ctime
        r.last_time = ctime
    end
    elapsed = ctime - r.last_time
    rossleep(r.duration - elapsed)
    r.last_time += r.duration

    if ctime - r.last_time > r.duration*2
        r.last_time = ctime
    end
end

"""
    sleep(t::Duration), sleep(t::Rate)

Call `rossleep` with a `Duration` or `Rate` object. Use `rossleep` to specify sleep time directly.
"""
sleep(t::Duration) = rossleep(t)
sleep(t::Rate) = rossleep(t)
