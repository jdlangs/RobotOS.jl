#All time related types and functions

import Base: convert, isless, sleep, +, -, *, ==
export Time, Duration, Rate, to_sec, to_nsec, get_rostime, rossleep

#Time type definitions
abstract TVal

immutable Time <: TVal
    secs::Int32
    nsecs::Int32
    function Time(s::Real,n::Real)
        cs, cns = _canonical_time(s,n)
        new(cs, cns)
    end
end
Time() = Time(0,0)
Time(t::Real) = Time(t,0)

immutable Duration <: TVal
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

#Temporal arithmetic
+(t1::Time,     t2::Duration) = Time(    t1.secs+t2.secs, t1.nsecs+t2.nsecs)
+(t1::Duration, t2::Time)     = Time(    t1.secs+t2.secs, t1.nsecs+t2.nsecs)
+(t1::Duration, t2::Duration) = Duration(t1.secs+t2.secs, t1.nsecs+t2.nsecs)
-(t1::Time,     t2::Duration) = Time(    t1.secs-t2.secs, t1.nsecs-t2.nsecs)
-(t1::Duration, t2::Duration) = Duration(t1.secs-t2.secs, t1.nsecs-t2.nsecs)
-(t1::Time,     t2::Time)     = Duration(t1.secs-t2.secs, t1.nsecs-t2.nsecs)
*(td::Duration, tf::Real)     = Duration(tf*td.secs     , tf*td.nsecs)
*(tf::Real, td::Duration)     = Duration(tf*td.secs     , tf*td.nsecs)

#PyObject conversions
convert(::Type{Time},     o::PyObject) = Time(    o[:secs],o[:nsecs])
convert(::Type{Duration}, o::PyObject) = Duration(o[:secs],o[:nsecs])
convert(::Type{PyObject}, t::Time)     = __rospy__[:Time](    t.secs,t.nsecs)
convert(::Type{PyObject}, t::Duration) = __rospy__[:Duration](t.secs,t.nsecs)

#Real number conversions
to_sec{T<:TVal}(t::T) = t.secs + 1e-9*t.nsecs
to_nsec{T<:TVal}(t::T) = 1_000_000_000*t.secs + t.nsecs
convert{T<:TVal}(::Type{Float64}, t::T) = to_sec(t)

#Comparisons
=={T<:TVal}(t1::T, t2::T)     = (t1.secs == t2.secs) && (t1.nsecs == t2.nsecs)
isless{T<:TVal}(t1::T, t2::T) = to_nsec(t1) < to_nsec(t2)

function get_rostime()
    t = try
        __rospy__[:get_rostime]()
    catch ex
        error(pycall(pybuiltin("str"), PyAny, ex.val))
    end
    convert(Time, t)
end
now() = get_rostime()

function rossleep(td::Duration)
    #Busy sleep loop needed to allow both julia and python async activity
    tnsecs = to_nsec(td)
    t0 = time_ns()
    while time_ns()-t0 < tnsecs
        yield()                   #Allow julia callback loops to run
        __rospy__[:sleep](0.001)  #Allow rospy comm threads to run
    end
end
rossleep(t::Real) = rossleep(Duration(t))

sleep(t::Duration) = rossleep(t)

type Rate
    duration::Duration
    last_time::Time
end
Rate(d::Duration) = Rate(d, get_rostime())
Rate(hz::Real) = Rate(Duration(1.0/hz), get_rostime())

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
sleep(t::Rate) = rossleep(t)
