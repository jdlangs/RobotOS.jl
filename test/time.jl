t1 = Time(1,0)
t2 = Time(0, 999_999_999)
t3 = Time(2, 500_000_000)
d1 = Duration(0, 999_999_999)
d2 = Duration(1, 500_000_000)
d3 = Duration(0, 1)

@test t1 == Time(1,0)
@test t1 != t2
@test t1 > t2
@test t1 >= t2
@test t2 < t1
@test t2 <= t1

@test d1 == Duration(0, 999_999_999)
@test d1 != d2
@test d1 < d2
@test d1 <= d2
@test d2 > d1
@test d2 >= d1

@test t1 + d2 == t3
@test t2 + d3 == t1
@test t1 - t2 == d3
@test t1 - d3 == t2
@test d1 + d2 + d3 == Duration(t3.secs, t3.nsecs)
@test d2 - d1 - d3 == Duration(0, 500_000_000)

@test d2*2 == Duration(3,0)
@test 3.0*d2 == Duration(4,500_000_000)

tt = Time(2,0)
@test tt == Time(2.0)
@test convert(Float64,tt) == 2.0
@test to_sec(tt) == 2.0
@test to_nsec(tt) == 2_000_000_000

dt = Duration(3,0)
@test dt == Duration(3.0)
@test convert(Float64,dt) == 3.0
@test to_sec(dt) == 3.0
@test to_nsec(dt) == 3_000_000_000

@test dt + tt == Time(5.0)
@test dt + dt == Duration(6.0)

#PyObject stuff
ptt = convert(PyCall.PyObject, tt)
@test ptt[:secs] == 2
@test ptt[:nsecs] == 0
ptt[:nsecs] = 101
tt2 = convert(Time, ptt)
@test to_nsec(tt2) == 2_000_000_101

pdt = convert(PyCall.PyObject, dt)
@test pdt[:secs] == 3
@test pdt[:nsecs] == 0
pdt[:nsecs] = 202
dt2 = convert(Duration, pdt)
@test to_nsec(dt2) == 3_000_000_202

#rostime and sleeping
t1 = get_rostime()
rossleep(0.5)
t2 = get_rostime()
@test t2 - t1 >= Duration(0.4)

rte = Rate(Duration(0.5))
rossleep(rte)
t1 = RobotOS.now()
rossleep(rte)
t2 = RobotOS.now()
rossleep(rte)
t3 = RobotOS.now()
@test t2 - t1 >= Duration(0.4)
@test t3 - t2 >= Duration(0.4)
@test t3 - t1 >= Duration(0.8)

t1 = get_rostime()
RobotOS.sleep(Duration(0.5))
t2 = get_rostime()
@test t2 - t1 >= Duration(0.4)

RobotOS.sleep(rte)
t1 = get_rostime()
RobotOS.sleep(rte)
t2 = get_rostime()
@test t2 - t1 >= Duration(0.4)
