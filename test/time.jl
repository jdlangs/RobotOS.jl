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

tt = Time()
tt.secs = 2
@test tt == Time(2.0)
@test convert(Float64,tt) == 2.0
@test to_sec(tt) == 2.0
@test to_nsec(tt) == 2_000_000_000

dt = Duration()
dt.secs = 3
@test dt == Duration(3.0)
@test convert(Float64,dt) == 3.0
@test to_sec(dt) == 3.0
@test to_nsec(dt) == 3_000_000_000

@test tt + dt == Time(5.0)
@test dt + dt == Duration(6.0)

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
RobotOS.sleep(0.5)
t2 = get_rostime()
@test t2 - t1 >= Duration(0.4)

RobotOS.sleep(rte)
t1 = get_rostime()
RobotOS.sleep(rte)
t2 = get_rostime()
@test t2 - t1 >= Duration(0.4)
