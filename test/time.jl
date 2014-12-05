t1 = Time(1,0)
t2 = Time(0, 999_999_999)
t3 = Time(2, 500_000_000)
d1 = Duration(0, 999_999_999)
d2 = Duration(1, 500_000_000)
d3 = Duration(0, 1)

@test t1 == Time(1,0)
@test t1 != t2
@test t1 > t2

@test d1 == Duration(0, 999_999_999)
@test d1 != d2
@test d1 < d2

@test t1 + d2 == t3
@test t2 + d3 == t1
@test t1 - t2 == d3
@test t1 - d3 == t2
@test d1 + d2 + d3 == Duration(t3.secs, t3.nsecs)
@test d2 - d1 + d3 == Duration(0, 500_000_000)
