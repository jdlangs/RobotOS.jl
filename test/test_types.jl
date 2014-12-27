#Tests of proper type generation

usepkg("geometry_msgs", "PoseStamped", "Vector3")
gentypes()

@test isdefined(RobotOS, :geometry_msgs)
@test isdefined(RobotOS, :std_msgs)
@test isdefined(RobotOS.geometry_msgs, :Point)
@test isdefined(RobotOS.geometry_msgs, :Quaternion)
@test isdefined(RobotOS.geometry_msgs, :Pose)
@test isdefined(RobotOS.geometry_msgs, :PoseStamped)
@test isdefined(RobotOS.geometry_msgs, :Vector3)
@test isdefined(RobotOS.std_msgs, :Header)

posestamp = RobotOS.geometry_msgs.PoseStamped()
@test typeof(posestamp.pose) == RobotOS.geometry_msgs.Pose
@test typeof(posestamp.pose.position) == RobotOS.geometry_msgs.Point
