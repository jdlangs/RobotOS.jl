#Tests of proper type generation

@rosimport geometry_msgs.msg: PoseStamped, Vector3
gentypes()

@test isdefined(:geometry_msgs)
@test isdefined(:std_msgs)
@test isdefined(geometry_msgs.msg, :Point)
@test isdefined(geometry_msgs.msg, :Quaternion)
@test isdefined(geometry_msgs.msg, :Pose)
@test isdefined(geometry_msgs.msg, :PoseStamped)
@test isdefined(geometry_msgs.msg, :Vector3)
@test isdefined(std_msgs.msg, :Header)

posestamp = geometry_msgs.msg.PoseStamped()
@test typeof(posestamp.pose) == geometry_msgs.msg.Pose
@test typeof(posestamp.pose.position) == geometry_msgs.msg.Point
