#Tests of proper type generation
using PyCall

@rosimport geometry_msgs.msg: PoseStamped, Vector3
@rosimport visualization_msgs.msg: Marker
@rosimport std_srvs.srv: Empty, SetBool
@rosimport nav_msgs.srv.GetPlan
@rosimport std_msgs.msg: Empty
@rosimport std_msgs.msg: Float64, String

@test_throws ErrorException @rosimport fake_msgs.msg.FakeMsg
@test_throws ErrorException @rosimport std_msgs.msg.FakeMsg
@test_throws ErrorException @rosimport nav_msgs.srv.GetPlanRequest
rostypegen()

@test isdefined(Main, :geometry_msgs)
@test isdefined(Main, :std_msgs)
@test isdefined(Main, :nav_msgs)
@test isdefined(geometry_msgs.msg, :Point)
@test isdefined(geometry_msgs.msg, :Quaternion)
@test isdefined(geometry_msgs.msg, :Pose)
@test isdefined(geometry_msgs.msg, :PoseStamped)
@test isdefined(geometry_msgs.msg, :Vector3)
@test isdefined(std_msgs.msg, :Header)
@test isdefined(std_msgs.msg, :Empty)
@test isdefined(nav_msgs.msg, :Path)
@test isdefined(nav_msgs.srv, :GetPlan)
@test isdefined(nav_msgs.srv, :GetPlanRequest)
@test isdefined(nav_msgs.srv, :GetPlanResponse)

#type generation in a non-Main module
module TestModule
    using RobotOS
    @rosimport std_msgs.msg: Float32
    rostypegen(@__MODULE__)
end
@test !isdefined(std_msgs.msg, :Float32Msg)
@test isdefined(TestModule, :std_msgs)
@test isdefined(TestModule.std_msgs.msg, :Float32Msg)

#message creation
posestamp = geometry_msgs.msg.PoseStamped()
@test typeof(posestamp.pose) == geometry_msgs.msg.Pose
@test typeof(posestamp.pose.position) == geometry_msgs.msg.Point

#service creation
boolreq = std_srvs.srv.SetBoolRequest()
boolresp = std_srvs.srv.SetBoolResponse(true, "message")
planreq = nav_msgs.srv.GetPlanRequest()
planresp = nav_msgs.srv.GetPlanResponse()
@test typeof(planreq) == nav_msgs.srv.GetPlanRequest
@test typeof(planresp) == nav_msgs.srv.GetPlanResponse

#convert to/from PyObject
posestamp.pose.position = geometry_msgs.msg.Point(1,2,3)
pypose = convert(PyObject, posestamp)
@test pypose[:pose][:position][:x] == 1.
@test pypose[:pose][:position][:y] == 2.
@test pypose[:pose][:position][:z] == 3.
pypose2 = PyObject(posestamp)
@test pypose2[:pose][:position][:x] == 1.
@test pypose2[:pose][:position][:y] == 2.
@test pypose2[:pose][:position][:z] == 3.
pose2 = convert(geometry_msgs.msg.PoseStamped, pypose)
@test pose2.pose.position.x == 1.
@test pose2.pose.position.y == 2.
@test pose2.pose.position.z == 3.
@test_throws InexactError convert(geometry_msgs.msg.Pose, pypose)

#access message enum
@test visualization_msgs.msg.Marker[:CUBE] == 1

#Proper array handling
path = nav_msgs.msg.Path()
@test typeof(path.poses) == Array{geometry_msgs.msg.PoseStamped,1}
push!(path.poses, posestamp)

pypath = convert(PyObject, path)
path2 = convert(nav_msgs.msg.Path, pypath)
@test typeof(path.poses) == Array{geometry_msgs.msg.PoseStamped,1}
@test path2.poses[1].pose.position.x == 1.
@test path2.poses[1].pose.position.y == 2.
@test path2.poses[1].pose.position.z == 3.

#Issue #6 - Empty message
emptymsg = std_msgs.msg.Empty()
@test length(fieldnames(typeof(emptymsg))) == 0

#Issue #7/8 - Renaming conflicting message types
@test isdefined(std_msgs.msg, :Float64Msg)
@test isdefined(std_msgs.msg, :StringMsg)
@test Publisher{std_msgs.msg.Float64Msg}("x", queue_size=10) != nothing
@test Subscriber{std_msgs.msg.Float64Msg}("x", x->x, queue_size=10) != nothing
