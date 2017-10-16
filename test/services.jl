#pubsub.jl must be run first

using .std_srvs.srv
using .nav_msgs.srv

#Set up services
const srvcall = ServiceProxy("callme", SetBool)
println("Waiting for 'callme' service...")
wait_for_service("callme")

const flag = Bool[false]
const Nposes = 5

function srv_cb(req::GetPlanRequest)
    println("GetPlan call received")
    @test req.start.pose.position.x ≈ 1.0
    @test req.goal.pose.position.y ≈ 1.0

    resp = GetPlanResponse()
    for i=1:Nposes
        npose = PoseStamped()
        npose.header.stamp = get_rostime()
        npose.pose.position.z = i
        push!(resp.plan.poses, npose)
    end
    flag[1] = true
    return resp
end

const srvlisten = Service("getplan", GetPlan, srv_cb)

println("Calling service...")
srvcall(SetBoolRequest(true))

#Wait for call from echo
println("Waiting for service call from echo..")
while ! (flag[1] || is_shutdown())
    rossleep(Duration(0.1))
end
println("Response sent")

#Check the message replies caught by the geomety_msgs/PoseStamped subscriber in pubsub.jl which
#populates the msgs global variable
if flag[1]
    rossleep(Duration(2.0))
    @test length(msgs) == Nposes
    for i=1:Nposes
        @test msgs[i].pose.position.z ≈ i
    end
end
empty!(msgs)

#Test error handling
@test_throws ErrorException wait_for_service("fake_srv", timeout=1.0)
@test_throws ArgumentError srvcall(SetBoolResponse())
