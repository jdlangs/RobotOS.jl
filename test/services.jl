#typegeneration.jl and pubsub.jl must be run first

using std_srvs.srv
using nav_msgs.srv

const flag = Bool[false]
const Nposes = 5
empty!(msgs)

#Set up services
function srv_cb(req::GetPlanRequest)
    println("GetPlan call received")
    @test_approx_eq(req.start.pose.position.x, 1.0)
    @test_approx_eq(req.goal.pose.position.y, 1.0)

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

const srvcall = ServiceProxy{Empty}("callme")
const srvlisten = Service{GetPlan}("getplan", srv_cb)

#Call echo's Empty service
println("Waiting for 'callme' service...")
wait_for_service("callme")
println("Calling service now")
call(srvcall, EmptyRequest())

#Wait for call from echo
println("Waiting for service call from echo..")
while ! (flag[1] || is_shutdown())
    rossleep(Duration(0.1))
end
println("Response sent")

#Listen for message replies caught by the geomety_msgs/PoseStamped subscriber
#in pubsub.jl which populates the msgs global variable
if flag[1]
    rossleep(Duration(2.0))
    @test length(msgs) == Nposes
    for i=1:Nposes
        @test_approx_eq(msgs[i].pose.position.z, i)
    end
end
