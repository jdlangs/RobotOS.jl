#Test publish and subscribe ability
#works alongside republish.py

using RobotOS.geometry_msgs
init_node("jltest", anonymous=true)

const Nmsgs = 10
const rate = 20. #Hz
const msgs = PoseStamped[]
const refs = Array(Vector3, Nmsgs)

for i=1:Nmsgs
    refs[i] = Vector3(rand(3)...)
end

function pose_cb(msg::PoseStamped, msgs::Vector{PoseStamped})
    if msg.header.stamp.secs > 1.
        push!(msgs, msg)
    end
end
pose_cb(PoseStamped(), msgs)

ros_pub = Publisher{Vector3}("vectors", queue_size = 10)
ros_sub = Subscriber{PoseStamped}("poses", pose_cb, (msgs,), queue_size = 10)

#First message doesn't go out for some reason
publish(ros_pub, Vector3(1.1,2.2,3.3))
RobotOS.sleep(1.0)

r = Rate(20.0)
for i=1:Nmsgs
    publish(ros_pub, refs[i])
    RobotOS.sleep(r)
end
RobotOS.sleep(1.0)

@test length(msgs) == Nmsgs
for i=1:Nmsgs
    @test_approx_eq msgs[i].pose.position.x refs[i].x
    @test_approx_eq msgs[i].pose.position.y refs[i].y
    @test_approx_eq msgs[i].pose.position.z refs[i].z
end
