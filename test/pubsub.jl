#Test publish and subscribe ability
#works alongside echonode.py
#typegeneration.jl must be run first

using geometry_msgs.msg

const Nmsgs = 10
const rate = 20. #Hz
const msgs = PoseStamped[]
const refs = Array(Vector3, Nmsgs)
const t0 = to_nsec(get_rostime())

for i=1:Nmsgs
    refs[i] = Vector3(rand(3)...)
end

function pose_cb(msg::PoseStamped, msgs::Vector{PoseStamped})
    mtime = to_nsec(msg.header.stamp) - t0
    println("Message received, time: ",mtime," nanoseconds")
    if msg.header.stamp.secs > 1.
        push!(msgs, msg)
        println("Got message #",msg.header.seq)
    end
end
pose_cb(PoseStamped(), msgs)

const ros_pub = Publisher{Vector3}("vectors", queue_size = 10)
const ros_sub = Subscriber{PoseStamped}("poses", pose_cb, (msgs,), queue_size = 10)

#First message doesn't go out for some reason
publish(ros_pub, Vector3(1.1,2.2,3.3))
rossleep(Duration(1.0))

const r = Rate(20.0)
for i=1:Nmsgs
    publish(ros_pub, refs[i])
    rossleep(r)
end
rossleep(Duration(1.0))
println("Received ",length(msgs)," / ",Nmsgs)

@test length(msgs) == Nmsgs
for i=1:Nmsgs
    @test_approx_eq msgs[i].pose.position.x refs[i].x
    @test_approx_eq msgs[i].pose.position.y refs[i].y
    @test_approx_eq msgs[i].pose.position.z refs[i].z
end
