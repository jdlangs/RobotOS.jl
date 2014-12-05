using Compat

msgdict = @compat Dict{String, Vector{String}}(
    "sensor_msgs" => ["Temperature", "Imu"],
    "geometry_msgs" => ["PoseStamped", "Vector3", "PoseArray"],
    "nav_msgs" => ["Path"],
)
ROS.usetypes(msgdict)
ROS.gentypes()

using ROS.geometry_msgs

ROS.init_node("pub_test")

mcb(msg::Vector3, name) = println(name, ": ", msg)
sub = ROS.Subscriber("pts", Vector3, mcb, ("vector",), queue_size=10)

pub = ROS.Publisher("pts", Vector3, queue_size=10)
rate = ROS.Rate(1.0)

println("Publishing...")
while ! ROS.is_shutdown()
    msg = Vector3(rand(), rand()+1, rand()+2)
    ROS.publish(pub, msg)
    ROS.sleep(rate)
end
println("Done")
