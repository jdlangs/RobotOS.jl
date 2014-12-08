using Compat

msgdict = @compat Dict{String, Vector{String}}(
    "sensor_msgs" => ["Temperature", "Imu"],
    "geometry_msgs" => ["PoseStamped", "Vector3", "PoseArray"],
    "nav_msgs" => ["Path"],
)
RobotOS.usetypes(msgdict)
RobotOS.gentypes()

using RobotOS.geometry_msgs

RobotOS.init_node("pub_test")

mcb(msg::Vector3, name) = println(name, ": ", msg)
sub = RobotOS.Subscriber("pts", Vector3, mcb, ("vector",), queue_size=10)

pub = RobotOS.Publisher("pts", Vector3, queue_size=10)
rate = RobotOS.Rate(1.0)

println("Publishing...")
while ! RobotOS.is_shutdown()
    msg = Vector3(rand(), rand()+1, rand()+2)
    RobotOS.publish(pub, msg)
    RobotOS.sleep(rate)
end
println("Done")
