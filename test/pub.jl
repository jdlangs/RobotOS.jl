using Compat

msgdict = @compat Dict{String, Vector{String}}(
    "sensor_msgs" => ["Temperature", "Imu"],
    "geometry_msgs" => ["PoseStamped", "Vector3", "PoseArray"],
    "nav_msgs" => ["Path"],
)
ROS.usetypes(msgdict)
ROS.gentypes()
ROS.init_node("pub_test")

pub = ROS.Publisher("pts", ROS.geometry_msgs.Vector3, queue_size=10)
rate = ROS.Rate(1.0)

while ! ROS.is_shutdown()
    msg = ROS.geometry_msgs.Vector3(1.1, 2.2, rand())
    ROS.publish(pub, msg)
    ROS.sleep(rate)
end
