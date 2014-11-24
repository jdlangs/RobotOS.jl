msgdict = Dict{String, Vector{String}}(
    "sensor_msgs" => ["Temperature", "Imu"],
    "geometry_msgs" => ["PoseStamped", "Vector3", "PoseArray"],
    "nav_msgs" => ["Path"],
)
ROS.genmsgs(msgdict)
ROS.init_node("pub_test")

pub = ROS.Publisher("pts", ROS.geometry_msgs.Vector3)
rate = ROS.Rate(1.0)

while ! ROS.is_shutdown()
    msg = ROS.geometry_msgs.Vector3(1.1, 2.2, rand())
    ROS.publish(pub, msg)
    ROS.sleep(rate)
end
