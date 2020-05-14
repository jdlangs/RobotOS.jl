using RobotOS
using RobotOS.TF

init_node("broadcaster", anonymous=true) # delete
tb = TransformBroadcaster()
tf_ref = Transform([0, 0, 0.], [0, 0, 0, 1.])

r = Rate(20.0)
while true
    sendTransform(tb, tf_ref, RobotOS.now(), "child_link", "parent_link")
    rossleep(r)
end
