#Test if tf listner works correctly

# NOTE delte before push
using RobotOS # delete
using Test #delete
using RobotOS.TF

init_node("jltest", anonymous=true) # delete
sleep(2)

tl = TransformListener()
waitForTransform(tl, "parent_link", "child_link", Time(), Duration(1.0))
transform = lookupTransform(tl, "parent_link", "child_link", Time())
tf_correct = Transform([0, 0, 0.], [0, 0, 0, 1.])
@test transform == tf_correct
