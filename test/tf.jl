#Test if tf listner works correctly
using RobotOS.TF

tl = TransformListener()
waitForTransform(tl, "parent_link", "child_link", Time(), Duration(1.0))
transform = lookupTransform(tl, "parent_link", "child_link", Time())
println("Received Transform: ", transform)
tf_correct = Transform([0, 0, 0.], [0, 0, 0, 1.])
@test transform == tf_correct
