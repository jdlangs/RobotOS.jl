#Test if tf listner works correctly

tl = TransformListener()
waitForTransform(tl, "parent_link", "child_link", Time(), Duration(1.0))
transform = lookupTransform(tl, "parent_link", "child_link", Time())
@test transform == ([0, 0, 0.], [0, 0, 0, 1.])
                   
