using Test
using PyCall
using RobotOS
RobotOS.debug(true)

#Generally, later tests rely on things defined in previous tests, so the order is important
include("rospy.jl")
include("time.jl")
include("typegeneration.jl")
include("pubsub.jl")
include("services.jl")
