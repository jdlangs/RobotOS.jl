#Test basic rospy interactions
init_node("jltest", anonymous=true)

#Parameters
@test length(RobotOS.get_param_names()) > 0
@test has_param("rosdistro")
@test chomp(get_param("rosdistro")) in ["hydro", "indigo", "jade", "kinetic", "lunar", "melodic"]
@test ! has_param("some_param")
@test_throws KeyError get_param("some_param")
@test_throws KeyError delete_param("some_param")
@test get_param("some_param", 1.1) == 1.1
@test get_param("some_param", "some_val") == "some_val"
set_param("some_param", "val")
@test get_param("some_param", 1.1) == "val"
delete_param("some_param")
@test ! has_param("some_param")

#Really just running this stuff for coverage

#Logging
logdebug("testing: %s", "debug")
loginfo("testing: %s", "info")
logwarn("testing: %s", "warn")
logerr("testing: %s", "err")
logfatal("testing: %s", "fatal")
@test ! is_shutdown()

#Generic stuff
@test startswith(RobotOS.get_name()[2:end], "jltest")
@test RobotOS.get_namespace() == "/"
RobotOS.get_node_uri()
RobotOS.get_caller_id()
RobotOS.get_published_topics()
RobotOS.get_ros_root()
