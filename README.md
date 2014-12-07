# ROS
## Overview

This package enables interfacing Julia code with a ROS system. It works by
generating native Julia types for ROS messages, the same as in C++ or Python,
and then wrapping rospy through the PyCall package to get communication through
topics and parameters.

## Type Generation

To use the package, first specify the ROS message types you want to use in your
system as strings "_package_/_message_". All the needed dependencies will be
automatically included. There are three interface functions to do this. For
example:

    using ROS
    usetypes("geometry_msgs/Pose", "nav_msgs/Path")
    usetypes(Dict("sensor_msgs" => ["Imu", "NavSatFix"], "std_msgs" => ["Header"]))
    usepkg("geometry_msgs", "PoseStamped", "Point", "Vector3")

The first `usetypes` takes a variable number of fully qualified message
strings. The second takes a Dict mapping package names to lists of messages,
possibly useful for a large number of messages all at once. The final form,
`usepkg`, takes the package name as the first argument, followed by a variable
number of message names.

Any number of these three functions can be called as needed to specify the
desired message types to generate. When finished, initiate the message
generation with:

    ROS.gentypes()

The new types will be placed in newly created submodules in `ROS`,
corresponding to the packages requested. For example, `"std_msgs/Header" =>
ROS.std_msgs.Header`. After calling `gentypes()` they can be interacted with
just like regular modules with `import` and `using` statements bringing the
generated type names into the local namespace.

    using ROS.nav_msgs
    import ROS.geometry_msgs: Pose, Vector3
    p = Path()
    v = Vector3(1.1,2.2,3.3)

## ROS API

In general, the ROS api functions directly match those provided in rospy, with
few cosmetic differences. The API functions can reviewed here:
[http://wiki.ros.org/rospy/Overview](http://wiki.ros.org/rospy/Overview) The
only large differences are the current lack of services, and custom ROS
exceptions.

### General Functions

- `init_node(name::String; kwargs...)` : Initialize node. Passes keyword
arguments on to rospy directly. (Required)
- `is_shutdown()` : Check for ROS shutdown state.
- `spin()` :  Wait for callbacks until shutdown happens.
- `logdebug`,`loginfo`,`logwarn`,`logerr`,`logfatal` all work as in rospy.

### Time

Native Julia types `ROS.Time` and `ROS.Duration` are defined, both as a
composite of an integral number of seconds and nanoseconds, as in rospy.
Arithmetic and comparison operators are also defined. Related functions are:

- `get_rostime()`, `now()` : Current time as `Time` object.
- `to_sec(time_obj)`, `convert(Float64, time_obj)` : Convert `Time` or
`Duration` object to floating-point number of seconds.
- `to_nsec(time_obj)` : Convert object to integral number of nanoseconds.

### Publishing Messages

Publishing messages is the same as in rospy, except use the `publish` method,
paired with a Publisher object. For example:

    using ROS.geometry_msgs
    pub = ROS.Publisher{PointStamped}("topic", queue_size = 10) #or...
    #pub = ROS.Publisher("topic", PointStamped, queue_size = 10)
    msg = PointStamped()
    msg.header.stamp = ROS.now()
    msg.point.x = 1.1
    ROS.publish(pub, msg)

The keyword arguments in the `Publisher` constructor are passed directly on to
rospy so anything it accepts will be valid.

### Subscribing to a Topic

Subscribing to a topic is same as in rospy. When creating a `Subscriber`, an
optional `callback_args` parameter can be given to forward on whenever the
callback is invoked. Note that it must be passed as a tuple, even if there is
only a single argument. And again, keyword arguments are directly forwarded. An
example:

    using ROS.sensor_msgs
    cb1(msg::Imu, a::String) = println(a,": ",msg.linear_acceleration.x)
    cb2(msg::Imu) = println(msg.angular_velocity.z)
    sub1 = ROS.Subscriber{Imu}("topic", cb1, ("accel",), queue_size = 10) #or...
    #sub1 = ROS.Subscriber("topic", Imu, cb1, ("accel",), queue_size = 10)
    sub2 = ROS.Subscriber{Imu}("topic", cb2, queue_size = 10)
    ROS.spin()

### Parameter Server

`get_param`, `set_param`, `has_param`, and `delete_param` are all implemented
in the `ROS` module with the same syntax as in rospy.

[![Build
Status](https://travis-ci.org/phobon/ROS.jl.svg?branch=master)](https://travis-ci.org/phobon/ROS.jl)
