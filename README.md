# RobotOS.jl

[![Build Status](https://travis-ci.org/phobon/RobotOS.jl.svg?branch=master)](https://travis-ci.org/phobon/RobotOS.jl)

## Overview

### Description

This package enables interfacing Julia code with a ROS ([Robot Operating
System](http://wiki.ros.org)) system. It works by generating native Julia types for
ROS messages, the same as in C++ or Python, and then wrapping rospy through the
PyCall package to get communication through topics and parameters.

### Installation

    Pkg.add("RobotOS")
    using RobotOS

### Contributing

The package will hopefully continue to undergo substantial improvement. Please
feel free to submit either an issue or pull request through github if you want
to fix something or suggest a needed improvment, even if it's just to add an
extra sentence in this README.

Once the package gets tested somewhat in the wild, I'll plan to move it
directly to version 1.0.

## Usage: Type Generation

ROS message types are brought into your program with the `@rosimport` macro
which specifies a package and one or more types. The three valid forms can be
seen here:

    @rosimport std_msgs.msg.Header
    @rosimport geometry_msgs.msg: PoseStamped
    @rosimport sensor_msgs.msg: Imu, Image

`@rosimport` will bring in the python modules for the requested type and all
its dependencies but the native Julia types are not created yet since any
inter-module dependencies have to be resolved first. After the final
`@rosimport` call, initiate the message generation with:

    gentypes()

The new types will be placed in newly created modules in `Main`, corresponding
to the packages requested. For example, `"std_msgs/Header" =>
std_msgs.msg.Header`. After calling `gentypes()` they can be interacted with
just like regular modules with `import` and `using` statements bringing the
generated type names into the local namespace.

    using nav_msgs.msg
    import geometry_msgs.msg: Pose, Vector3
    p = Path()
    v = Vector3(1.1,2.2,3.3)

An additional function, `cleartypes()`, resets the type generation process,
possibly useful for development in the REPL. When invoked, new `@rosimport`
calls will be needed to generate the same or different types, and previously
generated modules will be overwritten after `gentypes()` is called again.  Keep
in mind that names cannot be cleared once defined so if a module is not
regenerated, the first version will remain.

## Usage: ROS API

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

Native Julia types `Time` and `Duration` are defined, both as a composite of an
integral number of seconds and nanoseconds, as in rospy.  Arithmetic and
comparison operators are also defined. A `Rate` type is defined as a wrapper
for the rospy Rate, which keeps loops running on a near fixed time interval. It
can be constructed with a `Duration` object, or a floating-point value,
specifying the loop rate in Hz. Other functions are:

- `get_rostime()`, `now()` : Current time as `Time` object.
- `to_sec(time_obj)`, `convert(Float64, time_obj)` : Convert `Time` or
`Duration` object to floating-point number of seconds.
- `to_nsec(time_obj)` : Convert object to integral number of nanoseconds.
- `sleep(t::Duration)`, `sleep(t::FloatingPoint)`, `sleep(t::Rate)` : Sleep the
amount implied by type and value of the `t` parameter.

### Publishing Messages

Publishing messages is the same as in rospy, except use the `publish` method,
paired with a Publisher object. For example:

    using geometry_msgs.msg
    pub = Publisher{PointStamped}("topic", queue_size = 10) #or...
    #pub = Publisher("topic", PointStamped, queue_size = 10)
    msg = PointStamped()
    msg.header.stamp = now()
    msg.point.x = 1.1
    publish(pub, msg)

The keyword arguments in the `Publisher` constructor are passed directly on to
rospy so anything it accepts will be valid.

### Subscribing to a Topic

Subscribing to a topic is the same as in rospy. When creating a `Subscriber`,
an optional `callback_args` parameter can be given to forward on whenever the
callback is invoked. Note that it must be passed as a tuple, even if there is
only a single argument. And again, keyword arguments are directly forwarded. An
example:

    using sensor_msgs.msg
    cb1(msg::Imu, a::String) = println(a,": ",msg.linear_acceleration.x)
    cb2(msg::Imu) = println(msg.angular_velocity.z)
    sub1 = Subscriber{Imu}("topic", cb1, ("accel",), queue_size = 10) #or...
    #sub1 = Subscriber("topic", Imu, cb1, ("accel",), queue_size = 10)
    sub2 = Subscriber{Imu}("topic", cb2, queue_size = 10)
    spin()

### Parameter Server

`get_param`, `set_param`, `has_param`, and `delete_param` are all implemented
in the `RobotOS` module with the same syntax as in rospy.

## ROS Integration

Since Julia code needs no prior compilation, it is possible to integrate very
tightly and natively with a larger ROS system. Just make sure you:

- Keep your code inside your ROS packages as usual.
- Ensure your .jl script is executable (e.g., `chmod a+x script.jl`) and has
the hint to the Julia binary as the first line (`#!/usr/bin/env julia`).

Now your Julia code will run exactly like any python script that gets invoked
through `rosrun` or `roslaunch`. And since `include` takes paths relative to
the location of the calling file, you can bring in whatever other modules or
functions reside in your package from the single executable script.

    #!/usr/bin/env julia
    #main.jl in thebot_pkg/src
    using RobotOS

    include("BotSrc/Bot.jl")
    using Bot
    #...

## Full example

This example demonstrates publishing a random `geometry_msgs/Point` message at
5 Hz. It also listens for incoming `geometry_msgs/Pose2D` messages and
republishes them as Points.

    #!/usr/bin/env julia

    using RobotOS
    @rosimport geometry_msgs.msg: Point, Pose2D
    gentypes()
    using geometry_msgs.msg

    callback(msg::Pose2D, pub_obj::Publisher{Point}) = begin
        pt_msg = Point(msg.x, msg.y, 0.0)
        publish(pub_obj, pt_msg)
    end
    pub = Publisher{Point}("pts", queue_size=10)
    sub = Subscriber{Pose2D}("pose", callback, (pub,), queue_size=10)

    init_node("rosjl_example")
    loop_rate = Rate(5.0)
    while ! is_shutdown()
        npt = Point(rand(), rand(), 0.0)
        publish(pub, npt)
        sleep(loop_rate)
    end

## Versions

- `0.1` : Initial release
- `0.2` : Changed type gen API and moved generated modules to Main
