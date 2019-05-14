# RobotOS.jl Documentation


## Overview

### Description

This package enables interfacing Julia code with a ROS ([Robot Operating
System](http://wiki.ros.org)) system. It works by generating native Julia types
for ROS types, the same as in C++ or Python, and then wrapping rospy through
the PyCall package to get communication through topics, services, and
parameters.

### Installation

    Pkg.add("RobotOS")
    using RobotOS

### Contributing

The package will hopefully continue to undergo substantial improvement. Please
feel free to submit either an issue or pull request through github if you want
to fix something or suggest a needed improvment, even if it's just to add an
extra sentence in this README.

#### Testing

Currently, `Pkg.test("RobotOS")` requires some bootstrapping to work properly.
Before running Julia, make sure a ROS master is running and start the helper
node by running the `test/echonode.py` file.

## Usage: Type Generation

ROS types are brought into your program with the `@rosimport` macro which
specifies a package and one or more types. The three valid syntax forms can be
seen in these examples:

    @rosimport std_msgs.msg.Header
    @rosimport nav_msgs.srv: GetPlan
    @rosimport geometry_msgs.msg: PoseStamped, Vector3

`@rosimport` will import the python modules for the requested type and all
its dependencies but the native Julia types are not created yet since any
inter-module dependencies have to be resolved first. After the final
`@rosimport` call, initiate the type generation with:

    rostypegen()

The new types will be placed in newly created modules in `Main`, corresponding
to the packages requested. For example, `"std_msgs/Header" =>
std_msgs.msg.Header`. After calling `rostypegen()` they can be interacted with
just like regular modules with `import` and `using` statements bringing the
generated type names into the local namespace.

    using .nav_msgs.msg
    import geometry_msgs.msg: Pose, Vector3
    p = Path()
    v = Vector3(1.1,2.2,3.3)

There is one special case, where the ROS type name conflicts with a built-in
Julia type name (e.g., `std_msgs/Float64` or `std_msgs/String`). In these
cases, the generated Julia type will have "Msg" appended to the name for
disambiguation (e.g., `std_msgs.msg.Float64Msg` and `std_msgs.msg.StringMsg`).

An additional function, `rostypereset()`, resets the type generation process,
possibly useful for development in the REPL. When invoked, new `@rosimport`
calls will be needed to generate the same or different types, and previously
generated modules will be overwritten after `rostypegen()` is called again.  Keep
in mind that names cannot be cleared once defined so if a module is not
regenerated, the first version will remain.

### Compatibility with Package Precompilation
As described above, by default `rostypegen` creates modules in `Main` -- however,
this behavior is incompatible with Julia package precompilation. If you are using
`RobotOS` in your own module or package, as opposed to a script, you may reduce
load-time latency (useful for real-life applications!) by generating the ROS type
modules inside your package module using an approach similar to the example below:

    # MyROSPackage.jl
    module MyROSPackage

    using RobotOS

    @rosimport geometry_msgs.msg: Pose
    rostypegen(@__MODULE__)
    import .geometry_msgs.msg: Pose
    # ...

    end

In this case, we have provided `rostypegen` with a root module (`MyROSPackage`)
for type generation. The Julia type corresponding to `geometry_msgs/Pose` now
lives at `MyROSPackage.geometry_msgs.msg.Pose`; note the extra dot in
`import .geometry_msgs.msg: Pose`.

## Usage: ROS API

In general, the API functions provided directly match those provided in rospy,
with few cosmetic differences. The rospy API functions can reviewed here:
[http://wiki.ros.org/rospy/Overview](http://wiki.ros.org/rospy/Overview)

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

- `get_rostime()`, `RobotOS.now()` : Current time as `Time` object.
- `to_sec(time_obj)`, `convert(Float64, time_obj)` : Convert `Time` or
`Duration` object to floating-point number of seconds.
- `to_nsec(time_obj)` : Convert object to integral number of nanoseconds.
- `rossleep(t)` with `t` of type `Duration`, `Rate`, `Real`. Also
`sleep(t::Duration)` and `sleep(t::Rate)` : Sleep the amount implied by type
and value of the `t` parameter.

### Publishing Messages

Publishing messages is the same as in rospy, except use the `publish` method,
paired with a Publisher object. For example:

    using .geometry_msgs.msg
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

    using .sensor_msgs.msg
    cb1(msg::Imu, a::String) = println(a,": ",msg.linear_acceleration.x)
    cb2(msg::Imu) = println(msg.angular_velocity.z)
    sub1 = Subscriber{Imu}("topic", cb1, ("accel",), queue_size = 10) #or...
    #sub1 = Subscriber("topic", Imu, cb1, ("accel",), queue_size = 10)
    sub2 = Subscriber{Imu}("topic", cb2, queue_size = 10)
    spin()

### Using services

ROS services are fully supported, including automatic request and response type
generation. For the `@rosimport` call, use the plain service type name. After
`rostypegen()`, the generated `.srv` submodule will contain 3 types: the plain
type, a request type, and a response type. For example `@rosimport
nav_msgs.srv.GetPlan` will create `GetPlan`, `GetPlanRequest`, and
`GetPlanResponse`. To provide the service to other nodes, you would create a
`Service{GetPlan}` object. To call it, a `ServiceProxy{GetPlan}` object. The
syntax exactly matches rospy to construct and use these objects. For example,
if `myproxy` is a `ServiceProxy` object, it can be called with
`myproxy(my_request)`.

### Parameter Server

`get_param`, `set_param`, `has_param`, and `delete_param` are all implemented
in the `RobotOS` module with the same syntax as in rospy.

### Message Constants
Message constants may be accessed using `getproperty` syntax. For example for
[visualization_msgs/Marker.msg](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html)
we have:

    import .visualization_msgs.msg: Marker
    Marker.SPHERE == getproperty(Marker, :SPHERE) == 2   # true

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
    rostypegen()
    using .geometry_msgs.msg

    function callback(msg::Pose2D, pub_obj::Publisher{Point})
        pt_msg = Point(msg.x, msg.y, 0.0)
        publish(pub_obj, pt_msg)
    end

    function loop(pub_obj)
        loop_rate = Rate(5.0)
        while ! is_shutdown()
            npt = Point(rand(), rand(), 0.0)
            publish(pub_obj, npt)
            rossleep(loop_rate)
        end
    end

    function main()
        init_node("rosjl_example")
        pub = Publisher{Point}("pts", queue_size=10)
        sub = Subscriber{Pose2D}("pose", callback, (pub,), queue_size=10)
        loop(pub)
    end

    if ! isinteractive()
        main()
    end

## Versions

- `0.1` : Initial release
- `0.2` : Changed type gen API and moved generated modules to Main
- `0.3` : Added service type generation and API
- `0.4` : Julia v0.4+ support only
- `0.5` : Docs website, Julia v0.5+ support only
- `0.6` : Julia v0.6+ support only
