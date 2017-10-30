
<a id='API-Reference-1'></a>

# API Reference


<a id='ROS-Type-Generation-1'></a>

## ROS Type Generation

<a id='RobotOS.@rosimport' href='#RobotOS.@rosimport'>#</a>
**`RobotOS.@rosimport`** &mdash; *Macro*.



```
@rosimport
```

Import ROS message or service types into Julia. Call `rostypegen()` after all `@rosimport` calls. Package or type dependencies are also imported automatically as needed.

Example usages:

```julia
  @rosimport geometry_msgs.msg.PoseStamped
  @rosimport sensor_msgs.msg: Image, Imu
  @rosimport nav_msgs.srv.GetPlan
```


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/gentypes.jl#L69-L81' class='documenter-source'>source</a><br>

<a id='RobotOS.rostypegen' href='#RobotOS.rostypegen'>#</a>
**`RobotOS.rostypegen`** &mdash; *Function*.



```
rostypegen()
```

Initiate the Julia type generation process after importing some ROS types. Creates modules in `Main` with the same behavior as imported ROS modules in python. Should only be called once, after all `@rosimport` statements are done.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/gentypes.jl#L146-L152' class='documenter-source'>source</a><br>

<a id='RobotOS.rostypereset' href='#RobotOS.rostypereset'>#</a>
**`RobotOS.rostypereset`** &mdash; *Function*.



```
rostypereset()
```

Clear out the previous `@rosimport`s, returning the type generation to its original state. Cannot do anything about already generated modules in `Main`.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/gentypes.jl#L162-L167' class='documenter-source'>source</a><br>


<a id='Publishing-and-Subscribing-1'></a>

## Publishing and Subscribing

<a id='RobotOS.Publisher' href='#RobotOS.Publisher'>#</a>
**`RobotOS.Publisher`** &mdash; *Type*.



```
Publisher{T}(topic; kwargs...)
Publisher(topic, T; kwargs...)
```

Create an object to publish messages of type `T` on a topic. Keyword arguments are directly passed to rospy.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/pubsub.jl#L6-L12' class='documenter-source'>source</a><br>

<a id='RobotOS.publish' href='#RobotOS.publish'>#</a>
**`RobotOS.publish`** &mdash; *Function*.



```
publish(p::Publisher{T}, msg::T)
```

Publish `msg` on `p`, a `Publisher` with matching message type.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/pubsub.jl#L26-L30' class='documenter-source'>source</a><br>

<a id='RobotOS.Subscriber' href='#RobotOS.Subscriber'>#</a>
**`RobotOS.Subscriber`** &mdash; *Type*.



```
Subscriber{T}(topic, callback, cb_args=(); kwargs...)
Subscriber(topic, T, callback, cb_args=(); kwargs...)
```

Create a subscription to a topic with message type `T` with a callback to use when a message is received, which can be any callable type. Extra arguments provided to the callback when invoked can be provided in the `cb_args` tuple. Keyword arguments are directly passed to rospy.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/pubsub.jl#L35-L42' class='documenter-source'>source</a><br>


<a id='Services-1'></a>

## Services

<a id='RobotOS.Service' href='#RobotOS.Service'>#</a>
**`RobotOS.Service`** &mdash; *Type*.



```
Service{T}(name, callback; kwargs...)
Service(name, T, callback; kwargs...)
```

Create a service object that can receive requests and provide responses. The callback can be of any callable type. Keyword arguments are directly passed to rospy.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/services.jl#L42-L48' class='documenter-source'>source</a><br>

<a id='RobotOS.ServiceProxy' href='#RobotOS.ServiceProxy'>#</a>
**`RobotOS.ServiceProxy`** &mdash; *Type*.



```
ServiceProxy{T}(name; kwargs...)
ServiceProxy(name, T; kwargs...)
```

Create a proxy object used to invoke a remote service. Use `srv_proxy(msg_request)` with the object to invoke the service call. Keyword arguments are directly passed to rospy.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/services.jl#L6-L12' class='documenter-source'>source</a><br>

<a id='RobotOS.wait_for_service' href='#RobotOS.wait_for_service'>#</a>
**`RobotOS.wait_for_service`** &mdash; *Function*.



```
wait_for_service(srv_name; kwargs...)
```

Block until the specified service is available. Keyword arguments are directly passed to rospy. Throws an exception if the waiting timeout period is exceeded.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/services.jl#L88-L93' class='documenter-source'>source</a><br>


<a id='General-ROS-Functions-1'></a>

## General ROS Functions

<a id='RobotOS.init_node' href='#RobotOS.init_node'>#</a>
**`RobotOS.init_node`** &mdash; *Function*.



```
init_node(name; args...)
```

Initialize this node, registering it with the ROS master. All arguments are passed on directly to the rospy init_node function.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/rospy.jl#L6-L11' class='documenter-source'>source</a><br>

<a id='RobotOS.is_shutdown' href='#RobotOS.is_shutdown'>#</a>
**`RobotOS.is_shutdown`** &mdash; *Function*.



```
is_shutdown()
```

Return the shutdown status of the node.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/rospy.jl#L15-L19' class='documenter-source'>source</a><br>

<a id='RobotOS.spin' href='#RobotOS.spin'>#</a>
**`RobotOS.spin`** &mdash; *Function*.



```
spin()
```

Block execution and process callbacks/service calls until the node is shut down.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/rospy.jl#L25-L29' class='documenter-source'>source</a><br>


<a id='Time-Handling-1'></a>

## Time Handling

<a id='RobotOS.Time' href='#RobotOS.Time'>#</a>
**`RobotOS.Time`** &mdash; *Type*.



```
Time(secs, nsecs), Time(), Time(t::Real)
```

Object representing an absolute time from a fixed past reference point at nanosecond precision.

Basic arithmetic can be performed on combinations of `Time` and `Duration` objects that make sense. For example, if `t::Time` and `d::Duration`, `t+d` will be a `Time`, `d+d` a `Duration`, `t-d` a `Time`, `d-d` a `Duration`, and `t-t` a `Duration`.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/time.jl#L11-L19' class='documenter-source'>source</a><br>

<a id='RobotOS.Duration' href='#RobotOS.Duration'>#</a>
**`RobotOS.Duration`** &mdash; *Type*.



```
Duration(secs, nsecs), Duration(), Duration(t::Real)
```

Object representing a relative period of time at nanosecond precision.

Basic arithmetic can be performed on combinations of `Time` and `Duration` objects that make sense. For example, if `t::Time` and `d::Duration`, `t+d` will be a `Time`, `d+d` a `Duration`, `t-d` a `Time`, `d-d` a `Duration`, and `t-t` a `Duration`.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/time.jl#L31-L39' class='documenter-source'>source</a><br>

<a id='RobotOS.Rate' href='#RobotOS.Rate'>#</a>
**`RobotOS.Rate`** &mdash; *Type*.



```
Rate(hz::Real), Rate(d::Duration)
```

Used to allow a loop to run at a fixed rate. Construct with a frequency or `Duration` and use with `rossleep` or `sleep`. The rate object will record execution time of other work in the loop and modify the sleep time to compensate, keeping the loop rate as consistent as possible.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/time.jl#L101-L107' class='documenter-source'>source</a><br>

<a id='RobotOS.to_sec' href='#RobotOS.to_sec'>#</a>
**`RobotOS.to_sec`** &mdash; *Function*.



```
to_sec(t)
```

Return the value of a ROS time object in absolute seconds (with nanosecond precision)


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/time.jl#L82-L86' class='documenter-source'>source</a><br>

<a id='RobotOS.to_nsec' href='#RobotOS.to_nsec'>#</a>
**`RobotOS.to_nsec`** &mdash; *Function*.



```
to_nsec(t)
```

Return the value of a ROS time object in nanoseconds as an integer.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/time.jl#L89-L93' class='documenter-source'>source</a><br>

<a id='RobotOS.now' href='#RobotOS.now'>#</a>
**`RobotOS.now`** &mdash; *Function*.



```
RobotOS.now()
```

Return the current ROS time as a `Time` object.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/time.jl#L129-L133' class='documenter-source'>source</a><br>

<a id='RobotOS.get_rostime' href='#RobotOS.get_rostime'>#</a>
**`RobotOS.get_rostime`** &mdash; *Function*.



```
get_rostime()
```

Return the current ROS time as a `Time` object.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/time.jl#L115-L119' class='documenter-source'>source</a><br>

<a id='RobotOS.rossleep' href='#RobotOS.rossleep'>#</a>
**`RobotOS.rossleep`** &mdash; *Function*.



```
rossleep(t)
```

Sleep and process callbacks for a number of seconds implied by the type and value of `t`, which may be a real-value, a `Duration` object, or a `Rate` object.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/time.jl#L136-L141' class='documenter-source'>source</a><br>

<a id='Base.sleep' href='#Base.sleep'>#</a>
**`Base.sleep`** &mdash; *Function*.



```
sleep(t::Duration), sleep(t::Rate)
```

Call `rossleep` with a `Duration` or `Rate` object. Use `rossleep` to specify sleep time directly.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/time.jl#L167-L171' class='documenter-source'>source</a><br>


<a id='Parameters-1'></a>

## Parameters

<a id='RobotOS.get_param' href='#RobotOS.get_param'>#</a>
**`RobotOS.get_param`** &mdash; *Function*.



```
get_param(param_name, default=nothing)
```

Request the value of a parameter from the parameter server, with optional default value. If no default is given, throws a `KeyError` if the parameter cannot be found.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/rospy.jl#L39-L44' class='documenter-source'>source</a><br>

<a id='RobotOS.set_param' href='#RobotOS.set_param'>#</a>
**`RobotOS.set_param`** &mdash; *Function*.



```
set_param(param_name, val)
```

Set the value of a parameter on the parameter server.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/rospy.jl#L57-L61' class='documenter-source'>source</a><br>

<a id='RobotOS.has_param' href='#RobotOS.has_param'>#</a>
**`RobotOS.has_param`** &mdash; *Function*.



```
has_param(param_name)
```

Return a boolean specifying if a parameter exists on the parameter server.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/rospy.jl#L65-L69' class='documenter-source'>source</a><br>

<a id='RobotOS.delete_param' href='#RobotOS.delete_param'>#</a>
**`RobotOS.delete_param`** &mdash; *Function*.



```
delete_param(param_name)
```

Delete a parameter from the parameter server. Throws a `KeyError` if no such parameter exists.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/rospy.jl#L73-L77' class='documenter-source'>source</a><br>


<a id='Logging-1'></a>

## Logging

<a id='RobotOS.logdebug' href='#RobotOS.logdebug'>#</a>
**`RobotOS.logdebug`** &mdash; *Function*.



```
logdebug, loginfo, logwarn, logerr, logfatal
```

Call the rospy logging system at the corresponding message level, passing a message and other arguments directly.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/rospy.jl#L98-L103' class='documenter-source'>source</a><br>

<a id='RobotOS.loginfo' href='#RobotOS.loginfo'>#</a>
**`RobotOS.loginfo`** &mdash; *Function*.



```
logdebug, loginfo, logwarn, logerr, logfatal
```

Call the rospy logging system at the corresponding message level, passing a message and other arguments directly.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/rospy.jl#L98-L103' class='documenter-source'>source</a><br>

<a id='RobotOS.logwarn' href='#RobotOS.logwarn'>#</a>
**`RobotOS.logwarn`** &mdash; *Function*.



```
logdebug, loginfo, logwarn, logerr, logfatal
```

Call the rospy logging system at the corresponding message level, passing a message and other arguments directly.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/rospy.jl#L98-L103' class='documenter-source'>source</a><br>

<a id='RobotOS.logerr' href='#RobotOS.logerr'>#</a>
**`RobotOS.logerr`** &mdash; *Function*.



```
logdebug, loginfo, logwarn, logerr, logfatal
```

Call the rospy logging system at the corresponding message level, passing a message and other arguments directly.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/rospy.jl#L98-L103' class='documenter-source'>source</a><br>

<a id='RobotOS.logfatal' href='#RobotOS.logfatal'>#</a>
**`RobotOS.logfatal`** &mdash; *Function*.



```
logdebug, loginfo, logwarn, logerr, logfatal
```

Call the rospy logging system at the corresponding message level, passing a message and other arguments directly.


<a target='_blank' href='https://github.com/jdlangs/RobotOS.jl/blob/be3b15cc340146f51a96b30f4a812d4a3b650dd0/src/rospy.jl#L98-L103' class='documenter-source'>source</a><br>

