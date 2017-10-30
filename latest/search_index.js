var documenterSearchIndex = {"docs": [

{
    "location": "index.html#",
    "page": "Home",
    "title": "Home",
    "category": "page",
    "text": ""
},

{
    "location": "index.html#RobotOS.jl-Documentation-1",
    "page": "Home",
    "title": "RobotOS.jl Documentation",
    "category": "section",
    "text": ""
},

{
    "location": "index.html#Overview-1",
    "page": "Home",
    "title": "Overview",
    "category": "section",
    "text": ""
},

{
    "location": "index.html#Description-1",
    "page": "Home",
    "title": "Description",
    "category": "section",
    "text": "This package enables interfacing Julia code with a ROS (Robot Operating System) system. It works by generating native Julia types for ROS types, the same as in C++ or Python, and then wrapping rospy through the PyCall package to get communication through topics, services, and parameters."
},

{
    "location": "index.html#Installation-1",
    "page": "Home",
    "title": "Installation",
    "category": "section",
    "text": "Pkg.add(\"RobotOS\")\nusing RobotOS"
},

{
    "location": "index.html#Contributing-1",
    "page": "Home",
    "title": "Contributing",
    "category": "section",
    "text": "The package will hopefully continue to undergo substantial improvement. Please feel free to submit either an issue or pull request through github if you want to fix something or suggest a needed improvment, even if it's just to add an extra sentence in this README."
},

{
    "location": "index.html#Testing-1",
    "page": "Home",
    "title": "Testing",
    "category": "section",
    "text": "Currently, Pkg.test(\"RobotOS\") requires some bootstrapping to work properly. Before running Julia, make sure a ROS master is running and start the helper node by running the test/echonode.py file."
},

{
    "location": "index.html#Usage:-Type-Generation-1",
    "page": "Home",
    "title": "Usage: Type Generation",
    "category": "section",
    "text": "ROS types are brought into your program with the @rosimport macro which specifies a package and one or more types. The three valid syntax forms can be seen in these examples:@rosimport std_msgs.msg.Header\n@rosimport nav_msgs.srv: GetPlan\n@rosimport geometry_msgs.msg: PoseStamped, Vector3@rosimport will import the python modules for the requested type and all its dependencies but the native Julia types are not created yet since any inter-module dependencies have to be resolved first. After the final @rosimport call, initiate the type generation with:rostypegen()The new types will be placed in newly created modules in Main, corresponding to the packages requested. For example, \"std_msgs/Header\" => std_msgs.msg.Header. After calling rostypegen() they can be interacted with just like regular modules with import and using statements bringing the generated type names into the local namespace.using nav_msgs.msg\nimport geometry_msgs.msg: Pose, Vector3\np = Path()\nv = Vector3(1.1,2.2,3.3)There is one special case, where the ROS type name conflicts with a built-in Julia type name (e.g., std_msgs/Float64 or std_msgs/String). In these cases, the generated Julia type will have \"Msg\" appended to the name for disambiguation (e.g., std_msgs.msg.Float64Msg and std_msgs.msg.StringMsg).An additional function, rostypereset(), resets the type generation process, possibly useful for development in the REPL. When invoked, new @rosimport calls will be needed to generate the same or different types, and previously generated modules will be overwritten after rostypegen() is called again.  Keep in mind that names cannot be cleared once defined so if a module is not regenerated, the first version will remain."
},

{
    "location": "index.html#Usage:-ROS-API-1",
    "page": "Home",
    "title": "Usage: ROS API",
    "category": "section",
    "text": "In general, the API functions provided directly match those provided in rospy, with few cosmetic differences. The rospy API functions can reviewed here: http://wiki.ros.org/rospy/Overview"
},

{
    "location": "index.html#General-Functions-1",
    "page": "Home",
    "title": "General Functions",
    "category": "section",
    "text": "init_node(name::String; kwargs...) : Initialize node. Passes keywordarguments on to rospy directly. (Required)is_shutdown() : Check for ROS shutdown state.\nspin() :  Wait for callbacks until shutdown happens.\nlogdebug,loginfo,logwarn,logerr,logfatal all work as in rospy."
},

{
    "location": "index.html#Time-1",
    "page": "Home",
    "title": "Time",
    "category": "section",
    "text": "Native Julia types Time and Duration are defined, both as a composite of an integral number of seconds and nanoseconds, as in rospy.  Arithmetic and comparison operators are also defined. A Rate type is defined as a wrapper for the rospy Rate, which keeps loops running on a near fixed time interval. It can be constructed with a Duration object, or a floating-point value, specifying the loop rate in Hz. Other functions are:get_rostime(), RobotOS.now() : Current time as Time object.\nto_sec(time_obj), convert(Float64, time_obj) : Convert Time orDuration object to floating-point number of seconds.to_nsec(time_obj) : Convert object to integral number of nanoseconds.\nrossleep(t) with t of type Duration, Rate, Real. Alsosleep(t::Duration) and sleep(t::Rate) : Sleep the amount implied by type and value of the t parameter."
},

{
    "location": "index.html#Publishing-Messages-1",
    "page": "Home",
    "title": "Publishing Messages",
    "category": "section",
    "text": "Publishing messages is the same as in rospy, except use the publish method, paired with a Publisher object. For example:using geometry_msgs.msg\npub = Publisher{PointStamped}(\"topic\", queue_size = 10) #or...\n#pub = Publisher(\"topic\", PointStamped, queue_size = 10)\nmsg = PointStamped()\nmsg.header.stamp = now()\nmsg.point.x = 1.1\npublish(pub, msg)The keyword arguments in the Publisher constructor are passed directly on to rospy so anything it accepts will be valid."
},

{
    "location": "index.html#Subscribing-to-a-Topic-1",
    "page": "Home",
    "title": "Subscribing to a Topic",
    "category": "section",
    "text": "Subscribing to a topic is the same as in rospy. When creating a Subscriber, an optional callback_args parameter can be given to forward on whenever the callback is invoked. Note that it must be passed as a tuple, even if there is only a single argument. And again, keyword arguments are directly forwarded. An example:using sensor_msgs.msg\ncb1(msg::Imu, a::String) = println(a,\": \",msg.linear_acceleration.x)\ncb2(msg::Imu) = println(msg.angular_velocity.z)\nsub1 = Subscriber{Imu}(\"topic\", cb1, (\"accel\",), queue_size = 10) #or...\n#sub1 = Subscriber(\"topic\", Imu, cb1, (\"accel\",), queue_size = 10)\nsub2 = Subscriber{Imu}(\"topic\", cb2, queue_size = 10)\nspin()"
},

{
    "location": "index.html#Using-services-1",
    "page": "Home",
    "title": "Using services",
    "category": "section",
    "text": "ROS services are fully supported, including automatic request and response type generation. For the @rosimport call, use the plain service type name. After rostypegen(), the generated .srv submodule will contain 3 types: the plain type, a request type, and a response type. For example @rosimport nav_msgs.srv.GetPlan will create GetPlan, GetPlanRequest, and GetPlanResponse. To provide the service to other nodes, you would create a Service{GetPlan} object. To call it, a ServiceProxy{GetPlan} object. The syntax exactly matches rospy to construct and use these objects. For example, if myproxy is a ServiceProxy object, it can be called with myproxy(my_request)."
},

{
    "location": "index.html#Parameter-Server-1",
    "page": "Home",
    "title": "Parameter Server",
    "category": "section",
    "text": "get_param, set_param, has_param, and delete_param are all implemented in the RobotOS module with the same syntax as in rospy."
},

{
    "location": "index.html#Message-Constants-1",
    "page": "Home",
    "title": "Message Constants",
    "category": "section",
    "text": "Message constants may be accessed using getindex syntax. For example for visualization_msgs/Marker.msg we have:import visualization_msgs.msg: Marker\nMarker[:SPHERE] == getindex(Marker, :SPHERE) == 2   # true"
},

{
    "location": "index.html#ROS-Integration-1",
    "page": "Home",
    "title": "ROS Integration",
    "category": "section",
    "text": "Since Julia code needs no prior compilation, it is possible to integrate very tightly and natively with a larger ROS system. Just make sure you:Keep your code inside your ROS packages as usual.\nEnsure your .jl script is executable (e.g., chmod a+x script.jl) and hasthe hint to the Julia binary as the first line (#!/usr/bin/env julia).Now your Julia code will run exactly like any python script that gets invoked through rosrun or roslaunch. And since include takes paths relative to the location of the calling file, you can bring in whatever other modules or functions reside in your package from the single executable script.#!/usr/bin/env julia\n#main.jl in thebot_pkg/src\nusing RobotOS\n\ninclude(\"BotSrc/Bot.jl\")\nusing Bot\n#..."
},

{
    "location": "index.html#Full-example-1",
    "page": "Home",
    "title": "Full example",
    "category": "section",
    "text": "This example demonstrates publishing a random geometry_msgs/Point message at 5 Hz. It also listens for incoming geometry_msgs/Pose2D messages and republishes them as Points.#!/usr/bin/env julia\n\nusing RobotOS\n@rosimport geometry_msgs.msg: Point, Pose2D\nrostypegen()\nusing geometry_msgs.msg\n\nfunction callback(msg::Pose2D, pub_obj::Publisher{Point})\n    pt_msg = Point(msg.x, msg.y, 0.0)\n    publish(pub_obj, pt_msg)\nend\n\nfunction loop(pub_obj)\n    loop_rate = Rate(5.0)\n    while ! is_shutdown()\n        npt = Point(rand(), rand(), 0.0)\n        publish(pub_obj, npt)\n        rossleep(loop_rate)\n    end\nend\n\nfunction main()\n    init_node(\"rosjl_example\")\n    pub = Publisher{Point}(\"pts\", queue_size=10)\n    sub = Subscriber{Pose2D}(\"pose\", callback, (pub,), queue_size=10)\n    loop(pub)\nend\n\nif ! isinteractive()\n    main()\nend"
},

{
    "location": "index.html#Versions-1",
    "page": "Home",
    "title": "Versions",
    "category": "section",
    "text": "0.1 : Initial release\n0.2 : Changed type gen API and moved generated modules to Main\n0.3 : Added service type generation and API\n0.4 : Julia v0.4+ support only"
},

{
    "location": "api.html#",
    "page": "API Reference",
    "title": "API Reference",
    "category": "page",
    "text": ""
},

{
    "location": "api.html#RobotOS.Duration",
    "page": "API Reference",
    "title": "RobotOS.Duration",
    "category": "Type",
    "text": "Duration(secs, nsecs), Duration(), Duration(t::Real)\n\nObject representing a relative period of time at nanosecond precision.\n\nBasic arithmetic can be performed on combinations of Time and Duration objects that make sense. For example, if t::Time and d::Duration, t+d will be a Time, d+d a Duration, t-d a Time, d-d a Duration, and t-t a Duration.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.Publisher",
    "page": "API Reference",
    "title": "RobotOS.Publisher",
    "category": "Type",
    "text": "Publisher{T}(topic; kwargs...)\nPublisher(topic, T; kwargs...)\n\nCreate an object to publish messages of type T on a topic. Keyword arguments are directly passed to rospy.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.Rate",
    "page": "API Reference",
    "title": "RobotOS.Rate",
    "category": "Type",
    "text": "Rate(hz::Real), Rate(d::Duration)\n\nUsed to allow a loop to run at a fixed rate. Construct with a frequency or Duration and use with rossleep or sleep. The rate object will record execution time of other work in the loop and modify the sleep time to compensate, keeping the loop rate as consistent as possible.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.Service",
    "page": "API Reference",
    "title": "RobotOS.Service",
    "category": "Type",
    "text": "Service{T}(name, callback; kwargs...)\nService(name, T, callback; kwargs...)\n\nCreate a service object that can receive requests and provide responses. The callback can be of any callable type. Keyword arguments are directly passed to rospy.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.ServiceProxy",
    "page": "API Reference",
    "title": "RobotOS.ServiceProxy",
    "category": "Type",
    "text": "ServiceProxy{T}(name; kwargs...)\nServiceProxy(name, T; kwargs...)\n\nCreate a proxy object used to invoke a remote service. Use srv_proxy(msg_request) with the object to invoke the service call. Keyword arguments are directly passed to rospy.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.Subscriber",
    "page": "API Reference",
    "title": "RobotOS.Subscriber",
    "category": "Type",
    "text": "Subscriber{T}(topic, callback, cb_args=(); kwargs...)\nSubscriber(topic, T, callback, cb_args=(); kwargs...)\n\nCreate a subscription to a topic with message type T with a callback to use when a message is received, which can be any callable type. Extra arguments provided to the callback when invoked can be provided in the cb_args tuple. Keyword arguments are directly passed to rospy.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.Time",
    "page": "API Reference",
    "title": "RobotOS.Time",
    "category": "Type",
    "text": "Time(secs, nsecs), Time(), Time(t::Real)\n\nObject representing an absolute time from a fixed past reference point at nanosecond precision.\n\nBasic arithmetic can be performed on combinations of Time and Duration objects that make sense. For example, if t::Time and d::Duration, t+d will be a Time, d+d a Duration,t-daTime,d-daDuration, andt-taDuration`.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.delete_param-Tuple{AbstractString}",
    "page": "API Reference",
    "title": "RobotOS.delete_param",
    "category": "Method",
    "text": "delete_param(param_name)\n\nDelete a parameter from the parameter server. Throws a KeyError if no such parameter exists.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.get_param",
    "page": "API Reference",
    "title": "RobotOS.get_param",
    "category": "Function",
    "text": "get_param(param_name, default=nothing)\n\nRequest the value of a parameter from the parameter server, with optional default value. If no default is given, throws a KeyError if the parameter cannot be found.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.get_rostime-Tuple{}",
    "page": "API Reference",
    "title": "RobotOS.get_rostime",
    "category": "Method",
    "text": "get_rostime()\n\nReturn the current ROS time as a Time object.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.has_param-Tuple{AbstractString}",
    "page": "API Reference",
    "title": "RobotOS.has_param",
    "category": "Method",
    "text": "has_param(param_name)\n\nReturn a boolean specifying if a parameter exists on the parameter server.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.init_node-Tuple{AbstractString}",
    "page": "API Reference",
    "title": "RobotOS.init_node",
    "category": "Method",
    "text": "init_node(name; args...)\n\nInitialize this node, registering it with the ROS master. All arguments are passed on directly to the rospy init_node function.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.is_shutdown-Tuple{}",
    "page": "API Reference",
    "title": "RobotOS.is_shutdown",
    "category": "Method",
    "text": "is_shutdown()\n\nReturn the shutdown status of the node.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.logdebug",
    "page": "API Reference",
    "title": "RobotOS.logdebug",
    "category": "Function",
    "text": "logdebug, loginfo, logwarn, logerr, logfatal\n\nCall the rospy logging system at the corresponding message level, passing a message and other arguments directly.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.logerr",
    "page": "API Reference",
    "title": "RobotOS.logerr",
    "category": "Function",
    "text": "logdebug, loginfo, logwarn, logerr, logfatal\n\nCall the rospy logging system at the corresponding message level, passing a message and other arguments directly.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.logfatal",
    "page": "API Reference",
    "title": "RobotOS.logfatal",
    "category": "Function",
    "text": "logdebug, loginfo, logwarn, logerr, logfatal\n\nCall the rospy logging system at the corresponding message level, passing a message and other arguments directly.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.loginfo",
    "page": "API Reference",
    "title": "RobotOS.loginfo",
    "category": "Function",
    "text": "logdebug, loginfo, logwarn, logerr, logfatal\n\nCall the rospy logging system at the corresponding message level, passing a message and other arguments directly.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.logwarn",
    "page": "API Reference",
    "title": "RobotOS.logwarn",
    "category": "Function",
    "text": "logdebug, loginfo, logwarn, logerr, logfatal\n\nCall the rospy logging system at the corresponding message level, passing a message and other arguments directly.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.publish-Union{Tuple{MT}, Tuple{RobotOS.Publisher{MT},MT}} where MT<:RobotOS.AbstractMsg",
    "page": "API Reference",
    "title": "RobotOS.publish",
    "category": "Method",
    "text": "publish(p::Publisher{T}, msg::T)\n\nPublish msg on p, a Publisher with matching message type.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.rossleep-Tuple{RobotOS.Duration}",
    "page": "API Reference",
    "title": "RobotOS.rossleep",
    "category": "Method",
    "text": "rossleep(t)\n\nSleep and process callbacks for a number of seconds implied by the type and value of t, which may be a real-value, a Duration object, or a Rate object.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.rostypegen-Tuple{}",
    "page": "API Reference",
    "title": "RobotOS.rostypegen",
    "category": "Method",
    "text": "rostypegen()\n\nInitiate the Julia type generation process after importing some ROS types. Creates modules in Main with the same behavior as imported ROS modules in python. Should only be called once, after all @rosimport statements are done.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.rostypereset-Tuple{}",
    "page": "API Reference",
    "title": "RobotOS.rostypereset",
    "category": "Method",
    "text": "rostypereset()\n\nClear out the previous @rosimports, returning the type generation to its original state. Cannot do anything about already generated modules in Main.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.set_param-Tuple{AbstractString,Any}",
    "page": "API Reference",
    "title": "RobotOS.set_param",
    "category": "Method",
    "text": "set_param(param_name, val)\n\nSet the value of a parameter on the parameter server.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.spin-Tuple{}",
    "page": "API Reference",
    "title": "RobotOS.spin",
    "category": "Method",
    "text": "spin()\n\nBlock execution and process callbacks/service calls until the node is shut down.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.to_nsec-Union{Tuple{T}, Tuple{T}} where T<:RobotOS.TVal",
    "page": "API Reference",
    "title": "RobotOS.to_nsec",
    "category": "Method",
    "text": "to_nsec(t)\n\nReturn the value of a ROS time object in nanoseconds as an integer.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.to_sec-Union{Tuple{T}, Tuple{T}} where T<:RobotOS.TVal",
    "page": "API Reference",
    "title": "RobotOS.to_sec",
    "category": "Method",
    "text": "to_sec(t)\n\nReturn the value of a ROS time object in absolute seconds (with nanosecond precision)\n\n\n\n"
},

{
    "location": "api.html#RobotOS.wait_for_service-Tuple{AbstractString}",
    "page": "API Reference",
    "title": "RobotOS.wait_for_service",
    "category": "Method",
    "text": "wait_for_service(srv_name; kwargs...)\n\nBlock until the specified service is available. Keyword arguments are directly passed to rospy. Throws an exception if the waiting timeout period is exceeded.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.@rosimport-Tuple{Any}",
    "page": "API Reference",
    "title": "RobotOS.@rosimport",
    "category": "Macro",
    "text": "@rosimport\n\nImport ROS message or service types into Julia. Call rostypegen() after all @rosimport calls. Package or type dependencies are also imported automatically as needed.\n\nExample usages:\n\n  @rosimport geometry_msgs.msg.PoseStamped\n  @rosimport sensor_msgs.msg: Image, Imu\n  @rosimport nav_msgs.srv.GetPlan\n\n\n\n"
},

{
    "location": "api.html#Base.sleep-Tuple{RobotOS.Duration}",
    "page": "API Reference",
    "title": "Base.sleep",
    "category": "Method",
    "text": "sleep(t::Duration), sleep(t::Rate)\n\nCall rossleep with a Duration or Rate object. Use rossleep to specify sleep time directly.\n\n\n\n"
},

{
    "location": "api.html#RobotOS.now-Tuple{}",
    "page": "API Reference",
    "title": "RobotOS.now",
    "category": "Method",
    "text": "RobotOS.now()\n\nReturn the current ROS time as a Time object.\n\n\n\n"
},

{
    "location": "api.html#API-Reference-1",
    "page": "API Reference",
    "title": "API Reference",
    "category": "section",
    "text": "Modules = [RobotOS]"
},

]}
