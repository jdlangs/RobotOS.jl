# Tutorials
This page presents specific examples on how to use the RobotOS package. In particular, some representative examples in the official [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) are reproduced with Julia here.

## 😊 Beginner-Level ROS Tutorials
As a prerequisite, you'd better first go through the [beginner level core ROS tutorial](http://wiki.ros.org/ROS/Tutorials) to get a basic understanding of crucial ROS concepts, like *node*, *topic*, *message*, and *service* etc.

&nbsp;

### 🅰️ Writing a Simple Publisher and Subscriber (Julia)
Please check the [C++](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) and [Python](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) versions (especially the latter) for a reference. We try to follow the Python version closely below. If you are new to ROS, it is highly recommended to read the more detailed explanations along with the [Python](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) version, since we mostly only list the Julia code without explaining its logic here.

**Task Description**

Implement a publisher named "talker" that publishes messages to a topic "chatter", and then implement a subscriber "listener" that subscribes to the same topic. 

**Preparatory Work**

1. Please first follow the official [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) (beginner level, particularly sections 3 and 4) to create a workspace `catkin_ws` and a package `beginner_tutorials` inside that workspace. 

2. Then go to the [Python tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) and we will follow it step by step in the below. To begin with, we need to create a sub-folder `scripts` inside the `beginner_tutorials` directory.
```bash
$ roscd beginner_tutorials
$ mkdir scripts
$ cd scripts
```

3. Before writing Julia code, first get familiar with the basic RobotOS usages in the *Introduction* page.

**Writing the Publisher Node**

Inside the folder `scripts`, create a Julia script file `talker.jl` using any editor you like. For instance, we may use the built-in editor `nano` on Ubuntu:
```bash
$ nano talker.jl
```
Copy and paste the following code into `talker.jl`
```julia
#!/usr/bin/env julia

using RobotOS
@rosimport std_msgs.msg.String
rostypegen()
using .std_msgs.msg: StringMsg


function talker()
    init_node("talker") # node name
    pub = Publisher{StringMsg}("chatter", queue_size=10) # topic name
    rate = Rate(10) # 10 Hz
    while !is_shutdown()
        hello_str = "hello world $(to_sec(get_rostime()))"
        loginfo(hello_str)
        publish(pub, StringMsg(hello_str))
        rossleep(rate)
    end
end


if !isinteractive()
    talker()
end
```
It should be fairly easy to understand the above code thanks to the virtually line-to-line correspondence between the Julia code and the [Python code]((http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)) in the official documentation. More details regarding the RobotOS APIs used above are available in the *API Reference* page.

Finally, change the mode of `talker.jl` such that it becomes executable.
```bash
$ chmod +x talker.jl
```

**Test the Publisher Node**

Let's first test the effectiveness of the publisher before proceeding to the subscriber. In the terminal, launch `roscore` by 
```bash
$ roscore
```
Then, in *another* terminal window, run our "talker" node as follows
```bash
$ rosrun beginner_tutorials talker.jl
```
Note that you can execute the above command in any directory assuming you have *sourced* the `catkin_ws/devel/setup.bash` in the current terminal (a standard practice in ROS). In the case that an error happens stating that the ROS package *beginner_tutorials* cannot be found, please first source the prior `setup.bash` file (always do this in a newly opened terminal). 

After we launch the "talker" node as above, the output should look like
```bash
┌ Warning: Message type 'String' conflicts with Julia builtin, will be imported as 'StringMsg'
└ @ RobotOS ~/.julia/packages/RobotOS/j0Tsl/src/gentypes.jl:181
[INFO] [1598183942.752312]: hello world 1.5981839427519388e9
[INFO] [1598183942.809463]: hello world 1.598183942809102e9
[INFO] [1598183942.908818]: hello world 1.598183942908384e9
[INFO] [1598183943.009542]: hello world 1.5981839430091467e9
[INFO] [1598183943.108855]: hello world 1.5981839431084187e9
[INFO] [1598183943.209047]: hello world 1.5981839432086961e9
...
```
The first warning is due to the name conflicts between ROS message types and standard Julia types, and RobotOS consequently renames the `String` message type to `StringMsg`. 

**Writing the Subscriber Node**

 After we confirm the validness of the publisher above, let's continue with the subscriber in this section. Assuming that you are in the `scripts` directory now, create a `listener.jl` file
 ```
 $ nano listener.jl
 ```
and fill it with the following code
```julia
#!/usr/bin/env julia

using RobotOS
@rosimport std_msgs.msg.String
rostypegen()
using .std_msgs.msg: StringMsg

function callback(msg)
    loginfo("$(RobotOS.get_caller_id()) I heard $(msg.data)")
end

function listener()
    init_node("listener")  # node name
    # subscribe to the topic "chatter"; the keyword argument queue_size is optional
    sub = Subscriber{StringMsg}("chatter", callback; queue_size=10)
    # spin() simply keeps Julia from exiting until this node is stopped
    spin()
end

if !isinteractive()
    listener()
end
```
Similarly, it should be straightforward to figure out the logic of the above code by referring the explanation accompanying the [Python implementation](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29). One interesting point may be the `RobotOS.get_caller_id()` method. We have to include the `RobotOS` package prefix here because `get_caller_id` is not exported. You may find this method in the [`rospy.jl` source file](https://github.com/jdlangs/RobotOS.jl/blob/master/src/rospy.jl) as well as other non-exported methods that wrap the interfaces of *rospy*. More generally, you may add your own wrapper of *rospy* here if the existing ones are not enough for your application.   

**Test the Subscriber Node**

You may notice a "Building your nodes" section in the Python tutorial. Nonetheless, since we did not use any custom messages or services in the above code and Julia needs no pre-compilation, there is no need to run `catkin_make` here. 

Supposing you have already started the `roscore` and the "talker" node in the above, now launch the "listener" node:
```bash
$ rosrun beginner_tutorials listener.jl
```
The output looks like
```bash
┌ Warning: Message type 'String' conflicts with Julia builtin, will be imported as 'StringMsg'
└ @ RobotOS ~/.julia/packages/RobotOS/j0Tsl/src/gentypes.jl:181
[INFO] [1598185341.640304]: /listener I heard hello world 1.5981853415771134e9
[INFO] [1598185341.682786]: /listener I heard hello world 1.598185341677189e9
[INFO] [1598185341.784312]: /listener I heard hello world 1.5981853417778049e9
[INFO] [1598185341.883779]: /listener I heard hello world 1.598185341877341e9
[INFO] [1598185341.983243]: /listener I heard hello world 1.5981853419769475e9
[INFO] [1598185342.082552]: /listener I heard hello world 1.5981853420773246e9
...
```
As we see, the subscriber `listener.jl` receives the messages published by the publisher `talker.jl` successfully. Since ROS is a loosely coupled framework, you may mix up nodes written in various languages. For example, you can write the above subscriber in Python (or C++, or any other language) and code the publisher still in Julia, or vice versa. 

&nbsp;

### 🅱️ Writing a Simple Service and Client (Julia)
We follow closely the official [Python tutorial](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29). 

**Task Description**

Implement a service node named "add_two_ints_server" that receives two integers and returns the sum of them. Then, write a client that makes use of this service for integer summation.

**Preparatory Work**

1. (Skip this step if you have already done it in the above task A.) Please first follow the official [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) (beginner level, particularly sections 3 and 4) to create a workspace `catkin_ws` and a package `beginner_tutorials`. 

2. Create the service file and make proper configurations according to the guide [Creating a ROS msg and srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv).
After your finish this step, make sure that you can find two (generated) Python files `catkin_ws/devel/lib/python3/dist-packages/beginner_tutorials/srv/_AddTwoInts.py` and `__init__.py` in the same folder (your Python version may differ though), since RobotOS.jl depends on such a Python package to create Julia types for corresponding ROS messages/service types.

3. Unless you have done it already, we need to create a sub-folder `scripts` inside the `beginner_tutorials` directory.
```bash
$ roscd beginner_tutorials
$ mkdir scripts
$ cd scripts
```

**Writing a Service Node**

Starting from this section, please refer to the [Writing a Simple Service and Client (Python)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29) for detailed explanation of the code. Particularly, recall that the service form appears as follows:
```bash
$ rossrv show AddTwoInts
[rospy_tutorials/AddTwoInts]:
int64 a
int64 b
---
int64 sum

[beginner_tutorials/AddTwoInts]:
int64 a
int64 b
---
int64 sum
```
(Two services because we have used the same name as the one in the ros package "ros_tutorials")

NOw change into the above `scripts` directory we have just made. Create a `add_two_ints_server.jl` file in this directory and paste the following code inside it:
```julia
#!/usr/bin/env julia

using RobotOS
@rosimport beginner_tutorials.srv.AddTwoInts
rostypegen()
# three types are generated in a submodule .[package].srv
using .beginner_tutorials.srv: AddTwoInts, AddTwoIntsResponse, AddTwoIntsRequest

function handle_add_two_ints(req)
    println("Returning [$(req.a) + $(req.b) = $(req.a + req.b)]")
    return AddTwoIntsResponse(req.a + req.b)
end

function add_two_ints_server()
    init_node("add_two_ints_server") # node name
    s = Service{AddTwoInts}("add_two_ints", handle_add_two_ints) # service name
    println("Ready to add two ints")
    spin()
end

if !isinteractive()
    add_two_ints_server()
end

```
Please refer to the [Python tutorial](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29) for an explanation of the above code. Don't forget to make the node executable:
```bash
chmod +x add_two_ints_server.jl
```

**Examine the Service Server**

As always, be sure that you have already sourced the proper setup file. If you have not otherwise, run the following command in a terminal
```bash
catkin_ws$ source devel/setup.bash
```
Assuming that you have started the master node by `roscore`, now let's launch the server with `rosrun` (just like a normal ROS node).
```bash
$ rosrun beginner_tutorials add_two_ints_server.jl 
Ready to add two ints

```
The above output confirms that our server is working. 

**Writing the Client Node**

Like the above server code, first create a `add_two_ints_client.jl` file in the above `scripts` folder. A convenient way to change into this folder from any current directory is to use the `roscd` command, which is capable to locate a ROS package automatically:
```bash
$ roscd beginner_tutorials/scripts
$ nano add_two_ints_client.jl
```
Then, paste the following client code into the `add_two_ints_client.jl` file.
```julia
#!/usr/bin/env julia

using RobotOS
@rosimport beginner_tutorials.srv.AddTwoInts
rostypegen()
using .beginner_tutorials.srv: AddTwoInts, AddTwoIntsResponse, AddTwoIntsRequest

function add_two_ints_client(x, y)
    wait_for_service("add_two_ints")
    try
        add_two_ints = ServiceProxy{AddTwoInts}("add_two_ints")
        respl = add_two_ints(AddTwoIntsRequest(x, y)) # got an AddTwoIntsResponse
        return respl.sum
    catch err
        println("Service call failed: $err")
    end
end

usage() = "$PROGRAM_FILE [x y]"

if !isinteractive()
    if length(ARGS) == 2
        x = parse(Int, ARGS[1])
        y = parse(Int, ARGS[2])
    else
        println(usage())
        exit(1)
    end
    println("Requesting $x + $y")
    println("$x + $y = $(add_two_ints_client(x, y))")
end

```
Again, please check the [Python tutorial](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29) for an explanation of the above code. Finally, make the above file executable.
```bash
$ chmod +x add_two_ints_client.jl
```

**Examining the Simple Service and Client**

Assuming that you have started the master node by `roscore`, let's start by running the service server (just like above):
```bash
$ rosrun beginner_tutorials add_two_ints_server.jl
```
Next, we run the client of the service in a *new* terminal. Note that whenever you open a new terminal, you should first `source` the `[Workspace]/devel/setup.bash` file; otherwise, the ROS package we have developed cannot be found. The client is started just like any other node with `rosrun`. 
```bash
$ rosrun beginner_tutorials add_two_ints_client.jl 1 3
Requesting 1 + 3
1 + 3 = 4
```
The above output proves that our server and client are both working well.


