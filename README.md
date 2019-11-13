# ROS package tutorial

### Create the workspace
```
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
catkin_init_workspace
```
Add to sourced files (e.g. `~/.bash_aliases`) this line
```
source ~/ros_ws/devel/setup.bash
```

### Install a package
```
sudo apt-get install ros-melodic-[package_name]
```

### Build the workspace
```
catkin build
```

### Create a new package
```
cd ~/ros_ws/src
catkin_create_pkg [package_name] [depend1] [depend2] [depend3]
```
e.g.
```
catkin_create_pkg intro_tutorial std_msgs roscpp
```

##### Navigate and edit packages
* find a package `rospack find [package_name]`
* find a metapackage `rosstack find [metapackage_name]`
* list the files inside a package `rosls [package_name]`
* modify a file `rosed [package_name] [file_name]`
* move to the package folder `roscd [package_name]`

### Package folder structure
```
ros_ws
├── action
│  └── Fibonacci.action
├── CMakeLists.txt
├── include
│  └── intro_tutorial
├── launch
│  └── example1.launch
├── msg
│  └── msg1.msg
├── package.xml
├── src
│  ├── .ycm_extra_conf.py
│  ├── example1_a.cpp
│  ├── example1_b.cpp
│  ├── example2_a.cpp
│  ├── example2_b.cpp
│  ├── example3_a.cpp
│  ├── example3_b.cpp
│  ├── fibonacci_client.cpp
│  └── fibonacci_server.cpp
└── srv
   └── srv1.srv
```

### Start ros and check node status
Start ros with `roscore`.

Info on nodes
```
rosnode list
rosnode info [node_name]
rosnode kill [node_name]
rosnode list
rosnode ping [node_name]
```

Info on topics
```
rostopic echo [msg_name]
rostopic find [msg_type]
rostopic info [msg_name]
rostopic list
rostopic type [msg_name]
```

### Publisher - subscriber
Run the example
```
rosrun intro_tutorial ex1_publisher
rosrun intro_tutorial ex1_subscriber
```

Publish a message
```cpp
ros::NodeHandle n;
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("message", 1000);
chatter_pub.publish(msg);
ros::spinOnce();
```

Listen for a message
```cpp
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("message", 1000, chatterCallback);
ros::spin();
```

### Use messages
Define the message structure in `msg/msg1.msg`
```
int32 A
int32 B
int32 C
```

Use the message as a struct basically
```cpp
intro_tutorial::msg1 msg;
msg.A = 1; msg.B = 2; msg.C = 3;
pub.publish(msg);
```

And in the callback
```cpp
ROS_INFO("I heard: [%d] [%d] [%d]", msg->A, msg->B, msg->C);
```
<!-- TODO: commands to list msgs -->

### Use services
Define the service structure in `srv/srv1.srv`
```
int32 A
int32 B
int32 C
---
int32 sum
```
<!-- TODO: commands to list srvs -->

The server `add_3_ints_server` provides a service
```cpp
ros::ServiceServer service = n.advertiseService("add_3_ints", add);
```
Inside a function with request/response as params
```cpp
bool add(intro_tutorial::srv1::Request &req, intro_tutorial::srv1::Response &res) {
    res.sum = req.A + req.B + req.C;
    return true;
}
```

The client `add_3_ints_client` uses the service
```cpp
// create the client that will use the add_3_ints service
ros::ServiceClient client = n.serviceClient<intro_tutorial::srv1>("add_3_ints");
// create and populate the service request
intro_tutorial::srv1 srv;
srv.request.A = atoll(argv[1]);
srv.request.B = atoll(argv[2]);
srv.request.C = atoll(argv[3]);
// call the function provided
if (client.call(srv)) {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
}
else {
    ROS_ERROR("Failed to call service add_3_ints");
    return 1;
}
```

### CMake/manifest structure

In the `CMakeList.txt` file
```
add_message_files(
    FILES
        msg1.msg
)
generate_messages(
    DEPENDENCIES
        std_msgs
)
catkin_package(
    CATKIN_DEPENDS
        message_runtime
)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
)
```

In the `package.xml` file
```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
