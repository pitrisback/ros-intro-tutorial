# ROS package tutorial

### Create the workspace
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
catkin_init_workspace
```
Add to sourced files (e.g. `~/.bash_aliases`) this line
```bash
source ~/ros_ws/devel/setup.bash
```

### Install a package
```bash
sudo apt-get install ros-melodic-[package_name]
```

### Build the workspace
```bash
catkin build
```

### Create a new package
```bash
cd ~/ros_ws/src
catkin_create_pkg [package_name] [depend1] [depend2] [depend3]
```
e.g.
```bash
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
~/ros_ws/src/intro_tutorial
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
│  ├── ex1_publisher.cpp
│  ├── ex1_subscriber.cpp
│  ├── ex2_msg_publisher.cpp
│  ├── ex2_msg_subscriber.cpp
│  ├── ex3_add3ints_client.cpp
│  ├── ex3_add3ints_server.cpp
│  ├── ex4_fibonacci_client.cpp
│  └── ex4_fibonacci_server.cpp
└── srv
   └── srv1.srv
```

### Start ros and check status
Start ros with `roscore`.

Info on nodes
```bash
rosnode list
rosnode info [node_name]
rosnode kill [node_name]
rosnode list
rosnode ping [node_name]
```

Info on topics
```bash
rostopic echo [msg_name]
rostopic find [msg_type]
rostopic info [msg_name]
rostopic list
rostopic type [msg_name]
```

Info on messages
```bash
rosmsg show [msg_name]
rosmsg list
rosmsg [package_name]
rosmsg packages [msg_name]
```

Info on services
```bash
rosservice call [srv_name] [args]
rosservice find [srv_type]
rosservice info [srv_name]
rosservice list
rosservice type [srv_name]
```

### Publisher - subscriber
Run the example
```bash
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

### Use actions
Define the action structure in `action/Fibonacci.action`
```
#goal definition
int32 order
---
#result definition
int32[] sequence
---
#feedback
int32[] sequence
```

Create the class FibonacciAction server side
```cpp
protected:
    ros::NodeHandle nh_;
    // create the server
    actionlib::SimpleActionServer<intro_tutorial::FibonacciAction> as_;
    std::string action_name_;
    intro_tutorial::FibonacciFeedback feedback_;
    intro_tutorial::FibonacciResult result_;
public:
    FibonacciAction(std::string name) :
        as_(nh_,
            name,
            boost::bind(&FibonacciAction::executeCB,
                        this,
                        _1),
            false
        ),
        action_name_(name){
            as_.start();
        }
```
Setup the client side
```cpp
// create the client
actionlib::SimpleActionClient<intro_tutorial::FibonacciAction> ac("fibonacci", true);
// setup goal
intro_tutorial::FibonacciGoal goal;
goal.order = 20;
ac.sendGoal(goal);
```

### Setup environment with launch file
Create the launch file `example1.launch`
```xml
<?xml version="1.0"?>
<launch>
    <node name ="ex1_publisher" pkg="intro_tutorial" type="ex1_publisher"/>
    <node name ="ex1_subscriber" pkg="intro_tutorial" type="ex1_subscriber"/>
</launch>
```
And launch with
```bash
roslaunch intro_tutorial example1.launch
```


### CMake/manifest structure

In the `CMakeList.txt` file
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(intro_tutorial)

## Find catkin macros and libraries
find_package(
    catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        actionlib
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
        msg1.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
        srv1.srv
)

## Generate actions in the 'action' folder
add_action_files(
    DIRECTORY
        action
    FILES
        Fibonacci.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
        std_msgs
        actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        std_msgs
        actionlib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(ex1_publisher src/ex1_publisher.cpp)
add_dependencies(ex1_publisher intro_tutorial_generate_messages_cpp)
target_link_libraries(ex1_publisher ${catkin_LIBRARIES})
```

In the `package.xml` file
```xml
<?xml version="1.0"?>
<package format="2">
  <name>intro_tutorial</name>
  <version>0.0.0</version>
  <description>The intro_tutorial package</description>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <exec_depend>roscpp</exec_depend>

  <build_depend>std_msgs</build_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>std_msgs</exec_depend>

  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <depend>actionlib</depend>

  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
</package>
```
