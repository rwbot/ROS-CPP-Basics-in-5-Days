## My first ROS program

1 - Create in the src directory in my_package a C++ file that will be executed.
For this exercise, just copy this simple C++ code simple.cpp.
> SIMPLE.CPP

```
#include <ros/ros.h>
// Here we are including all the headers necessary to use the most common public pieces of the ROS system.
// Always we create a new C++ file, we will need to add this include.

int main(int argc, char** argv) { // We start the main C++ program

    ros::init(argc, argv, "ObiWan"); // We initiate a ROS node called ObiWan
    ros::NodeHandle nh; // We create a handler for the node. This handler will actually do the initialization of the node
    ROS_INFO("Help me Obi-Wan Kenobi, you're my only hope"); // This is the same as a print in ROS
    ros::spinOnce();
    /*
    Calling ros::spinOnce() here is not necessary for this simple program, because we are not receiving any callbacks.
    However, if you were to add a subscription into this application, and did not have ros::spinOnce() here,
    your callbacks would never get called. So, add it for good measure.
    */
    return 0; // We end our program
}
```

2 - Create a launch directory inside the package named my_package {Example 1.4}.
3 - Create a new launch file inside the launch directory.
4 - Fill this launch file
```
touch launch/my_package_launch_file.launch
```
Launch File
```
<launch>
    <node pkg="my_package" type="simple" name="ObiWan" output="screen"></node>
</launch>

```


## Modifying the CMakeLists.txt
5 - Modify the CMakeLists.txt file in order to generate an executable from the C++ file you have just created.
#### Note: This is something that is required when working in ROS with C++.

In the Build section of your CMakeLists.txt file, add the following lines:
```
add_executable(simple src/simple.cpp)
add_dependencies(simple ${simple_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple
   ${catkin_LIBRARIES}
 )
```

When coding with C++, it will be necessary to create binaries(executables) of your programs in order to be able to execute them. For that, you will need to modify the CMakeLists.txt file of your package, in order to indicate that you want to create an executable of your C++ file.

To do this, you need to add some lines into your CMakeLists.txt file. In fact, these lines are already in the file, but they are commented. You can also find them, and uncomment them. Whatever you want.

In the previous Exercise, you had the following lines:

```
add_executable(simple src/simple.cpp)
```
This line generates an executable from the simple.cpp file, which is in the src folder of your package. This executable will be placed by default into the package directory of your devel space, which is located by default at ~/catkin_ws/devel/lib/.

```
add_dependencies(simple ${simple_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```
This line adds all the cmake target dependencies of the executable.

```
target_link_libraries(simple
   ${catkin_LIBRARIES}
 )
 ```
This line specifies the libraries to use when linking a given target. For this case, it indicates to use the catkin libraries when linking to the executable you have created.


## ROS Nodes

> SIMPLE_LOOP.CPP

This program creates an endless loop that repeats itself 2 times per second (2Hz) until somebody presses Ctrl + C
```
#include <ros/ros.h>
int main(int argc, char** argv) {

    ros::init(argc, argv, "ObiWan");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2); // We create a Rate object of 2Hz

    while (ros::ok()) // Endless loop until Ctrl + C
    {
        ROS_INFO("Help me Obi-Wan Kenobi, you're my only hope");
        ros::spinOnce();
        loop_rate.sleep(); // We sleep the needed time to maintain the Rate fixed above
    }
    return 0;
}
```

---
ROSNODE_INFO
```
rosnode info /ObiWan
```
Output:
```
user ~ $ rosnode info /ObiWan
--------------------------------------------------------------------------------
Node [/ObiWan]
Publications:
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]

Services:
 * /ObiWan/set_logger_level
 * /ObiWan/get_loggers


contacting node http://ip-172-31-30-5:58680/ ...
Pid: 1215
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo (http://ip-172-31-30-5:46415/)
    * direction: inbound
    * transport: TCPROS
```

## Parameter Server
A Parameter Server is a dictionary that ROS uses to store parameters. These parameters can be used by nodes at runtime and are normally used for static data, such as configuration parameters.

To get a list of these parameters, you can type:
```
rosparam list
```
To get a value of a particular parameter, you can type:
```
rosparam get <parameter_name>
```
And to set a value to a parameter, you can type:
```
rosparam set <parameter_name> <value>
```

## Environment Variables
ROS uses a set of Linux system environment variables in order to work properly. You can check these variables by typing:

```
export | grep ROS
```
Output:
```
user ~ $ export | grep ROS
declare -x ROSLISP_PACKAGE_DIRECTORIES="/home/user/catkin_ws/devel/share/common-lisp"
declare -x ROS_DISTRO="indigo"
declare -x ROS_ETC_DIR="/opt/ros/indigo/etc/ros"
declare -x ROS_MASTER_URI="http://localhost:11311"
declare -x ROS_PACKAGE_PATH="/home/user/catkin_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks"
declare -x ROS_ROOT="/opt/ros/indigo/share/ros"
```
The most important variables are the `ROS_MASTER_URI` and the `ROS_PACKAGE_PATH`.
```
ROS_MASTER_URI -> Contains the url where the ROS Core is being executed. Usually, your own computer (localhost).
ROS_PACKAGE_PATH -> Contains the paths in your Hard Drive where ROS has packages in it.
```


























































#
