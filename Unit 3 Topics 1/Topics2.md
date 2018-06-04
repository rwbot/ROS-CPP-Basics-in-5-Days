# Subscribers

* topic_publisher_pkg

* simple_topic_subscriber.cpp

```
#include <ros/ros.h>
#include <std_msgs/Int32.h>

// Define a function called 'callback' that receives a parameter named 'msg'
void counterCallback(const std_msgs::Int32::ConstPtr& msg)  {

    // Print the value 'data' inside the 'msg' parameter
    ROS_INFO("%d", msg->data);
}

int main(int argc, char** argv) {

    // Initiate a Node called 'topic_subscriber'
    ros::init(argc, argv, "topic_subscriber");
    ros::NodeHandle nh;

    // Create a Subscriber object that will listen to the /counter topic and
    // will call the 'callback' function each time it reads something from the topic
    ros::Subscriber sub = nh.subscribe("counter", 1000, counterCallback);  

    // Create a loop that will keep the program in execution
    ros::spin();
    return 0;
}
```

* topic_subscriber_launch_file.launch

```
<launch>
    <node pkg="topic_subscriber_pkg" type="simple_topic_subscriber" name="topic_subscriber" output="screen">
    </node>
</launch>
```

* CMakeLists.txt

```
add_executable(simple_topic_subscriber src/simple_topic_subscriber.cpp)

add_dependencies(simple_topic_subscriber ${simple_topic_subscriber_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(simple_topic_subscriber
  ${catkin_LIBRARIES}
)
```

## Publishing to a Topic
```
rostopic pub <topic_name> <message_type> <value>
```
**You can use TAB after <message_type> to autogenerate the message syntax**

Example:
```
rostopic pub /counter std_msgs/Int32 "data: 0"
```

### Odom_Subscriber.cpp

```
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("%s", msg->header.frame_id.c_str());
  ROS_INFO("%f", msg->twist.twist.linear.x);
  ROS_INFO("%f", msg->pose.pose.position.x);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "odom_sub_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("odom", 1000, odomCallback);

    ros::spin();

    return 0;
}
```

* odom_subscriber.launch

```
<launch>
    <node pkg="odom_subscriber_pkg" type="odom_subscriber" name="odom_sub_node" output="screen">
    </node>
</launch>
```

* CMakeLists.txt

```
add_executable(odom_subscriber src/odom_subscriber.cpp)

add_dependencies(odom_subscriber ${odom_subscriber_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(odom_subscriber
  ${catkin_LIBRARIES}
)
```


# How to Create a Custom ROS Message

In order to create a new message, Age.msg:

1. ##### Create a directory named `msg` inside your package, and there create a `.msg` file.
```
roscd <package_name>
mkdir msg
# touch message_name.msg
touch Age.msg
```
2. ##### Inside `message_name.msg`, and define the message types.
```
float32 years
float32 months
float32 days
```
3. ##### Modify CMakeLists.txt file
    * #### `find_package()`
    This is where all the packages required to COMPILE the messages of the topics, services, and actions go. In package.xml, you have to state them as build_depend.
    ```
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation   # Add message_generation here, after the other packages
    )
    ```

    * #### `add_message_files()`
    This function includes all of the messages of this package (in the msg folder) to be compiled. The file should look like this.
    ```
    add_message_files(
        FILES
        Age.msg
    )
    ```

    * #### `generate_messages()`
    Here is where the packages needed for the messages compilation are imported.
    ```
    generate_messages(
        DEPENDENCIES
        std_msgs
    )
    ```

    * #### `catkin_package()`
    State here all of the packages that will be needed by someone that executes something from your package. All of the packages stated here must be in the package.xml as run_depend.
    ```
    catkin_package(
        CATKIN_DEPENDS roscpp std_msgs message_runtime   # This will NOT be the only thing here
     )
    ```

##### This is the minimum required for CMakeLists.txt to work:

 ```
 cmake_minimum_required(VERSION 2.8.3)
 project($PACKAGE_NAME$)


 find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   message_generation
 )

 add_message_files(
   FILES
   Age.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

 catkin_package(
   CATKIN_DEPENDS roscpp std_msgs message_runtime
 )

 include_directories(
   ${catkin_INCLUDE_DIRS}
 )
 ```

4. ##### Modify `package.xml` file
```
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>
```

##### This is the minimum required for CMakeLists.txt to work:
```
<?xml version="1.0"?>
<package>
  <name>$PACKAGE_NAME$</name>
  <version>0.0.0</version>
  <description>The $PACKAGE_NAME$ package</description>

  <maintainer email="user@todo.todo">user</maintainer>

  <license>TODO</license>

 <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>std_msgs</run_depend>
  <run_depend>message_runtime</run_depend>

  <export>

  </export>
</package>
```

5. ##### Compile

This executes this bash file that sets, among other things, the newly generated messages created through the catkin_make.

```
roscd; cd ..
catkin_make
source devel/setup.bash
```


6. ##### Use in CPP Files by adding the following to CMakeLists.txt

```
add_dependencies(publish_age topic_ex_generate_messages_cpp)
```






































z
