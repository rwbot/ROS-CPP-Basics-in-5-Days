# Publishers

### Simple_Topic_Publisher.CPP
```
#include <ros/ros.h>
#include <std_msgs/Int32.h>
// Import all the necessary ROS libraries and import the Int32 message from the std_msgs package

int main(int argc, char** argv) {

    ros::init(argc, argv, "topic_publisher"); // Initiate a Node named 'topic_publisher'
    // This the name for the parameter 'name=' in the launch file
    ros::NodeHandle nh;

    // Create a Publisher object, that will publish on the /counter topic messages of type Int32
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("counter", 1000);
    ros::Rate loop_rate(2); // Set a publish rate of 2 Hz

    std_msgs::Int32 count; // Create a variable of type Int32
    count.data = 0; // Initialize 'count' variable

    while (ros::ok()) // Create a loop that will go until stopped
    {
        pub.publish(count); // Publish the message within the 'count' variable
        ros::spinOnce();
        loop_rate.sleep(); // Make sure the publish rate maintains at 2 Hz
        ++count.data; // Increment 'count' variable
    }
    return 0;
}
```
### Launch
```
<launch>
    <node pkg="topic_publisher_pkg" type="simple_topic_publisher" name="topic_publisher" output="screen">
    </node>
</launch>
```
### For the CMakeLists.txt
```
add_executable(simple_topic_publisher src/simple_topic_publisher.cpp)

add_dependencies(simple_topic_publisher ${simple_topic_publisher_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(simple_topic_publisher
   ${catkin_LIBRARIES}
 )
 ```

## Topics
This command will start printing all of the information that is being published into the topic. In this case, you can read just the last message published into a topic with:
```
rostopic echo <topic_name> -n1
```

Finally, you can check the different options that rostopic command has by:
```
rostopic -h
```

## Messages
Messages are defined in .msg files, which are located inside a msg directory of a package.
To get information about a message:
```
rosmsg show <message>
```
Example
```
$ rosmsg show std_msgs/Int32
[std_msgs/Int32]:
int32 data
```
In this case, the Int32 message has only one variable named data of type int32. This Int32 message comes from the package std_msgs, and you can find it in its msg directory. If you want, you can have a look at the Int32.msg file by:
```
roscd std_msgs/msg/
```


## Publishing to the /cmd_vel topic
Investigating the `/cmd_vel` topic
```
$ rostopic info /cmd_vel

Type: geometry_msgs/Twist

Publishers: None

Subscribers:
 * /gazebo (http://10.8.0.1:33185/)
```
Investigating the `geometry_msgs/Twist` message type
```
$ rosmsg show geometry_msgs/Twist

geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

* Make move_robot package
catkin_create_pkg

* move_robot_publisher.cpp
```
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "move_cmd_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Rate loop_rate(2);

    geometry_msgs::Twist vel;
    vel.linear.x = 0.5;
    vel.angular.z = 0.5;

    while (ros::ok()){
        pub.publish(vel);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
```
* move_robot.launch
```
<launch>
    <node pkg="move_robot" type="move_robot_publisher" name="move_cmd_publisher" output="screen">
    </node>
</launch>
```

* CMakeLists.txt
```
add_executable(move_robot_publisher src/move_robot_publisher.cpp)

add_dependencies(move_robot_publisher ${move_robot_publisher_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(move_robot_publisher
 ${catkin_LIBRARIES}
)
```
