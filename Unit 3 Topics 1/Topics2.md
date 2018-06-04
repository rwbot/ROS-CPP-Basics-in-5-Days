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






































z
