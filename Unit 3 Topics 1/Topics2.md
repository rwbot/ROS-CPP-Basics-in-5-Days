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


# Publishing a Custom ROS Message
#### First set up new ROS Msg using the "Create Custom ROS Message" Tutorial

* age_msg_publisher.cpp

```
#include <ros/ros.h>
#include <pub_custom_ros_msg/Age.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "publish_age_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<pub_custom_ros_msg::Age>("age_info", 1000);
    ros::Rate loop_rate(2);

    pub_custom_ros_msg::Age age;
    age.years = 5;
    age.months = 10;
    age.days = 21;

    while (ros::ok()){
        pub.publish(age);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
```

* age_msg_pub.launch

```
<launch>
    <node pkg="pub_custom_ros_msg" type="age_msg_publisher" name="publish_age_node" output="screen">
    </node>
</launch>
```

* CMakeLists.txt

```
add_executable(age_msg_publisher src/age_msg_publisher.cpp)

# Include Custom Message as Dependency
add_dependencies(age_msg_publisher pub_custom_ros_msg_generate_messages_cpp)

add_dependencies(age_msg_publisher ${age_msg_publisher_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(age_msg_publisher
  ${catkin_LIBRARIES}
)
```



# Turtlebot Obstacle Avoidance - Pub, Sub and Msg Project

* brain.cpp

```
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

//Declare variables to store velocity
float linX, angZ;

//Callback that evaluates the distance from each side of the robot using the LaserScan data
//and adjust the velocities accordingly
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

    //We print the distance to an obstacle in front of the robot
    ROS_INFO("%f", msg->ranges[360]);

    //If the distance in front > 1 meter, the robot will move forward
    if (msg->ranges[360] > 1){
        linX = 0.1;
        angZ = 0.0;
    }

    //If the distance in front < 1 meter, the robot will turn left
    if (msg->ranges[360] < 1){
        linX = 0.0;
        angZ = 0.2;
    }

    //If the distance to left side of < 0.3 meters, the robot will turn right
    if (msg->ranges[719] < 0.3){
        linX = 0.0;
        angZ = -0.2;
    }

    //If the distance to right side < 0.3 meters, the robot will turn left
    if (msg->ranges[0] < 0.3){
        linX = 0.0;
        angZ = 0.2;
    }
}

int main(int argc, char** argv){

    //Initialize a node of name "brain_node"
    ros::init(argc, argv, "brain_node");
    ros::NodeHandle nh;

    //Initialize subscriber that subscribes to /kobuki/laser/scan
    //and calls the laserCallback upon receipt
    ros::Subscriber sub = nh.subscribe("/kobuki/laser/scan", 1000, laserCallback);

    //Initialize publisher that publishes Twist msgs to the /cmd_vel topic
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Rate loop_rate(2);

    //Declare Twist variable to store velocity
    geometry_msgs::Twist vel;


    while (ros::ok()){
        //Assign velocities chosen from callback
        vel.linear.x = linX;
        vel.angular.z = angZ;
        //Publishes Twist msg on /cmd_vel topic
        pub.publish(vel);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
```

* project.launch

```
<launch>
    <node pkg="topics_project_pkg" type="brain" name="brain_node" output="screen"></node>
</launch>
```

* CMakeLists.txt

```
add_executable(brain src/brain.cpp)

add_dependencies(brain ${brain_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(brain
 ${catkin_LIBRARIES}
)
```
