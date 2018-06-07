
Solutions for Unit 3 Services Part 2


Index:
Solution Exercise 3.4
Solution Exercise 3.5
Solution Exercise 3.4
The objective of this exercise 3.2 is to create a service that when called, BB8 robot moves in a square like trajectory.
Nothing to comment.

You can work on a new package or use the one you created for exercise 3.1, as you prefer. In this case, we are going to work on the package unit_3_services
Create a C++ file that has a class inside that allows the movement of the BB8 in a square {Fig-3.1}. This class could be called, for reference, MoveBB8. And the C++ code that contains it, could called move_bb8.cpp.
To move BB-8, you just have to write in the /cmd_vel Topic, as you did in Unit1 Topics.
Bear in mind that although this is a simulation, BB-8 has weight and, therefore, it won't stop immediately due to inertia.
Also, when turning, friction and inertia will be playing a role. Bear in mind that by only moving through /cmd_vel, you don't have a way of checking if it turned the way you wanted (it's called an open loop system). Unless, of course, you find a way to have some positional feedback information. That's a challenge for advanced AstroMech builders (if you want to try, think about using the /odom topic).
But for considering the movement Ok, you just have to more or less move in a square, doesnt have to be perfect.
So first of all we create the C++ file move_bb8.cpp in the package unit_3_services.

Here you have the C++ file with the class MoveBB8, that moves the BB8 in a square in open loop.

C++ Program: move_bb8.cpp

```

void changeState(int state, float duration)

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
​
class MoveBB8
{
    // Other variables
    bool running_;
    int state_;
    int rate_hz_;
    float duration_;
    int times_;
​
    // ROS Objects
    ros::NodeHandle nh_;
    ros::Rate *rate_;
​
    // ROS Publishers
    ros::Publisher pub_cmd_vel_;

    public:

        MoveBB8()
        {
            // Other variables
            running_ = false;
            state_ = 0;
            rate_hz_ = 20;
            duration_ = 0;
            times_ = 4 * 1;
​
            // ROS Objects
            rate_ = new ros::Rate(rate_hz_);

            // ROS Publishers
            pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        }

        ~MoveBB8(void)
        {

        }
​
        void rateSleep(void)
        {
            rate_->sleep();
        }
​
        geometry_msgs::Twist getStateVelocity() {
            geometry_msgs::Twist vel;
            switch (state_) {
                case 0:
                    // go ahead
                    vel.linear.x = 0.2;
                    vel.angular.z = 0;
                    break;
                case 1:
                    // stop
                    vel.linear.x = 0;
                    vel.angular.z = 0;
                    break;
                case 2:
                    // turn right
                    vel.linear.x = 0;
                    vel.angular.z = 0.2;
                    break;
                case 3:
                    // stop
                    vel.linear.x = 0;
                    vel.angular.z = 0;
                    break;
            }
            return vel;
        }
​
        void runTimeStateMachine(void)
        {
            geometry_msgs::Twist vel;
            running_ = true;
​
            if (!running_)
            {
                vel.linear.x = 0;
                vel.angular.z = 0;
                pub_cmd_vel_.publish(vel);
                return;
            }
​
            vel = this->getStateVelocity();
​
            pub_cmd_vel_.publish(vel);
​
            duration_ -= 1/(float)rate_hz_;
​
            ROS_INFO("State [%d], Vel[%.2f, %.2f], Duration [%.2f]", state_, vel.linear.x, vel.angular.z, duration_);
​
            if (duration_ <= 0) {
                float state_duration[4] = {2.0, 3.8, 4.0, 0.1};
                int next_state = state_ + 1;
                if (state_ == 3)
                {
                    next_state = 0;
                    times_ -= 1;
                }
                int next_state_duration = state_duration[next_state];
                this->changeState(next_state, next_state_duration);
            }
​
            if (times_ == 0) {
                running_ = false;
                vel.linear.x = 0;
                vel.angular.z = 0;
                pub_cmd_vel_.publish(vel);
            }
        }
​
        void changeState(int state, float duration)
        {
            state_ = state;
            duration_ = duration;
            ROS_INFO("Change to state [%d]", state_);
        }

};
​
int main(int argc, char** argv)
{
  ros::init(argc, argv, "MoveBB8Server");

  MoveBB8 moveBB8;
​
  while (ros::ok())
  {
      moveBB8.runTimeStateMachine();
​
      moveBB8.rateSleep();
​
      ros::spinOnce();
  }

  return 0;
}
```

END C++ Program: move_bb8.cpp

Add a service server that accepts an Empty Service messages and activates the square movement. This service could be called /perform_square.
This activation will be done through a call to the Class MoveBB8.
So the first step is to create a C++ file called bb8_move_in_square_service_server.cpp, which will be a modified version of the previous move_bb8.cpp file.

C++ Program: bb8_move_in_square_service_server.cpp

```
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
​
class MoveBB8
{
    // Other variables
    bool running_;
    int state_;
    int rate_hz_;
    float duration_;
    int times_;
​
    // ROS Objects
    ros::NodeHandle nh_;
    ros::Rate *rate_;
​
    // ROS Services
    ros::ServiceServer srv_perform_square_;
​
    // ROS Publishers
    ros::Publisher pub_cmd_vel_;

    public:

        MoveBB8()
        {
            // Other variables
            running_ = false;
            state_ = 0;
            rate_hz_ = 20;
            duration_ = 0;
            times_ = 0;
​
            // ROS Objects
            rate_ = new ros::Rate(rate_hz_);
​
            // ROS Services
            srv_perform_square_ = nh_.advertiseService("/perform_square", &MoveBB8::my_callback, this);

            // ROS Publishers
            pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        }

        ~MoveBB8(void)
        {

        }
​
        void rateSleep(void)
        {
            rate_->sleep();
        }
​
        geometry_msgs::Twist getStateVelocity() {
            geometry_msgs::Twist vel;
            switch (state_) {
                case 0:
                    // go ahead
                    vel.linear.x = 0.2;
                    vel.angular.z = 0;
                    break;
                case 1:
                    // stop
                    vel.linear.x = 0;
                    vel.angular.z = 0;
                    break;
                case 2:
                    // turn right
                    vel.linear.x = 0;
                    vel.angular.z = 0.2;
                    break;
                case 3:
                    // stop
                    vel.linear.x = 0;
                    vel.angular.z = 0;
                    break;
            }
            return vel;
        }
​
        void runTimeStateMachine(void)
        {
            geometry_msgs::Twist vel;
​
            if (!running_)
            {
                vel.linear.x = 0;
                vel.angular.z = 0;
                pub_cmd_vel_.publish(vel);
                return;
            }
​
            vel = this->getStateVelocity();
​
            pub_cmd_vel_.publish(vel);
​
            duration_ -= 1/(float)rate_hz_;
​
            ROS_INFO("State [%d], Vel[%.2f, %.2f], Duration [%.2f]", state_, vel.linear.x, vel.angular.z, duration_);
​
            if (duration_ <= 0) {
                float state_duration[4] = {2.0, 3.8, 4.0, 0.1};
                int next_state = state_ + 1;
                if (state_ == 3)
                {
                    next_state = 0;
                    times_ -= 1;
                }
                int next_state_duration = state_duration[next_state];
                this->changeState(next_state, next_state_duration);
            }
​
            if (times_ == 0) {
                running_ = false;
                vel.linear.x = 0;
                vel.angular.z = 0;
                pub_cmd_vel_.publish(vel);
            }
        }
​
        void changeState(int state, float duration)
        {
            state_ = state;
            duration_ = duration;
            ROS_INFO("Change to state [%d]", state_);
        }

        bool my_callback(std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res)
        {
            running_ = !running_;
            // once for each side of the square
            times_ = 4 * 1;
            return running_;
        }

};
​
int main(int argc, char** argv)
{
  ros::init(argc, argv, "MoveBB8Server");

  MoveBB8 moveBB8;
​
  while (ros::ok())
  {
      moveBB8.runTimeStateMachine();
​
      moveBB8.rateSleep();
​
      ros::spinOnce();
  }
​
  ros::spin();

  return 0;
}
```
END C++ Program: bb8_move_in_square_service_server.cpp

Remember, in order to generate the C++ executable, just add the following to your CMakeLists.txt file.
```

add_executable(bb8_move_in_square_service_server src/bb8_move_in_square_service_server.cpp)
​
add_dependencies(bb8_move_in_square_service_server ${bb8_move_in_square_service_server_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
​
target_link_libraries(bb8_move_in_square_service_server
   ${catkin_LIBRARIES}
 )
 ```
Once you have everything properly compiled, you can test it through a call in the WebConsole to the service /perform_square.

Execute in WebShell #1


```
rosservice call /perform_square [TAB]+[TAB]
```
You should get BB8 moving like so:



Create a launch file called start_bb8_move_in_square_service_server.launch. Inside it you have to start a node that launches the bb8_move_in_square_service_server.cpp.
So this is the start_bb8_move_in_square_service_server.launch that launches the bb8_move_in_square_service_server.cpp.

Launch Program: start_bb8_move_in_square_service_server.launch

```
<launch>
    <!-- Start Service Server for move_bb8_in_square service -->
    <node pkg="unit_3_services" type="bb8_move_in_square_service_server" name="MoveBB8Server"  output="screen">
    </node>
</launch>
```
END Launch Program: start_bb8_move_in_square_service_server.launch

Test it by launching this launch start_bb8_move_in_square_service_server.launch and calling the service as before. Is the exact same procedure , only that we are launching the service server through a launch file, instead of directly through a C++ executable.

Create a new C++ code, called bb8_move_in_square_service_client.cpp, that calls the service /perform_square. Remember how it was done in Unit3 Services Part1.
Then generate a new launch file, called call_bb8_move_in_square_service_server.launch, that executes the bb8_move_in_square_service_client.cpp through a node.
The bb8_move_in_square_service_client.cpp should be similar to this:

C++ Program: bb8_move_in_square_service_client.cpp
```

#include "ros/ros.h"
#include "std_srvs/Empty.h"
// Import the service message used by the service /perform_square
​
int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_move_bb8_in_square_client"); // Initialise a ROS node
  ros::NodeHandle nh;

  // Create the connection to the service /perform_square
  ros::ServiceClient perform_square_service_client = nh.serviceClient<std_srvs::Empty>("/perform_square");
  std_srvs::Empty srv; // Create an object of type Empty

  if (perform_square_service_client.call(srv))
  {
    ROS_INFO("Service successfully called. Moving BB8 in a square.");
  }
  else
  {
    ROS_ERROR("Failed to call service /perform_square");
    return 1;
  }
​
  return 0;
}
```
END C++ Program: bb8_move_in_square_service_client.cpp

The call_bb8_move_in_square_service_server.launch should be like this:

Launch Program: call_bb8_move_in_square_service_server.launch

```
<launch>
    <!-- Start Service Server for move_bb8_in_square service -->
    <node pkg="unit_3_services"
            type="bb8_move_in_square_service_client" name="service_move_bb8_in_square_client"  output="screen">
    </node>
</launch>
```
Launch Program: call_bb8_move_in_square_service_server.launch

To generate the C++ executable, add the following to your CMakeLists.txt:

```
add_executable(bb8_move_in_square_service_client src/bb8_move_in_square_service_client.cpp)
​
add_dependencies(bb8_move_in_square_service_client ${bb8_move_in_square_service_client_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
​
target_link_libraries(bb8_move_in_square_service_client
   ${catkin_LIBRARIES}
 )
 ```
When launched call_bb8_move_in_square_service_server.launch, bb8 should move in a square.
This should works exactly the same way as the other calls you have performed. But in this case you launch start_bb8_move_in_square_service_server.launch and the in another terminal you launch call_bb8_move_in_square_service_server.launch.

# Solution Exercise 3.5
Upgrade the C++ file move_bb8.cpp so that it can move BB8 in a square of variable size.
As you may have seen in the code, this is already done. The only thing you need to do in order to modify the size of the square BB8 performs, is to change the time that it will move forwards. This can be done in the following line:

```
float state_duration[4] = {2.0, 3.8, 4.0, 0.1}; // It will move forward for 2 seconds
float state_duration[4] = {4.0, 3.8, 4.0, 0.1}; // It will move forward for 4 seconds (bigger square)
```
So basically, we only change the time BB8 is moving forwards to describe a bigger or smaller Square. Of course, without calibration nor closed loop controle, it's very difficult to make it create a perfect size square. But the objective is not that. The objective is that the square changes, it doesn't have to be exact.

Create a new C++ file, called bb8_move_custom_service_server.cpp, modifying the service server that accepts an Empty Service message and activates the square movement that you created in Exercise 3.4. This new service could be called /move_bb8_in_square_custom. This new service will have to use service messages of type BB8CustomServiceMessage defined here:
The first thing is to create the BB8CustomServiceMessage.srv, creating a srv folder inside the package unit_3_services.

Service Message: BB8CustomServiceMessage.srv

```
float64 side       # The distance of each side of the square
int32 repetitions    # The number of times BB-8 has to execute the square movement when the service is called
---
bool success         # Did it achieve it?
END Service Message: BB8CustomServiceMessage.srv
```

We have to also modify the CMakelists.txt and the package.xml, as explained in the notebook, to compile the new service message.

CMakelists.txt


cmake_minimum_required(VERSION 2.8.3)
project(unit_3_services)
​
​
## Here go all the packages needed to COMPILE the messages of topic, services and actions.
## Its only geting its paths, and not really importing them to be used in the compilation.
## Its only for further functions in CMakeLists.txt to be able to find those packages.
## In package.xml you have to state them as build


```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
```

## Generate services in the 'srv' folder
## In this function will be all the action messages of this package ( in the action folder ) to be compiled.
## You can state that it gets all the actions inside the action directory: DIRECTORY action
## Or just the action messages stated explicitly: FILES my_custom_action.action
## In your case you only need to do one of two things, as you wish.
---


```
add_service_files(
  FILES
  BB8CustomServiceMessage.srv
)
```
​



## Here is where the packages needed for the action messages compilation are imported.


```
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

​
## State here all the packages that will be needed by someone that executes something from your package.
## All the packages stated here must be in the package.xml as run_depend

```

catkin_package(
  CATKIN_DEPENDS roscpp
)
​
​
include_directories(
  ${catkin_INCLUDE_DIRS}
)
```

package.xml

```
<?xml version="1.0"?>
<package>
  <name>unit_3_services</name>
  <version>0.0.0</version>
  <description>The unit_3_services package</description>
​
  <maintainer email="user@todo.todo">user</maintainer>
​
  <license>TODO</license>
​
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>message_generation</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>message_runtime</run_depend>
​
​
  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->
  </export>
</package>
```

Once you have it just compile and source in ALL the WebShells that you are going to use so that ROS can find the new Messages.

```
roscd;cd ..
catkin_make
source devel/setup.bash
```
And check it finds the messages:


rossrv list | grep BB8CustomServiceMessage
You should get:


unit_3_services/BB8CustomServiceMessage
Use the data passed to this new /move_bb8_in_square_custom to change the way BB-8 moves.
Depending on the side value, the service must move the BB-8 has to generate a shape of a square based on the side given.
Also, the BB-8 must repeat the shape as many times as indicated in the repetitions variable of the message.
Finally, it must return True if everything went OK in the success variable.
Now it's time to create the bb8_move_custom_service_server.cpp, using the new service messages BB8CustomServiceMessage.

C++ File: bb8_move_custom_service_server.cpp

```
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unit_3_services/BB8CustomServiceMessage.h>
​
class MoveBB8
{
    // Other variables
    bool running_;
    int state_;
    int rate_hz_;
    float duration_;
    int times_;
    float radius_;
    int repetitions_;
​
    // ROS Objects
    ros::NodeHandle nh_;
    ros::Rate *rate_;
​
    // ROS Services
    ros::ServiceServer srv_perform_square_;
​
    // ROS Publishers
    ros::Publisher pub_cmd_vel_;

    public:

        MoveBB8()
        {
            // Other variables
            running_ = false;
            state_ = 0;
            rate_hz_ = 20;
            duration_ = 0;
            times_ = 0;
            radius_ = 0;
            repetitions_ = 0;
​
            // ROS Objects
            rate_ = new ros::Rate(rate_hz_);
​
            // ROS Services
            srv_perform_square_ = nh_.advertiseService("/perform_square", &MoveBB8::my_callback, this);

            // ROS Publishers
            pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        }

        ~MoveBB8(void)
        {

        }
​
        void rateSleep(void)
        {
            rate_->sleep();
        }
​
        geometry_msgs::Twist getStateVelocity() {
            geometry_msgs::Twist vel;
            switch (state_) {
                case 0:
                    // go ahead
                    vel.linear.x = 0.2;
                    vel.angular.z = 0;
                    break;
                case 1:
                    // stop
                    vel.linear.x = 0;
                    vel.angular.z = 0;
                    break;
                case 2:
                    // turn right
                    vel.linear.x = 0;
                    vel.angular.z = 0.2;
                    break;
                case 3:
                    // stop
                    vel.linear.x = 0;
                    vel.angular.z = 0;
                    break;
            }
            return vel;
        }
​
        void runTimeStateMachine(void)
        {
            geometry_msgs::Twist vel;
​
            if (!running_)
            {
                vel.linear.x = 0;
                vel.angular.z = 0;
                pub_cmd_vel_.publish(vel);
                return;
            }
​
            vel = this->getStateVelocity();
​
            pub_cmd_vel_.publish(vel);
​
            duration_ -= 1/(float)rate_hz_;
​
            ROS_INFO("State [%d], Vel[%.2f, %.2f], Duration [%.2f]", state_, vel.linear.x, vel.angular.z, duration_);
​
            if (duration_ <= 0) {
                float state_duration[4] = {radius_, 3.8, 4.0, 0.1};
                int next_state = state_ + 1;
                if (state_ == 3)
                {
                    next_state = 0;
                    times_ -= 1;
                }
                int next_state_duration = state_duration[next_state];
                this->changeState(next_state, next_state_duration);
            }
​
            if (times_ == 0) {
                running_ = false;
                vel.linear.x = 0;
                vel.angular.z = 0;
                pub_cmd_vel_.publish(vel);
            }
        }
​
        void changeState(int state, float duration)
        {
            state_ = state;
            duration_ = duration;
            ROS_INFO("Change to state [%d]", state_);
        }

        bool my_callback(unit_3_services::BB8CustomServiceMessage::Request &req,
                        unit_3_services::BB8CustomServiceMessage::Response &res)
        {
            running_ = !running_;
            radius_ = req.radius;
            repetitions_ = req.repetitions;
            times_ = 4 * repetitions_;
            return running_;
        }

};
​
int main(int argc, char** argv)
{
  ros::init(argc, argv, "MoveBB8Server");

  MoveBB8 moveBB8;
​
  while (ros::ok())
  {
      moveBB8.runTimeStateMachine();
​
      moveBB8.rateSleep();
​
      ros::spinOnce();
  }
​
  ros::spin();

  return 0;
}
```
END C++ File: bb8_move_custom_service_server.cpp

As you can see in the code above, we are receiving the side and repetitions values from the request message:

```
bool my_callback(unit_3_services::BB8CustomServiceMessage::Request &req,
                        unit_3_services::BB8CustomServiceMessage::Response &res)
        {
            running_ = !running_;
            // once for each side of the square
            radius_ = req.radius;
            repetitions_ = req.repetitions;
            times_ = 4 * repetitions_;
            return running_;
        }
```
Then, we are just using these values to set the number of repetitions:


times_ = 4 * repetitions_;
and to set the radius of the square:


float state_duration[4] = {radius_, 3.8, 4.0, 0.1};
And that's it! Let's continue.

Create a new launch called start_bb8_move_custom_service_server.launch that launches the server launched in the C++ file bb8_move_custom_service_server.cpp.
Test that when calling this new service /move_bb8_in_square_custom, BB8 moves accordingly.
This start_bb8_move_custom_service_server.launch is the same as the one made in Exercise 3.4, changing the C++ executable launched to bb8_move_custom_service_server.

```
<launch>
    <!-- Start Service Server for move_bb8_in_square service -->
    <node pkg="unit_3_services"
            type="bb8_move_custom_service_server" name="service_move_bb8_in_square_custom_server"  output="screen">
    </node>
</launch>
```
Create a new service client that calls the service /move_bb8_in_square_custom and makes BB8 move in a small square twice and in a big square once. It could be called bb8_move_custom_service_client.cpp and the launch that starts it call_bb8_move_in_square_custom_service_server.launch.
We first create the bb8_move_custom_service_client.cpp that will execute a call to perform the two small squares and one big square.

C++ File: bb8_move_custom_service_client.cpp

```
#include "ros/ros.h"
#include "unit_3_services/BB8CustomServiceMessage.h"
// Import the service message used by the service /perform_square
​
int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_move_bb8_in_square_client"); // Initialise a ROS node
  ros::NodeHandle nh;

  // Create the connection to the service /perform_square
ros::ServiceClient perform_square_service_client = nh.serviceClient<unit_3_services::BB8CustomServiceMessage>("/perform_square");
  unit_3_services::BB8CustomServiceMessage srv; // Create an object of type Empty
  srv.request.radius = 3.0;
  srv.request.repetitions = 2;

  if (perform_square_service_client.call(srv))
  {
    ROS_INFO("Service successfully called. Moving BB8 in a square.");
  }
  else
  {
    ROS_ERROR("Failed to call service /perform_square");
    return 1;
  }
​
  return 0;
}
```
END C++ File: bb8_move_custom_service_client.cpp

And now we have to create a launch that launches this C++ node ,called call_bb8_move_in_square_custom_service_server.launch:

Launch File: call_bb8_move_in_square_custom_service_server.launch

```
<launch>
    <!-- Start Service Server for move_bb8_in_square service -->
    <node pkg="unit_3_services"
            type="bb8_move_custom_service_client" name="service_move_bb8_in_square_custom_client"  output="screen">
    </node>
</launch>
```
END Launch File: call_bb8_move_in_square_custom_service_server.launch

In order to generate the C++ executables of this 2 last scripts, add this to the CMakeLists.txt file.

```
add_executable(bb8_move_custom_service_server src/bb8_move_custom_service_server.cpp)
add_dependencies(bb8_move_custom_service_server ${bb8_move_custom_service_server_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bb8_move_custom_service_server
   ${catkin_LIBRARIES}
 )
​
add_executable(bb8_move_custom_service_client src/bb8_move_custom_service_client.cpp)
add_dependencies(bb8_move_custom_service_client ${bb8_move_custom_service_client_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bb8_move_custom_service_client
   ${catkin_LIBRARIES}
 )
 ```
Finally, you just have to test all the pipeline. So you have to launch the server launch in one web shell and the client in the other. It should make the robot move as desired.

Execute in WebShell #1



roslaunch unit_3_services start_bb8_move_custom_service_server.launch
Execute in WebShell #2



roslaunch unit_3_services call_bb8_move_in_square_custom_service_server.launch
You should get something similar to this, but slower, because this has been accelerated for practical reasons:
