# How to Give a Service

**simple_service_server.cpp**

**req** accesses the request, **res** accesses the response
```
#include "ros/ros.h"
#include "std_srvs/Empty.h"
// Import the service message header file generated from the Empty.srv message

// We define the callback function of the service
bool my_callback(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){  

    // res.some_variable = req.some_variable + req.other_variable;

    // We print an string whenever the Service gets called
    ROS_INFO("My_callback has been called");
    return true;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "service_server");
    ros::NodeHandle nh;

    // create the Service called my_service with the defined callback
    ros::ServiceServer my_service = nh.advertiseService("/my_service", my_callback);

    // mantain the service open
    ros::spin();
    return 0;
}
```
##### Note that, in the example, there is a commented line in `my_callback` function. That commented line also shows you how you would return the RESPONSE of the service. For that, you have to access the variables in the RESPONSE part of the message. It would be like this: `res.variables_in_the_response_part_of_srv_message`. That gives you an example of how you would access the REQUEST given by the caller of your service. It's always `req.variables_in_the_request_part_of_srv_message`. In this case this is not necessary, because we are working with the Empty message, which is an special message that doesn't contain anything.
##### Well, for the case of `DeleteModel`, you were passing to the Service Server the name of the object to delete in a variable called `model_name`. So. if you wanted to access the value of that `model_name` variable in the Service Server, you would have to do it like this:
```
req.model_name
```


















The objective of this exercise is to create a service that when called, makes BB8 robot move in a square like trajectory.

You can work on a new package or use one of the ones you have already created.

Create a C++ file that has a class inside. This class has to allow the movement of the BB-8 in a square like movement {Fig-3.1}. This class could be called, for reference, MoveBB8. And the C++ file that contains it, could be called move_bb8.cpp.

To move the BB-8 robot, you just have to write into the /cmd_vel topic, as you did in the Topics Units.

Bear in mind that although this is a simulation, BB-8 has weight and, therefore, it won't stop immediately due to inertia.

Also, when turning, friction and inertia will be playing a role. Bear in mind that by only moving through /cmd_vel, you don't have a way of checking if it turned the way you wanted (it's called an open loop system). Unless, of course, you find a way to have some positional feedback information. That's a challenge for advanced AstroMech builders (if you want to try, think about using the /odom topic).

But for considering the movement Ok, you just have to perform more or less move a square, it doesn't have to be perfect.

Add a Service Server that accepts an Empty Service message and activates the square movement. This service could be called /move_bb8_in_square

This activation will be done through a call to the Class that you just have generated, called MoveBB8.

For that, you have to create a very similar C++ file as simple_service_server.cpp. You could call it bb8_move_in_square_service_server.cpp.

Create a launch file called start_bb8_move_in_square_service_server.launch. Inside it you have to start a node that launches the bb8_move_in_square_service_server.cpp.

Launch start_bb8_move_in_square_service_server.launch and check that when called through the WebShell, BB8 moves in a square.

Create a new C++ file, called bb8_move_in_square_service_client.cpp, that calls the service /move_bb8_in_square. Remember how it was done in the previous Chapter: Services Part1.
Then, generate a new launch file, called call_bb8_move_in_square_service_server.launch, that executes the code in thebb8_move_in_square_service_client.cpp file.

Finally, when you launch this call_bb8_move_in_square_service_server.launch file, BB-8 should move in a square.

























































z
