# Services

**Services are Synchronous.** When your ROS program calls a service, your program can't continue until it receives a result from the service.

**Actions are Asynchronous.** It's like launching a new thread. When your ROS program calls an action, your program can perform other tasks while the action is being performed in another thread.


* Getting info about a service
```
rosservice info /name_of_your_service
```
Example
```
user ~ $ rosservice info /execute_trajectory
Node: /iri_wam_reproduce_trajectory
URI: rosrpc://ip-172-31-17-169:35175
Type: iri_wam_reproduce_trajectory/ExecTraj
Args: file
```

**Node:** It states the node that provides (has created) that service.

**Type:** It refers to the kind of message used by this service. It has the same structure as topics do. It's always made of `Name_of_Package` / `Name_of_Service_Message`. In this case, the package is `iri_wam_reproduce_trajectory`, and the file where the Service Message is defined is called `ExecTraj`.

**Args:** Here you can find the arguments that this service takes when called. In this case, it only takes a trajectory file path stored in the variable called file.


### Launching a Service

* Calling a launch file inside a launch file (Launch_File_Ception)

```
<launch>
    <include file="$(find $PKG_NAME)/launch/$LAUNCH_FILE.launch"/>
    #EXAMPLE
    <include file="$(find iri_wam_reproduce_trajectory)/launch/start_service.launch"/>
</launch>
```

### Calling a Service Manually from Terminal

```
rosservice call /the_service_name TAB-TAB
```

Example
```
rosservice call /gazebo/delete_model [TAB]+[TAB]
```
gives
```
rosservice call /gazebo/delete_model "model_name: '$NAME'"
```

To do this, we first need to get the list of objects in the scene
```
user:~$ rostopic echo /gazebo/model_states -n1
name: ['ground_plane', 'kinect', 'bowl_2', 'bowl_1', 'unit_box_1', 'iri_wam']
pose:
    ...(the poses of each one of the models in the list in order)
```
---

## Using a Service Within a Node
* simple_service_client.cpp

```
#include "ros/ros.h"
#include "gazebo_msgs/DeleteModel.h"
// Import the service message used by the service /gazebo/delete_model

int main(int argc, char **argv){

    // Initialise a ROS node with the name service_client
    ros::init(argc, argv, "service_client");
    ros::NodeHandle nh;

    // Create the connection to the service /gazebo/delete_model
    ros::ServiceClient delete_model_service = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

    // Create an object of type DeleteModel
    gazebo_msgs::DeleteModel srv;

    // Fill the variable model_name of this object with the desired value
    srv.request.model_name = "bowl_1";

    // Send through the connection the name of the object to be deleted by the service
    if (delete_model_service.call(srv)){
        // Print the result given by the service called
        ROS_INFO("%s", srv.response.status_message.c_str());
    }
    else{
        ROS_ERROR("Failed to call service delete_model");
        return 1;
    }
    return 0;
}
```


## Finding out the Structure of the Service Message used by the Service

You can do a rosservice info to know the type of service message that it uses.

```
rosservice info /name_of_the_service
```

This will return you the `Name_of_Package`/`Name_of_Service_Message`

Then, you can explore the structure of that service message with the following command:

```
rossrv show name_of_the_package/Name_of_Service_message
```

* `rossrv` is similar to `rosmsg`, where Service Messages are messages of the extension `.srv`
* Service Messages are defined inside a `srv` directory rather than `msg`

Example:
```
$ rosservice info /gazebo/delete_model

Node: /gazebo
URI: rosrpc://ip-172-31-35-239:50272
Type: gazebo_msgs/DeleteModel
Args: model_name
```

```
$ rossrv show gazebo_msgs/DeleteModel

string model_name
---
bool success
string status_message
```

##### Service Messages have TWO Parts:

```
REQUEST
---
RESPONSE
```

In the case of the `DeleteModel` service, **REQUEST** contains a string called `model_name` and **RESPONSE** is composed of a boolean named `success`, and a string named `status_message`.

The Number of elements on each part of the service message can vary depending on the service needs. You can even put none if you find that it is irrelevant for your service. The important part of the message is the three dashes ---, because they define the file as a Service Message.

##### Summarizing:

The **REQUEST** is the part of the service message that defines HOW you will do a call to your service. This means, what variables you will have to pass to the Service Server so that it is able to complete its task.

The **RESPONSE** is the part of the service message that defines HOW your service will respond after completing its functionality. If, for instance, it will return an string with a certain message saying that everything went well, or if it will return nothing, etc...


## Exercise 3.2

* Finding the name of the Service and the syntax of the SRV Message

```
user:~$ rosservice info /execute_trajectory
Node: /iri_wam_reproduce_trajectory
URI: rosrpc://10.8.0.1:41659
Type: iri_wam_reproduce_trajectory/ExecTraj
Args: file
```
```
user:~$ rossrv show iri_wam_reproduce_trajectory/ExecTraj
string file
---
*Intentionally Blank because this srv has no response component*
```

**Service** = `/execute_trajectory`

**SRV.Request** = `string file`

**SRV.Response** = In this case it has none. So no response will be given when this service is called. It will be what we know as Empty response.

---
##### This `ros::package::getPath` works in the same way as `$(find name_of_package)` in the launch files.
##### `trajectory.request.file = ros::package::getPath("iri_wam_reproduce_trajectory") + "/config/get_food.txt";`
**Needs modification to CMakeLists.txt in order to work. See Below**

---


**execute_trajectory.cpp**
```
#include "ros/ros.h"
// Import the service message used by the service /gazebo/delete_model
#include "gazebo_msgs/DeleteModel.h"
// Import the service message used by the service /execute_trajectory
#include "iri_wam_reproduce_trajectory/ExecTraj.h"
// Needed to use the ros::package::getPath function
#include <ros/package.h>

int main(int argc, char **argv){

    /* // Initialise a ROS node with the name service_client
    ros::init(argc, argv, "service_client");
    */
    // Initialise a ROS node with the name execute_trajectory_node
    ros::init(argc, argv, "execute_trajectory_node");
    ros::NodeHandle nh;


    // Search for service path using 'rosservice list | grep $SERVICE_NAME'
    // ROS DOCS
    // ros::ServiceClient client = nh.serviceClient<my_package::Foo>("my_service_name");

    /* // Create the connection to the service /gazebo/delete_model
    ros::ServiceClient delete_model_service = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    // Create an object of type DeleteModel
    gazebo_msgs::DeleteModel srv;
    // Fill the variable model_name of this object with the desired value
    srv.request.model_name = "bowl_1";
    */

    // Create the connection to the service /execute_trajectory
    ros::ServiceClient execute_trajectory_service = nh.serviceClient<iri_wam_reproduce_trajectory::ExecTraj>("/execute_trajectory");
    // Create an object of type ExecTraj
    iri_wam_reproduce_trajectory::ExecTraj trajectory;

    // Fill the variable file of this object with the desired value
    // This ros::package::getPath works in the same way as $(find name_of_package) in the launch files.
    trajectory.request.file = ros::package::getPath("iri_wam_reproduce_trajectory") + "/config/get_food.txt";

    // Send through the connection the name of the object to be deleted by the service
    if (execute_trajectory_service.call(trajectory)){
        // Print the result given by the service called
        ROS_INFO("%s", "Service successfully called. Executing trajectory");
    }
    else{
        ROS_ERROR("Failed to call service execute_trajectory");
        return 1;
    }
    return 0;
}
```


**my_robot_arm_demo.launch**
```
<launch>
    <include file="$(find iri_wam_reproduce_trajectory)/launch/start_service.launch"/>

    <!-- Here will go our C++ code that calls the execute_trajectory service -->
    <node pkg="my_robot_arm_demo" type="execute_trajectory" name="execute_trajectory_node" output="screen">
    </node>
</launch>
```


**CMakeLists.txt**
```
add_executable(execute_trajectory src/execute_trajectory.cpp)

add_dependencies(execute_trajectory ${execute_trajectory_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(execute_trajectory
 ${catkin_LIBRARIES}
)

find_package(catkin REQUIRED COMPONENTS
    iri_wam_reproduce_trajectory
    roscpp
    roslib ## Required for `ros::package::getPath` to work
)
```
