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

 ## Here go all the packages needed to COMPILE the messages of topic, services and actions.
 ## Its only geting its paths, and not really importing them to be used in the compilation.
 ## Its only for further functions in CMakeLists.txt to be able to find those packages.
 ## In package.xml you have to state them as build
 find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   message_generation
 )

 ## Generate topic messages in the 'msg' folder
 ## In this function will be placed all the topic messages files of this package ( in the msg folder ) to be compiled.
 add_message_files(
   FILES
   Age.msg
 )

 ## Here is where the packages needed for the topic messages compilation are imported.
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

 ## State here all the packages that will be needed by someone that executes something from your package.
 ## All the packages stated here must be in the package.xml as run_depend
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
roscd
cd ..
catkin_make
source devel/setup.bash
```


6. ##### Use in CPP Files by adding the following to CMakeLists.txt

```
add_dependencies(publish_age topic_ex_generate_messages_cpp)
```
