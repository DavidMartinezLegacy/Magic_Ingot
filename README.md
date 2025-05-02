# Magic_Ingot

- ROS environment testing
```shell
roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
rosrun rqt_graph rqt_graph
```
# Workspace
<img src="https://github.com/OogwayLeonardo/Magic-Ingot/blob/main/ROS%20Structure.jpg" title="ROS%20Structure.jpg">

- Create a workspace
```shell
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
cd ..
catkin_make
```
```
git clone https://github.com/ros/ros_tutorials.git
```
- Environment variables
```shell
source ~/catkin_ws/devel/setup.bash
echo "source ~/ros_workspace/devel/setup.bash" >> ~/.bashrc
gedit/code ~/.bashrc #Modify the bashrc file to add global environment variables
source ~/.bashrc #Check if it is successful
```
- Terminator:<br>
Ctrl + Shift + E   Horizontal split screen<br>
Ctrl + Shift + O   Vertical split screen<br>
Ctrl + Shift + W   Cancel split screen<br>
Alt + ↑ ↓ ← →      Move focus <br>

# Package
- Package Structure：
```
Common files and paths under a package are:
├── CMakeLists.txt #package compilation rules (required)
├── package.xml #package description (required)
├── src/ #Source code file
├── include/ #C++ header file
├── scripts/ #Executable scripts
├── msg/ #Custom message
├── srv/ #Custom service
├── models/ #3D model file
├── urdf/ #urdf file
├── launch/ #launch file
```
```
The CMakeLists.txt and package.xml files are essential for a package. The catkin compilation system must first parse these two files before compiling. These two files define a package.
├── CMakeLists.txt: defines the package name, dependencies, source files, target files and other compilation rules of the package, which is an indispensable component of the package;
├── package.xml: describes the package name, version number, author, dependencies and other information of the package, which is an indispensable component of the package;
├── src/: stores ROS source code, including C++ source code (.cpp) and Python module (.py);
├── include/: stores header files corresponding to C++ source code;
├── scripts/: stores executable scripts, such as shell scripts (.sh) and Python scripts (.py);
├── msg/: stores messages in custom format (.msg);
├── srv/: stores services in custom formats (.srv);
├── models/: stores 3D models of robots or simulation scenes (.sda, .stl, .dae, etc.);
├── urdf/: stores the robot model description (.urdf or .xacro);
├── launch/: stores launch files (.launch or .xml);
Usually ROS files are organized in the above format, which is a common naming convention and is recommended to be followed. Among the above paths, only CMakeLists.txt and package.xml are required, and the rest of the paths are determined by whether the software package needs them.
```

_Package related commands:_

- rospack(Tools for package management):
```shell
rospack help	                   #Show rospack usage
rospack list	                   #List all packages on this machine
rospack depends                  #[package] Displays the package's dependencies
rospack find [package]	         #Locating a package
rospack profile	                 #Refresh the location records of all packages
```
- roscd (The command is similar to the Linux system's cd, but the improvement is that roscd can directly cd to the ROS software package):
```shell
roscd [pacakge]	                 #Go to the path where the ROS package is located
```
- rosls(It can be regarded as an improved version of the Linux command ls, which can directly ls the contents of the ROS package):
```shell
rosls [pacakge]	                 #List the files under the ROS package
```
```shell
git checkout noetic-devel
rosrun turtlesim turtlesim_node
roslaunch wpr_simulation wpb_simple.launch #
rosrun rqt_robot_steering rqt_robot_steering #Call rqt control interface
```
- The address of the common dependency package
```shell
/opt/ros/melodic/share
```
- Demo.cpp
```cpp
#include"ros/ros.h"
#include"std_msgs/String.h"
#include"sstream"

int main(int argc,char **argv)
{
  ros::init(argc,argv,"Demo");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",1000);
  ros::Rate loop_rate(10);
  int count = 0;

   while(ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;

    ss<<"Hello, World!" <<count;
    msg.data = ss.str();

    ROS_INFO("%s",msg.data.c_str());
    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
    return 0;
}
```
```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <termios.h>

static float linear_vel = 0.1;
static float angular_vel = 0.1;
static int k_vel = 3;

int GetCh()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON);
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  int c = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_vel_cmd");

  ros::NodeHandle n;
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = 0;
  base_cmd.linear.y = 0;
  base_cmd.angular.z = 0;

  while(n.ok())
  {
    int cKey = GetCh();
    if(cKey=='w')
    {
      base_cmd.linear.x += linear_vel;
      if(base_cmd.linear.x > linear_vel*k_vel)
        base_cmd.linear.x = linear_vel*k_vel;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",base_cmd.linear.x,base_cmd.linear.y,base_cmd.angular.z);
    } 
    else if(cKey=='s')
    {
      base_cmd.linear.x += -linear_vel;

      if(base_cmd.linear.x < -linear_vel*k_vel)
        base_cmd.linear.x = -linear_vel*k_vel;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",base_cmd.linear.x,base_cmd.linear.y,base_cmd.angular.z);
    } 
    else if(cKey=='a')
    {
      base_cmd.linear.y += linear_vel;
      if(base_cmd.linear.y > linear_vel*k_vel)
        base_cmd.linear.y = linear_vel*k_vel;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",base_cmd.linear.x,base_cmd.linear.y,base_cmd.angular.z);
    }
    else if(cKey=='d')
    {
      base_cmd.linear.y += -linear_vel;
      if(base_cmd.linear.y < -linear_vel*k_vel)
        base_cmd.linear.y = -linear_vel*k_vel;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",base_cmd.linear.x,base_cmd.linear.y,base_cmd.angular.z);
    } 
    else if(cKey=='q')
    {
      base_cmd.angular.z += angular_vel;
      if(base_cmd.angular.z > angular_vel*k_vel)
        base_cmd.angular.z = angular_vel*k_vel;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",base_cmd.linear.x,base_cmd.linear.y,base_cmd.angular.z);
    } 
    else if(cKey=='e')
    {
      base_cmd.angular.z += -angular_vel;
      if(base_cmd.angular.z < -angular_vel*k_vel)
        base_cmd.angular.z = -angular_vel*k_vel;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",base_cmd.linear.x,base_cmd.linear.y,base_cmd.angular.z);
    } 
    else if(cKey==' ')
    {
      base_cmd.linear.x = 0;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",base_cmd.linear.x,base_cmd.linear.y,base_cmd.angular.z);
    } 
    else if(cKey=='x')
    {
      base_cmd.linear.x = 0;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0;
      cmd_vel_pub.publish(base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n",base_cmd.linear.x,base_cmd.linear.y,base_cmd.angular.z);
      printf("quit！ \n");
      return 0;
    } 
    else
    {
       printf(" - Undefined instruction \n");
    }
    
  }
  return 0;
}
```
# SLAM
- bobac3_slam
```shell
roslaunch bobac3_slam bobac3_slam_sim.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
rosrun map_server map_saver -f demo
```
# Navigation

<img src="https://github.com/OogwayLeonardo/Magic-Ingot/blob/main/overview_tf.png" title="overview_tf.png">

```shell
sudo apt-get install ros-melodic-dwa-local-planner
cd catkin_ws/src/bobac3_navigation/launch
code map_server.launch
```
```HTML
<launch>
  <arg name="map_file"  default="reicom"/> <!--Mapfile_name-->
  <!--  ****** Maps *****  --> 
  <node name="map_server" pkg="map_server" type="map_server" 
    args="$(find bobac3_navigation)/maps/$(arg map_file).yaml">
    <param name="frame_id" value="map"/>
  </node>
</launch>
```
```
roslaunch bobac3_navigation demo_nav_2d.launch
```
# Voice System
- Voice Collection
```shell
sudo apt install libasound2-dev
sudo apt-get install sox
cd ~/catkin_ws/src
catkin_create_pkg bobac3_audio actionlib robot_audio  move_base_msgs roscpp
cd ~/catkin_ws/src/bobac3_audio/src
touch collect.cpp
```
```cpp
#include <ros/ros.h>
#include <robot_audio/Collect.h>
#include <iostream>
#include <string>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"collect"); //Initialize Node
    ros::NodeHandle n; //Node handle
    ros::ServiceClient collect_client = n.serviceClient<robot_audio::Collect>("voice_collect"); //Defining the Client
    robot_audio::Collect srv; //Defining a message
    srv.request.collect_flag = 1;
    ros::service::waitForService("voice_collect"); //Waiting for the service to start
    collect_client.call(srv); //Send Message
    ROS_INFO("File saved in : %s",srv.response.voice_filename.c_str());
    std::string dir = "play "+srv.response.voice_filename; //Edit as system command
    sleep(1);
    system(dir.c_str()); //Playing an audio file
}
```
```
cd ~/catkin_ws/src/bobac3_audio
code CMakeLists.txt
```
```txt
add_executable(collect_node src/collect.cpp)
add_dependencies(collect_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(collect_node${catkin_LIBRARIES})
```
```shell
cd ~/catkin_ws
catkin_make
cd ~/catkin_ws/src/bobac3_audio
mkdir launch
cd launch
touch voice_collect.launch
```
```HTML
<launch>
    <!-- Open the Experiment Node -->
    <node pkg="bobac3_audio" type="collect_node" name="collect" output="screen"/>
    <!-- Open the voice collection node -->
    <node name="voice_collect" pkg="robot_audio" type="voice_collect_node" output="screen">
        <!-- Audio file directory -->
        <param name="audio_file" type="string" value="./AIUI/audio/audio.wav"/>
    </node>
</launch>
```
```shell
roslaunch bobac3_audio voice_collect.launch
```
- Voice Dictation
```shell
cd ~/catkin_ws/src/bobac3_audio/src
touch dictation.cpp
code dictation.cpp
```
```cpp
#include <ros/ros.h>
#include <robot_audio/robot_iat.h>
#include <robot_audio/Collect.h>
#include <string>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "dictation");
    ros::NodeHandle n;
    ros::ServiceClient iat_client = n.serviceClient<robot_audio::robot_iat>("voice_iat");
    ros::ServiceClient collect_client = n.serviceClient<robot_audio::Collect>("voice_collect");
    
    robot_audio::Collect coll_srv; //Create a voice collection service instance
    coll_srv.request.collect_flag = 1; //Create a voice collection service instance求
    ros::service::waitForService("voice_collect");
    collect_client.call(coll_srv);
    std::cout<<"Voice collection ends:"<<coll_srv.response.voice_filename<<std::endl;

    robot_audio::robot_iat iat_srv; //Create a voice dictation service instance
    iat_srv.request.audiopath = coll_srv.response.voice_filename;
    ros::service::waitForService("voice_iat");
    iat_client.call(iat_srv);

    std::cout<< "What you hear:" << iat_srv.response.text <<std::endl;
    return 0;
}
```
```shell
cd ~/catkin_ws/src/bobac3_audio
code CMakeLists.txt
```
```txt
add_executable(dictation_node src/dictation.cpp)
add_dependencies(dictation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dictation_node${catkin_LIBRARIES})
```
```shell
cd ~/catkin_ws
catkin_make
cd ~/catkin_ws/src/bobac3_audio/launch
touch voice_dictation.launch
```
```HTML
<launch>
    <!-- Experimental Node -->
    <node pkg="bobac3_audio" type="dictation_node" name="dictation" output="screen"/>
    <!-- Open the voice collection node -->
    <node name="voice_collect" pkg="robot_audio" type="voice_collect_node" output="screen">
        <!-- Audio file directory -->
        <param name="audio_file" type="string" value="./AIUI/audio/audio.wav"/>
    </node>
    <!-- Enable Voice Service -->
    <node pkg="robot_audio" type="voice_aiui_node" name="voice_aiui_node"/>
```
```shell
roslaunch bobac3_audio voice_dictation.launch
```
# Ultimate Version
```
cd bobac3 && catkin_make
source ~/bobac3_ws/devel/setup.bash
roslaunch nav_goal demo.launch
```
- References:<br>
[ROS Index](https://index.ros.org/) <br>
[ROS-Tutorial](https://tr-ros-tutorial.readthedocs.io/)<br>
[Ubuntu install of ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)<br>
[ROS-teleop](https://github.com/ros-teleop)<br>
[Navigation](http://wiki.ros.org/navigation)
