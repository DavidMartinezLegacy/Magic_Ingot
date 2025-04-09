# Magic-Ingot

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
-Terminator:
Ctrl + Shift + E   Horizontal split screen
Ctrl + Shift + O   Vertical split screen
Ctrl + Shift + W   Cancel split screen
Alt + ↑ ↓ ← →      Move focus
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
```shell
git checkout noetic-devel
rosrun turtlesim turtlesim_node
roslaunch wpr_simulation wpb_simple.launch #
rosrun rqt_robot_steering rqt_robot_steering #Call rqt control interface
```
- References:<br>
[ROS Index](https://index.ros.org/) <br>
[Ubuntu install of ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)<br>
