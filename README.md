# Magic-Ingot
```shell
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh 

roscore
rosrun turtlesim turtlesim_node
```
- Workspace
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

# Package Structure：
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
其中定义package的是 CMakeLists.txt 和 package.xml ，这两个文件是package中必不可少的。catkin编译系统在编译前，首先就要解析这两个文件。这两个文件就定义了一个package。
	CMakeLists.txt: 定义package的包名、依赖、源文件、目标文件等编译规则，是package不可少的成分；
	package.xml: 描述package的包名、版本号、作者、依赖等信息，是package不可少的成分；
	src/: 存放ROS的源代码，包括C++的源码和(.cpp)以及Python的module(.py)；
	include/: 存放C++源码对应的头文件；
	scripts/: 存放可执行脚本，例如shell脚本(.sh)、Python脚本(.py)；
	msg/: 存放自定义格式的消息(.msg)；
	srv/: 存放自定义格式的服务(.srv)；
	models/: 存放机器人或仿真场景的3D模型(.sda, .stl, .dae等)；
	urdf/: 存放机器人的模型描述(.urdf或.xacro)；
	launch/: 存放launch文件(.launch或.xml)；
通常ROS文件组织都是按照以上的形式，这是约定俗成的命名习惯，建议遵守。以上路径中，只有 CMakeLists.txt 和 package.xml 是必须的，其余路径根据软件包是否需要来决定。

```shell
git checkout noetic-devel
rosrun turtlesim turtlesim_node
roslaunch wpr_simulation wpb_simple.launch #
rosrun rqt_robot_steering rqt_robot_steering #Call rqt control interface
```
- References:
[ROS Index]（https://index.ros.org/）
