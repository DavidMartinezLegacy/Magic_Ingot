# Magic-Ingot
```shell
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh 

roscore
rosrun turtlesim turtlesim_node
```
```shell
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
```
```
git clone https://github.com/ros/ros_tutorials.git
```
```shell
source ~/catkin_ws/devel/setup.bash
roslaunch wpr_simulation wpb_simple.launch #
```
```shell
rosrun rqt_robot_steering rqt_robot_steering #Call rqt control interface
```
