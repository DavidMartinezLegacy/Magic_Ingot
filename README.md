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
- References:
[ROS Index]（https://index.ros.org/）
```shell
git checkout noetic-devel
rosrun turtlesim turtlesim_node
roslaunch wpr_simulation wpb_simple.launch #
rosrun rqt_robot_steering rqt_robot_steering #Call rqt control interface
```
