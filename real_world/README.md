To install the ros repo, make sure to have installed the catkin tools. 
In ROS noetic, install them with:

sudo pip3 install git+https://github.com/catkin/catkin_tools.git

or 
sudo apt-get install ros-$ROS_DISTRO-catkin python3-catkin-tools (not tested)

then clone the repo and initialise it:
cd ~/catkin_ws
catkin init
catkin build




For qualisys

install

git clone https://gitlab.laas.fr/gepetto/ros-qualisys.git



export CMAKE_PREFIX_PATH=/opt/openrobots
source /opt/ros/XXXX/setup.bash
cd workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


#xxx is ros version 



source /opt/ros/XXXX/setup.bash
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
source workspace/install/setup.bash
roslaunch ros-qualisys qualisys_bauzil_bringup.launch

