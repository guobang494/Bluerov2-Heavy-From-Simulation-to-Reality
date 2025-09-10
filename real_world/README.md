This file mainly foucus on the real_world


## Install Mavros

sudo apt-get update

sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

sudo bash ./install_geographiclib_datasets.sh






## launch Mavros

roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@192.168.2.2:14555  （set to your own config）

Check the state

rostopic echo /mavros/state

## Lauch the function (MPC path tracing)


cd ~/workspace/real_world

souce./devel/setup.bash


roslaunch test_velocity.launch

