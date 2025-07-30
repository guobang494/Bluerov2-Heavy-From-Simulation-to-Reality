## Installation 
## Reference https://github.com/HKPolyU-UAV/bluerov2/tree/huyang-backup
## Based on the reference, eight propellers were introduced: bluerov2 -heavy config.


### Software version:
* Python 3.7
* ROS ([ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) recommended)
* [uuv simulator](https://uuvsimulator.github.io/)
* [Acados](https://docs.acados.org/installation/index.html)
  
### 1) Install Acados

    git clone https://github.com/acados/acados.git
    cd acados
    git submodule update --recursive --init
    mkdir -p build
    cd build
    cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> ..
    make install -j4

    pip install -e ~/acados/interfaces/acados_template

    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/root/acados/lib"' >> ~/.bashrc 
    echo 'export ACADOS_SOURCE_DIR="/root/acados"' >> ~/.bashrc
    source ~/.bashrc

   
### 2) Install ros uuv-simulator packages

    sudo apt-get install -y ros-noetic-geodesy
    mkdir -p ~/xxx_ws/src && \
    cd ~/catkin_ws/src && \
    git clone --branch noetic https://github.com/arturmiller/uuv_simulator.git
  

### 3) GIt clone

    cd ~/xxx_ws/src && \
    git clone --branch huyang-backup https://github.com/HKPolyU-UAV/bluerov2.git


### 4) Set the Acados Path 
    cd xxx_ws/src/dobmpc/CMakelists.txt
    change  line set(acados_include "~/acados/include") and set(acados_lib "~/acados/lib") to your path

### 4) Catkin_make
    cd ~/xxx_ws/src/bluerov2/bluerov2_dobmpc/scripts && \
    yes | python3 generate_c_code.py
    cd ~/xxx_ws && catkin_make

### 5) Launch
    cd ~/xxx_ws
    source devel/setup.bash
    roslaunch  ~/xxx_ws/src/bluerov2/bluerov2_dobmpc/launch/start_dob_tank.launch

    Creat the path
    cd ~/xxx_ws/src && \source devel/setup.bash
    cd ~/xxx_ws/src/bluerov2/bluerov2_dobmpc/scripts
    python3 export_and_plot_trajectory.py