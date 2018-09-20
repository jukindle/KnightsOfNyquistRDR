# KnightsOfNyquistRDR

## Installation

### Install ROS
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    
    sudo apt-get update
    
    sudo apt-get install ros-kinetic-desktop-full
    
    sudo rosdep init
    rosdep update
    
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

### Install Python control
TODO: this library will throw an error later, just do sudo apt install python-<whatever name> or google it
    

### Install this repository

    git clone <repo_URL> from top right of this page
    
    cd <Folder of repo which was created>
    
    git checkout vel-control
    
    git pull
    
    git submodule update --init
    
    source environment.sh
    
    catkin_make -C catkin_ws

### Install the simulator

    source <(curl -p https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_common_deps.sh)
    
    

## Usage

Every time you open a new terminal, type (inside the folder)

    source environment.sh
    
 
### Launch recording

    roslaunch launchfiles record.launch


### Launch following

    roslaunch launchfiles master.launch
    
### Launch simulator
Go to PXç/Firmware folder and execute:
    
    make posix_sitl_default jmavsim

