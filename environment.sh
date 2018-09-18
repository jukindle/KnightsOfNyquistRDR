source /opt/ros/kinetic/setup.sh

export ROS_HOSTNAME=$HOSTNAME.local


export R2D2LIDAR_ROOT=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

export PYTHONPATH=$R2D2LIDAR_ROOT/catkin_ws/src:$PYTHONPATH

source $R2D2LIDAR_ROOT/catkin_ws/devel/setup.bash
