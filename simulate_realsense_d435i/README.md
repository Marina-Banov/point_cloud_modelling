# simulate_realsense_d435i
This package uses a Turtlebot3 robot with an Intel Realsense D435i depth camera and tries to create pointclouds of Gazebo environments.

## Installation
### Ubuntu 20.0.4
```
sudo apt-get update
sudo apt-get upgrade
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh
chmod 755 ./install_ros2_foxy.sh 
bash ./install_ros2_foxy.sh
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install python3-colcon-common-extensions
sudo apt install ros-foxy-gazebo-* ros-foxy-cartographer ros-foxy-cartographer-ros ros-foxy-navigation2 ros-foxy-nav2-bringup
source ~/.bashrc 
sudo apt install ros-foxy-dynamixel-sdk ros-foxy-turtlebot3-msgs ros-foxy-turtlebot3 ros-foxy-turtlebot3-gazebo
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
source ~/.bashrc 
curl -s https://packagecloud.io/install/repositories/dirk-thomas/colcon/script.deb.sh | sudo bash
cd colcon_ws/
colcon build --symlink-install
source ~/.bashrc
chmod +x ~/Downloads/gazebo.sh
gnome-terminal -e ./Downloads/gazebo.sh
sudo apt install rosbash
sudo apt update
sudo apt-get update
sudo apt install ros-foxy-desktop
sudo apt install rosbash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt autoremove
sudo apt install ros-foxy-hls-lfcd-lds-driver
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations
cd ..
colcon build --symlink-install --parallel-workers 1
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
source ~/.bashrc 
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
ros2 run turtlebot3_teleop teleop_keyboard 
ros2 launch turtlebot3_bringup rviz2.launch.py
cd colcon_ws/src/
git clone -b foxy-devel https://github.com/pal-robotics/realsense_gazebo_plugin
cd realsense_gazebo_plugin/
colcon build
cd ../..
. install/setup.bash 
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/
git clone https://github.com/IntelRealSense/realsense-ros.git -b foxy
cd ..
sudo apt-get install python3-rosdep -y
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build
. install/local_setup.bash
ros2 launch realsense2_description view_model.launch.py model:=test_d435i_camera.urdf.xacro
ros2 launch simulate_realsense_d435i simulate_realsense_d435i.launch.py 
ros2 run xacro xacro test_d435i_camera.urdf.xacro > x.urdf
gz sdf -p x.urdf > robot.sdf
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/colcon_ws/src/simulate_realsense_d435i/models' >> ~/.bashrc
```
