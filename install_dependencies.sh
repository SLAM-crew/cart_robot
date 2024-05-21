sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libeigen3-dev
sudo apt install ros-noetic-octomap ros-noetic-octomap-mapping
sudo apt install ros-noetic-usb-cam
sudo apt install ros-noetic-navigation
sudo pip3 install imutils
sudo pip3 install opencv-python
sudo pip3 install opencv-contrib-python
sudo apt install ros-noetic-rplidar-ros