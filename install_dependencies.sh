sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libeigen3-dev
sudo apt-get install ros-noetic-octomap ros-noetic-octomap-mapping
