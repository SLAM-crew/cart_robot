sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libeigen3-dev