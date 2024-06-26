# Mobile robot

- You need `robot_localization` installed in your workspace
```
git clone -b noetic-devel https://github.com/cra-ros-pkg/robot_localization.git
```

- You need `orb_slam2_ros` installed in your workspace
```
git clone https://github.com/appliedAI-Initiative/orb_slam_2_ros.git
```

- You need `octomap_mapping` installed in your workspace
```
git clone -b kinetic-devel https://github.com/OctoMap/octomap_mapping.git
```

- You need `aruco_detect` installed in your workspace
```
git clone https://github.com/UbiquityRobotics/fiducials.git
```

- Install all required dependencies. Run it from your catkin folder
```
sh src/cart_robot/install_dependencies.sh
```

- Build it
```
catkin build
```

## Generate ArUco markers for Gazebo

- Run it from `cart_robot` folder
```
sh generate_gazebo_models.sh
```