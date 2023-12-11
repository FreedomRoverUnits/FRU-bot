# ROS2 Package for Freedom Rover Unit Host Development
![linorobot2](docs/linorobot2.gif)

## Description
Fork of [linorobot2](https://github.com/linorobot/linorobot2), a ROS2 package that facilitates development of 2/4 wheel-differential-drive and mecanum wheel drive robots built from accessible parts. This repository holds FRU's modifications that are proprietary to FRU's robot: [lds-01](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver) 2D lidar, MPU 6050, TT motors, dual motor driver interface, custom 3D printed [chassis](https://www.printables.com/en/model/355730-two-wheeled-robot-chassis/files), and ESP32-DevKitC-32UE. Agent is intended to run on a remote docker container communicating wirelessly with esp32.   

## Installation
1. Source ros distro
   ```
   source /opt/ros/<your_ros_distro>/setup.bash
   cd <your_ws>
   ```

2. Cloning the repository
   ```
   git clone https://github.com/FreedomRoverUnits/linorobot2_FRU src/linorobot2
   ```
3. Install linorobot2 dependencies
   ```
   cd <your_ws>
   rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent
   colcon build
   source install/setup.bash
   ```
4. Export robot base type.
   ```
   echo "export LINOROBOT2_BASE=2wd" >> ~/.bashrc
   source ~/.bashrc
   ```
5. Build Step
   ```
   colcon build
   source install/setup.bash
   ```
6. RViz Configuration installation.
   ```
   cd <host_machine_ws>
   git clone https://github.com/linorobot/linorobot2_viz src/linorobot2_viz
   rosdep update && rosdep install --from-path src --ignore-src -y 
   colcon build
   source install/setup.bash
   ```

## Quickstart
1. Connect to robot.
2. Start micro-ros agent via docker in another terminal.
   ```
   source /opt/ros/<your_ros_distro>/setup.bash
   sudo docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
   ```

3. Start EKF node and joint state publisher, set rviz flag to true to visualize robot description.
   ```
   ros2 launch linorobot2_bringup bringup_FRU.launch.py rviz:=true
   ```
4. Control the robot manually via teleop twist commands.
   ```
   python3 teleop_twist_keyboard_FRU.py
   ```

## Creating a map. 

1. Run SLAM toolbox.
   ```
   ros2 launch linorobot2_navigation slam.launch.py rviz:=true
   ``` 
2. Move the robot to build the map, and save it.
   ```
   cd linorobot2/linorobot2_navigation/maps
   ros2 run nav2_map_server map_saver_cli -f <map_name> --ros-args -p save_map_timeout:=10000.
   ```

## Autonomous Navigation.
1. First change MAP_NAME in navigation.launch.py to the name of the map you created. Or change it dynamically.
   ```
   cd <robot_ws>
   colcon build

   ros2 launch linorobot2_navigation navigation.launch.py map:=<path_to_map_file>/<map_name>.yaml
   ```
2. Run Nav2 package.
   ```
   ros2 launch linorobot2_navigation navigation.launch.py
   ```

Optional parameter for loading maps:

* map - Path to newly created map <map_name.yaml>.
  
Optional parameters for simulation on host machine:

* sim - Set to true for simulated robots on the host machine. Default value is false.
* rviz - Set to true to visualize the robot in RVIZ. Default value is false.

navigation.launch.py will continue to throw this error - 'Timed out waiting for transform from base_link to map to become available, tf error: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist' until the robot's pose has been initialized.
- Refer to [Nav2](https://navigation.ros.org/tutorials/docs/navigation2_on_real_turtlebot3.html#initialize-the-location-of-turtlebot-3) tutorial for more info.
## Troubleshooting Guide (Directly from [linorobot2](https://github.com/linorobot/linorobot2))
#### 1. The changes I made on a file are not taking effect on the package configuration/robot's behavior.
- You need to build your workspace every time you modify a file:

   ```
   cd <ros2_ws>
   colcon build
   #continue what you're doing...
   ```

#### 2. [`slam_toolbox]: Message Filter dropping message: frame 'laser'`
- Try to up `transform_timeout` by 0.1 in linorobot2_navigation/config/slam.yaml until the warning is gone.


#### 3. `target_frame - frame does not exist`
- Check your <robot_type>.properties.urdf.xacro and ensure that there's no syntax errors or repeated decimal points.

#### 4. Weird microROS agent behavior after updating the Linux/ROS
- Don't forget to update the microROS agent as well after your updates. Just run:
    
   ```
   bash update_microros.bash
   ```
