resource : https://github.com/clearpathrobotics/cpr_gazebo

### ~~catkin_make fix~~ Not necessary anymore if you follow Rover VM setup steps :
`catkin_make_isolated -DPYTHON_EXECUTABLE=/usr/bin/python3`

### Before running any ROS commands:
```
roscore
```

## Package Setup and Build
### Install ROS dependencies:
```
rosdep update
```

Run `rosdep install` outside of src directory
```
rosdep install --from-paths src --ignore-src -r
```

### Build ROS packages
Run this outside `build`, `devel`, `src` in `catkin_ws`:
```
catkin_make
```

### Before running any ROS command
Make sure to run this in your terminal:
```
source devel/setup.zsh 
```
or 
```
source devel/setup.bash
```
depending on your shell.

## Running Commands
### To add a Realsense depth camera to Jackal:
Reference: https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/description.html

Run this in any terminal used to launch the simulation:
```
export JACKAL_URDF_EXTRAS=<path-to-jackal_realsense.urdf.xacro>
```

### To launch gazebo:  
```
roslaunch cpr_inspection_gazebo inspection_world.launch platform:=jackal
```  

### To launch rviz and view the robot and its transforms:
```
roslaunch jackal_viz view_robot.launch
```

### To install the tele-twistop-controller, control with rqt_gui:
```
sudo apt install ros-noetic-teleop-twist-keyboard
```

### To run the teleop twist keyboard:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
```

### To view the camera output:
Since the TF tree isn't properly initialized, we cannot view the camera topic in RVIZ.
Instead, we will use  `rqt_image_view`.
```
rosrun rqt_image_view rqt_image_view
```
Click one of the front camera raw topics (`/front....raw`) and refresh.

### To install the twist mux controller:
```
sudo apt install ros-noetic-teleop-twist-joy
```

### To launch the GUI interface controller :
```
rosrun rqt_gui rqt_gui -s rqt_robot_steering
```
