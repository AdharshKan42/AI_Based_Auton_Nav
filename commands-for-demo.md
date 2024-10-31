resource : https://github.com/clearpathrobotics/cpr_gazebo

### catkin_make fix :
`catkin_make_isolated -DPYTHON_EXECUTABLE=/usr/bin/python3`

### Before running any ROS commands:
`roscore`

### To add a depth camera to Jackal:
Reference: https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/description.html

Find the jackal_gazebo package path using:
`rospack find jackal_gazebo`
Move to this path:
`cd Path-To-Jackal-Gazebo/launch`
In `...jackal_gazebo/launch/description.launch`, change the default `config` argument to be `front_bumblebee2`
You can now launch Gazebo and should see the camera on top of Jackal.

### to launch gazebo :
`roslaunch cpr_inspection_gazebo inspection_world.launch platform:=jackal`

### to launch rviz and view the robot and its transforms:
`roslaunch jackal_viz view_robot.launch`

### To install the tele-twistop-controller, control with rqt_gui:
`sudo apt install ros-noetic-teleop-twist-keyboard`

### To run the teleop twist keyboard:
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel`

### To view the camera output:
Since the TF tree isn't properly initialized, we cannot view the camera topic in RVIZ.
Instead, we will use  `rqt_image_view`.
`rosrun rqt_image_view rqt_image_view`
Click one of the front camera raw topics (`/front....raw`) and refresh.

### To install the twist mux controller:
`sudo apt install ros-noetic-teleop-twist-joy`

### To launch the GUI interface controller :
`rosrun rqt_gui rqt_gui -s rqt_robot_steering`
