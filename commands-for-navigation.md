### Jackal Navigation stuff :

### For the demo environment :

To launch jackal with the lidar :
```
roslaunch jackal_gazebo jackal_world.launch config:=front_laser
```

To make a map :
```
roslaunch jackal_navigation gmapping_demo.launch
```

```
roslaunch jackal_viz view_robot.launch config:=gmapping
```

To save the map :
```
rosrun map_server map_saver -f mymap
```

To localize with AMCL :
```
roslaunch jackal_navigation amcl_demo.launch map_file:=/path/to/my/map.yaml
```

To start navigation :
```
roslaunch jackal_viz view_robot.launch config:=localization
```

### For navigation with our environment (cpr_gazebo) :

Launching gazebo :
```
roslaunch cpr_inspection_gazebo inspection_world.launch platform:=jackal
```
Launching gmapping with custom scan topic:
```
roslaunch jackal_navigation gmapping_demo.launch scan_topic:=lidar/scan
```

```
roslaunch jackal_viz view_robot.launch config:=gmapping
```
To save the map :
```
rosrun map_server map_saver -f mymap
```

To localize with AMCL :
```
roslaunch jackal_navigation amcl_demo.launch map_file:=/path/to/my/map.yaml
```

To start navigation :
```
roslaunch jackal_viz view_robot.launch config:=localization
```