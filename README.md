# topological_nav

## Description

- ROS2 Foxy Fitzroy
- Topological navigation for ROS2


## Launch
```shell
ros2 launch topological_nav topological_nav_launch.py
```

## Shell Examples
```shell
ros2 service call /topological_nav/add_point topological_nav_interfaces/srv/AddPoint "{'point':{'id':'kitchen', 'pose':{'position':{'x':3.79, 'y':6.77, 'z':0.0}, 'orientation':{'x':0.0, 'y':0.0, 'z':0.99, 'w':0.12}}}}"
ros2 service call /topological_nav/get_point topological_nav_interfaces/srv/GetPoint "{'point':'kitchen'}"
ros2 service call /topological_nav/get_points topological_nav_interfaces/srv/GetPoints {}
ros2 action send_goal /topological_nav/navigation topological_nav_interfaces/action/TopoNav "{'point':'kitchen'}"
```
