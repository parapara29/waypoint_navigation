# ros2_topological_nav

## Launch
```shell
ros2 launch ros2_topological_nav ros2_topo_nav_launch.py
```

## Shell Examples
```shell
ros2 service call /add_point ros2_topological_nav_interfaces/srv/AddPoint "{'id':'nuevo', 'x':0.9, 'y':0.0, 'z':0.5, 'w':0.3}"
ros2 action send_goal /topo_nav ros2_topological_nav_interfaces/action/TopoNav "{'point':'kitchen'}"
```
