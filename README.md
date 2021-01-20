# ros2_topological_nav

## Launch
```shell
ros2 launch ros2_topological_nav ros2_topo_nav_launch.py
```

## Shell Examples
```shell
ros2 service call /topo_nav/add_point ros2_topological_nav_interfaces/srv/AddPoint "{'point':{'id':'kitchen', 'pose':{'position':{'x':3.79, 'y':6.77, 'z':0.0}, 'orientation':{'x':0.0, 'y':0.0, 'z':0.99, 'w':0.12}}}}"
ros2 service call /topo_nav/get_point ros2_topological_nav_interfaces/srv/GetPoint "{'point':'kitchen'}"
ros2 service call /topo_nav/get_points ros2_topological_nav_interfaces/srv/GetPoints {}
ros2 action send_goal /topo_nav/navigation ros2_topological_nav_interfaces/action/TopoNav "{'point':'kitchen'}"
```
