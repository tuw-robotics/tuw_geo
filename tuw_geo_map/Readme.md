
# tuw_geo_map

## geo_map_node
The node is a lifecycle node. Therefore you have to configure and activete the node after starting.
```
ros2 lifecycle set /geo_map configure; 
ros2 lifecycle set /geo_map activate
```

## run
```
ros2 run tuw_geo geo_map_node --ros-args -p mapimage_folder:=${workspaceFolder}/ws02/src/tuw_geo_map/config/tuw_geo_map/straden/ -p publish_tf:=true
# ros2 lifecycle set /geo_map configure; sleep 1; ros2 lifecycle set /geo_map activate
```
