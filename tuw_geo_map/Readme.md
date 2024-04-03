
# tuw_geo_map
## geo_map_node
### Parameters
* __publish_tf__: 
  * On true a tf from frame_utm to frame_map is published
  * default: *true*
* __frame_map__: 
  * Name of the map frame, only need if *publish_tf == true*
  * default: *geo_map*
* __frame_utm__: 
  * Name of the utm frame, only need if *publish_tf == true*
  * default: *utm*
* __mapimage_folder__: 
  * folder with the world file mapimage.jpw and the image mapimage.jpg
  * default: empty
### Publisher
* __nav_msg::OccupancyGrid__
  * Costmap with Objects 
  * Topic: *geo_map*
### Lifecycle node
The node is a lifecycle node. Therefore you have to configure and activete the node after starting.
```
ros2 lifecycle set /geo_map configure; 
ros2 lifecycle set /geo_map activate
```
## Demo
RViz
```
ros2 run rviz2 rviz2 -d ./ws02/src/tuw_geo/tuw_geo_map/config/rviz/geo_map.rviz
```

geo_map 
```
ros2 run tuw_geo_map geo_map_node --ros-args  -p mapimage_folder:=./ws02/src/tuw_geo/tuw_geo_map/config/tuw_geo_map/straden/ -p publish_tf:=true
```

Lifecycle
```
ros2 lifecycle set /geo_map configure; sleep 1; ros2 lifecycle set /geo_map activate
```