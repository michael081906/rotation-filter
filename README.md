# rotation_filter
This package is for point cloud filtering

TODO:  
1. roslaunch and remap topics
2. continuous integration travis and coveralls

## Useful steps  
Launch the ros master
```
roscore
```
Launch the tf to publish frame between two frames. 
```
roslaunch rotation_filter publish_frame.launch
```
Launch point cloud publisher to provide point cloud data. You can use external camera to do so as well.
```
roslaunch point_cloud_publisher point_cloud_publisher_single.launch
```
Launch the main node
```
rosrun rotation_filter rotation_filter_node
```
The node provide rqt_reconfigure to allow user adjusting rotations of the filter. 
```
rosrun rqt_reconfigure rqt_reconfigure
```
In order to set a new frame 
```
rosservice call /rtf_client "{task_id: 3, tf_ref: 'robot_base_frame', tf_target: 'camera_frame', tf_new: 'user_frame', offset_x: 0.4, offset_y: 0.4,
  offset_z: 0.0, filter: false}" 
```
Then allow the node to filter
```
rosservice call /rtf_client "{task_id: 4, tf_ref: 'robot_base_frame', tf_target: 'camera_frame', tf_new: 'user_frame', offset_x: 0.4, offset_y: 0.4,
  offset_z: 0.0, filter: true}"
```
