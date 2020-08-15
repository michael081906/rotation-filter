# Rotation Filter
[![Build Status](https://travis-ci.org/michael081906/rotation_filter.svg?branch=master)](https://travis-ci.org/michael081906/rotation_filter)
[![Coverage Status](https://coveralls.io/repos/github/michael081906/rotation_filter/badge.svg?branch=master)](https://coveralls.io/github/michael081906/rotation_filter?branch=master)
[![License: AGPL v3](https://img.shields.io/badge/License-AGPL%20v3-blue.svg)](https://www.gnu.org/licenses/agpl-3.0)  
ROS Kinetic package for point cloud filtering

## Description
<img src="https://github.com/michael081906/rotation-filter/blob/michael081906-patch-readme/docs/demo_origin.png" width="400" >  
<img src="https://github.com/michael081906/rotation-filter/blob/michael081906-patch-readme/docs/demo.gif" width="400" > 
<div class="after image" >
## Installation
```
cd ~/catkin_ws/src
git clone https://github.com/michael081906/rotation-filter.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```
## Demo  
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

### TODO:  
1. roslaunch and remap topics

</div>
