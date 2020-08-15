# Rotation Filter
[![Build Status](https://travis-ci.org/michael081906/rotation_filter.svg?branch=master)](https://travis-ci.org/michael081906/rotation_filter)
[![Coverage Status](https://coveralls.io/repos/github/michael081906/rotation_filter/badge.svg?branch=master)](https://coveralls.io/github/michael081906/rotation_filter?branch=master)
[![License: AGPL v3](https://img.shields.io/badge/License-AGPL%20v3-blue.svg)](https://www.gnu.org/licenses/agpl-3.0)  
ROS Kinetic package for point cloud filtering

## Description
<img src="https://github.com/michael081906/rotation-filter/blob/michael081906-patch-readme/docs/demo_origin.png" width="400" >  
<img src="https://github.com/michael081906/rotation-filter/blob/michael081906-patch-readme/docs/demo.gif" width="400" > 


## Installation  
```
cd ~/catkin_ws/src
git clone https://github.com/michael081906/rotation-filter.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```
## Demo  
Open a terminal and launch the ros master
```
roscore
```
Open second terminal 
```
roslaunch rotation_filter demo.launch
```
Open third terminal
```
cd ~/catkin_ws/src/rotation-filter/bash/
bash srv.bash
```
