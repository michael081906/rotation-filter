#!/bin/bash
rosservice call /rtf_client "{task_id: 3, tf_ref: 'robot_base_frame', tf_target: 'camera_frame', tf_new: 'user_frame', offset_x: 0.4, offset_y: 0.4,
  offset_z: 0.0, filter: false}"
rosservice call /rtf_client "{task_id: 4, tf_ref: 'robot_base_frame', tf_target: 'camera_frame', tf_new: 'user_frame', offset_x: 0.4, offset_y: 0.4,
  offset_z: 0.0, filter: true}"
