# ROS2 Monitor Node - Exercises 1 & 2

This node subscribes to `/scan`, finds the closest obstacle, transforms its position from `/base_laser_link` to `/odom`, and publishes a colored marker in RViz.

## Features
- Identifies the nearest valid laser scan point
- Transforms coordinates with TF2
- Publishes a colored marker (green if >1.0m, red otherwise)

## Topics
- Subscribed: `/scan`
- Published: `/visualization_marker`

## Frame Reference
- Source: `base_laser_link`
- Target: `odom`
