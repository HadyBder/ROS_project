# ROS2 Monitor Node - Exercise 3
publishes a MarkerArray to RViz with each obstacle point visualized individually.

## Features
- Loops through all finite LaserScan points
- Transforms each to `/odom`
- Publishes red/green sphere markers based on distance

## Topics
- Subscribed: `/scan`
- Published: `/visualization_marker_array`

## Frame Reference
- Source: `base_laser_link`
- Target: `odom`
