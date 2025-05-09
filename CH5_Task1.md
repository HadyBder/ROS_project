The br2_vff_avoidance node implements a Virtual Force Field (VFF)–based reactive obstacle avoidance behavior in ROS 2. 
It continuously processes laser scans tocompute attractive (forward) and repulsive (away from obstacles) force vectors, 
sums them into a resultant velocity command, and publishes that as a geometry_msgs/Twist on /output_vel. 
Key enhancements include three runtime-tunable parameters 
(max_linear_speed, repulsive_gain, and safety_distance), an emergency-stop that zeros velocities if any obstacle is within
the safety distance, and an expanded repulsion loop that aggregates contributions from all scan beams inside a configurableradius.
We also added clean initialization of the result vector each cycle and conditional publishing of debug MarkerArray arrows only when RViz 
(or another subscriber) is listening, improving flexibility, safety, and performance without altering the core VFF algorithm.
