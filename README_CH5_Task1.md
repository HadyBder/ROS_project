Objective:
Implement a Virtual Force Field (VFF)–based reactive obstacle avoidance node in ROS 2 that continuously processes laser scans and outputs velocity commands and optional debug markers for visualization.

✨ What We Changed

Parameterized Control: 
Exposed three new ROS parameters—max_linear_speed, repulsive_gain, and safety_distance—to allow runtime tuning of forward speed limits, avoidance strength, and emergency-stop thresholds.

Emergency Stop: 
Added a safety check that zeros both linear and angular velocities when any obstacle is detected closer than safety_distance.

Full-Beam Repulsion: 
Extended get_vff() to loop over all scan beams within the obstacle radius, summing each individual repulsive vector for smoother avoidance.

Clean Initialization:
Reset the result vector to {0.0, 0.0} at each cycle to avoid leftover data from previous iterations.

Conditional Debug Publishing: 
Wrapped debug MarkerArray publication in a get_subscription_count() check, only sending markers when RViz (or other listeners) is subscribed.
