MoveToFreeSpace.cpp file implements a custom synchronous Behavior Tree action node for ROS 2. Its role is to check for nearby obstacles using LIDAR data and command the robot to move forward only when space is clear. If an obstacle is detected, the action fails and allows fallback behavior in the tree (e.g., normal waypoint navigation).
How It Works
    Subscriptions:
        /scan_raw → Uses sensor_msgs::msg::LaserScan to detect obstacles.
    Publications:
        /nav_vel → Sends velocity commands using geometry_msgs::msg::Twist.
    Behavior:
        Moves forward at 0.2 m/s if no obstacle is within 0.6 meters.
        If an obstacle is detected, it stops and returns FAILURE.
