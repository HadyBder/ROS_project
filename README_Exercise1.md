# Exercise 1 – Publish Obstacle in `/odom` Frame

## 🧠 Objective
Update the original Monitor node (Chapter 4) so that the closest detected obstacle's position is **published in the `/odom` frame** instead of the laser frame (`/base_laser_link`).

## ✨ What We Changed

### 🔁 Transformation:
- Used `tf2_ros::Buffer` and `TransformListener` to **lookup the transform** between the laser scanner frame and `/odom`
- **Transformed the obstacle's 2D position** (calculated from range and angle) using:
  ```cpp
  tf_buffer_.transform(obstacle_in_laser, obstacle_in_odom, "odom");
  ```

### 🟢 Marker Update:
- The published marker's `header.frame_id` is now `odom`
- Position is set based on the **transformed coordinates**

## 🛠️ Why This Matters

Publishing in the `/odom` frame:
- Aligns the marker's position with **global robot localization**
- Makes it easier for other nodes (e.g. planners, navigators) to use obstacle info
- Demonstrates proper use of **TF2 frame transformation**, a core ROS2 concept

## 🚀 How It Helps

This update improves **modularity and interoperability**:  
Instead of being stuck in a sensor frame, obstacle data is now usable across the system (e.g., RViz view, mapping, planning layers).

