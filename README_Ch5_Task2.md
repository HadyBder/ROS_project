Implemented a reactive object detection node in ROS 2 that processes camera images using HSV filtering,
extracts contours for each detected object, and publishes the bounding box of the most recently detected 
object as a vision_msgs/msg/Detection2D message, with optional debug visualization.

What We Changed:

Contour Extraction: 
Replaced the global mask bounding box with cv::findContours() to identify each independent 
object blob in the thresholded image.

Multi-Object Bounding: 
Computed a cv::Rect bounding box for each contour, enabling per-objectlocalization instead
of a single aggregate region.

Recent Object Selection: 
cv::Rect bbx = boxes.back();
Selected and published only the bounding box corresponding to the most recently detected contour
(last in the list), focusing downstream processing on the newest detection.

Debug Overlay: 
Updated debug mode to draw and display only the selected bounding box and centroid on the image,
improving clarity when visualizing detections.
