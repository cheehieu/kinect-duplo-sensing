---
layout: project
title: kinect-duplo-sensing
subtitle: A ROS package for identifying Duplo blocks with the Microsoft Kinect sensor.
---

<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/many_duplos_result.png">

## Overview
This sensing and perception project involved using point cloud information to estimate and classify the sizes of Duplo bricks, which are construction toys (essentially big Legos) designed for young children. The specific task prompt was to "identify Duplo bricks of the same size" in 3-dimensional point cloud structures obtained from a robot-head-mounted Microsoft Kinect sensor.

A "point cloud" contains a collection of data points that represent the sensed 3-dimensional environment. Each data point contains a set of vertices in the 3D space (represented with XYZ Cartesian coordinates) and a color value (represented in RGB). Using the Robot Operating System (ROS) and the Point Cloud Library (PCL), Duplo bricks of the same size can be identified by first segmenting the bricks from the point cloud, then estimating the size of each Duplo brick, and finally searching the group for bricks of similar sizes.

This project was developed in the fall of 2011 for the "Sensing & Planning in Robotics" (CSCI-547) course at USC, taught by professor Gaurav Sukhatme.

## Hardware
An assortment of Duplo bricks are used to challenge the robustness of the sensing and perception algorithm. These include three different sizes (1x1, 1x2, and 1x4) and four different colors (red, green, blue, and yellow). The raw point cloud data is generated from the RGB camera and depth sensor inside a Microsoft Kinect.

* [Duplo](http://en.wikipedia.org/wiki/Lego_Duplo) bricks
* [Microsoft Kinect](http://en.wikipedia.org/wiki/Kinect) sensor 


## Software
The ROS framework and PCL APIs are heavily utilized to capture and manipulate point clouds generated from a Kinect sensor. First, a series of filtering, segmentation, and clustering techniques are applied to extract individual Duplo bricks from an input point cloud. The size of each Duplo cluster is estimated by calculating the height and width of its convex hull reconstruction. Finally, the estimated size is used to classify the cluster into a Duplo brick type, which can then be used in size comparison with other clusters.

* [Robot Operating System (ROS)](http://www.ros.org/)
* [Point Cloud Library (PCL)](http://pointclouds.org/)

### Segmentation
<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/segmentation_algorithm.png" width="100%">

A raw point cloud from the Kinect sensor contains over 300,000 data points. Thus, to improve performance, a pcl::VoxelGrid filter is used to downsample the input cloud. The voxel grid filter assembles a local 3D voxel grid over the input point cloud data. All data points within each voxel are then approximated with their centroid to represent the underlying surface, effectively compressing the point cloud.

The downsampled cloud then undergoes a planar segmentation to identify the tabletop surface underneath the Duplo bricks. The pcl::SACSegmentation algorithm uses the RANSAC method to find all points within a point cloud that support a plane model. This returns the four planar coefficients and all of the inlying points on the largest planar component in the cloud. Using these inlier point indices, pcl::ExtractIndices can project the inliers onto the plane model and create a new point cloud, which represents the objects on the table.

To remove unwanted points that are located in the plane, but off the table, a pcl::PassThrough filter is used. This filter allows data points to pass through to the output cloud, given the points satisfy a specified range along a particular dimension.

<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/segmentation.gif" width="100%">

Next, the filtered point cloud undergoes a color segmentation to separate the red, green, blue, and yellow Duplo bricks from each other. This is done by iterating over all points and pushing those points with specific color characteristics into four new point clouds. Red bricks are characterized by having more R-value than B-value, and twice as much R-value than G-value.

Each color-segmented point cloud undergoes a clustering algorithm to determine points that belong to the same Duplo brick. The pcl::EuclidianClusterExtraction uses a 3D grid subdivision (octree data structure) for spatial representation, and performs a clustering technique using a nearest neighbors search. A KdTree object is created for the search method of the extraction algorithm to find correspondences between groups of points. Each individual cluster is extracted to its own point cloud and concatenated into a vector of point cloud pointers for further processing.

<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/red.png" width="49%">
<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/green.png" width="49%">
<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/blue.png" width="49%">
<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/yellow.png" width="49%">

### Size Estimation
As each cluster is extracted from its color-segmented point cloud, a simple 2D convex hull polygon for the set of points is calculated using a pcl::ConvexHull reconstruction. This convex hull provides an envelope of points representing the minimal convex set. Using these convex hull points, we can estimate the height and width of each cluster. Height is calculated by finding the difference between the maximum and minimum distances from a point in the convex hull reconstruction to the table plane using the planar coefficients obtained in the pcl::SACSegmentation. The width of each cluster is calculated by finding the maximum distance between two points in the 2D convex hull polygon.

It is possible to get a more accurate size estimation by utilizing pcl::ConvexHull's getTotalArea() function on the convex polygon, which uses the libqhull library to return an area using more complex calculations. Also, running getTotalVolume() on a convex polyhedron reconstruction of the clusters would return volume information. However, this approach would not accurately estimate size because all points of a Duplo brick are not present in the point cloud, causing the 3D reconstruction to be incomplete.

### Size Classification
<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/size_classification_algorithm.png">

Using the height and width calculations, the Duplo bricks are classified by height orientation and brick type. The height classification speculates how the brick is situated on the table. It can determine if the brick is standing flat, standing on its side, or standing long ways (for larger bricks) by checking if the calculated height falls within the actual measured height, ±5mm for error. Similarly, the width classification speculates the brick type. In this project, only three different size Duplo bricks are used: 1x1, 1x2, and 1x4. The width classification thresholds were determined by measuring the minimum 2D brick length and maximum brick length along the 3D diagonal, ±5mm for error. Bricks with calculated heights and widths outside the classification thresholds are ID'ed as "unclassified."

### Perception
After Duplo brick segmentation, size estimation, and size classification, the ROS node will output information to the terminal window and publish its results to a pcl::PCLVisualizer viewer. The terminal output contains quantitative information on the number of color clusters, the total number of clusters, and the number of bricks classified for each brick type. It also lists the cluster indices for each brick type.

<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/term_1.png">

The clusters are labeled with their cluster indices, and are color-coded based on their brick type; the input point cloud retains its original RGB color values. Brick type 1x1 is colored RED, type 1x2 is colored YELLOW, type 1x4 is colored BLUE, and type “unclassified” is colored WHITE.
<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/group2_1.jpg" width="100%">
<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/results_size.png" width="100%">

## Results
Here are some additional static Duplo configurations that were used to test the algorithm: 

<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/group2_2.jpg" width="100%">
<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/viewer2.png" width="100%">
<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/term_2.png">
- - -
- - -
<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/group2_3.jpg" width="100%">
<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/viewer3.png" width="100%">
<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/term_3.png">
- - -
- - -
<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/group8_new1.jpg">
<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/many_duplos.png" width="100%">


## Demo
The algorithm was further tested with a Kinect sensor to obtain point cloud data in real-time. A set of Duplo bricks were placed on a white tabletop surface, with the Kinect viewing the scene from a high angle (to mimic the head-mounted Kinect on the PR2). The ROS node now subscribes to the “/camera/rgb/points” topic that is published by the openni_node. As the ROS node receives PointCloud2 messages from the openni_node, the callback function performs the cloud processing to identify Duplo bricks of the same size.

<img src="http://niftyhedgehog.com/kinect-duplo-sensing/images/demo_setup.jpg" width="100%">

The algorithm correctly segmented and classified all Duplo bricks in the scene. Because the Kinect sensor publishes a new PointCloud2 message every two seconds, the algorithm is running in near-real-time. As the point cloud data changes (when Duplo bricks are added/removed or reoriented), the algorithm reacts accordingly and outputs the correct results.

<iframe width="100%" height="400" src="https://www.youtube.com/embed/S1WBPGSog2c" frameborder="0" allowfullscreen></iframe>
<!--video src="http://niftyhedgehog.com/kinect-duplo-sensing/images/duplo_sensing_demo.mp4" width="100%" height="400px" controls="controls"></video-->


## Limitations
This algorithm works well for point clouds that have its Duplo bricks sufficiently spread out across a planar surface. However, it's performance is limited when these assumptions are not met. To resolve these issues, improvements must be made to increase the resolution of the segmentation algorithm to distinguish between same-color bricks that are spaced closely together. The size estimation algorithm can be improved by utilizing more advanced PCL functions dealing with surface reconstruction, model fitting, and volume estimation. This will become easier as the integration of ROS and PCL develops better, more up-to-date support.

This sensing task of identifying the size of Duplo bricks can be extended to manipulation tasks in robots and robotic simulators. Once the sizes of objects are classified, a robot can perform more intelligent tasks when it understands the physical properties associated with those objects. For example, given an assortment of brick sizes, a robot can construct a pyramid structure using larger bricks at the base and smaller bricks at the top.
