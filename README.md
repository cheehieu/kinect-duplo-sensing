kinect-duplo-sensing
====================

ROS package for identifying Duplo blocks with the Microsoft Kinect sensor

Developed in Fall 2011 for the Sensing & Planning in Robotics (CSCI 545) course at USC.

Used ROS and the PR2-simulator to obtain point cloud data from a robot-head-mounted Microsoft Kinect sensor. Segmented Duplo blocks by color and size to identify block orientation and perform robotic manipulation tasks. 

The segmentation algorithm involved extracting the planar inliers from a VoxelGrid downsampled and planar SACSegmentated input PointCloud. A PassThrough filter and RGBY color segmentation was then applied to make way for the Euclidian cluster extraction. 

The size classification algorithm invloved performing a ConvexHull reconstruction on the PointCloud clusters. The height and width of each cluster were estimated, which could then satisfy a brick classification tag. 

# Demo
Real-time Duplo recognition using a Kinect sensor and the PointCloud Library.

## Setup
<img src="images/demo_setup.jpg" width="800">

## Duplo Test Arrangement
<img src="images/demo_duplos.jpg" width="800">

## Video
<video width="320" height="240" controls>
	<source src="images/duplo_kinect_demo.mpeg" type="video/mpeg">
</video>

