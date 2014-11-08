#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <iostream>
#include <math.h>

#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
#include "pcl/features/normal_3d.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/console/parse.h>
//Duplo Sizes: 33x33x25 (1x1), 33x65x25 (1x2), 33x130x25 (1x4) 

ros::Publisher pub_input;
ros::Publisher pub_downsampled;
ros::Publisher pub_planar;
ros::Publisher pub_objects;
ros::Publisher pub_filtered;
ros::Publisher pub_red;
ros::Publisher pub_green;
ros::Publisher pub_blue;
ros::Publisher pub_yellow;
ros::Publisher pub_concat_clusters;
ros::Publisher pub_concat_hulls;
pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients ());
std::vector<pcl::PointXYZRGB> labels;
std::vector<int> block_ids;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
resultsVis (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds)
{
	// Visualize input and resulting cloud
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
	viewer->initCameraParameters ();
	
	int v1 (0);	//input cloud
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor (0, 0, 0, v1);
	viewer->addText("ViewPort 1: Input PointCloud", 10, 10, "v1_text", v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);
	
	int v2(0);	//size-classified cloud
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer->setBackgroundColor (0, 0, 0, v2);
        viewer->addText("ViewPort 2: Size-classified PointCloud", 10, 10, "v2_text", v2);
	for (size_t i = 0 ; i < clouds.size() ; i++)
	{
		char buffer[10];
		sprintf(buffer, "%d", i);
		if (block_ids[i] == 1)		//block 1x1 is colored red (255,0,0)
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(clouds[i], 255, 0, 0);
			viewer->addPointCloud<pcl::PointXYZRGB> (clouds[i], single_color, buffer, v2);
		}	
		else if(block_ids[i] == 2)	//block 1x2 is colored yellow (255,255,0)
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(clouds[i], 255, 255, 0);
			viewer->addPointCloud<pcl::PointXYZRGB> (clouds[i], single_color, buffer, v2);
		}	
		else if (block_ids[i] == 4)	//block 1x4 is colored blue (0,0,255)
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(clouds[i], 0, 0, 255);
			viewer->addPointCloud<pcl::PointXYZRGB> (clouds[i], single_color, buffer, v2);
		}	
		else				//unclassified block is colored white (255,255,255)
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(clouds[i], 255, 255, 255);
			viewer->addPointCloud<pcl::PointXYZRGB> (clouds[i], single_color, buffer, v2);
		}	
		viewer->addText3D(buffer, labels[i], 0.01, 1, 1, 1);
		//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, buffer);
	}
	//viewer->addCoordinateSystem (0.25);
	//updatePointCloud(cloud, "id")
	return (viewer);
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	std::cout << "\n\n\n----------------Received point cloud!-----------------\n";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_planar (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_objects (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_red (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_green (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blue (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_yellow (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_concat_clusters (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_concat_hulls (new pcl::PointCloud<pcl::PointXYZRGB>);
	sensor_msgs::PointCloud2 downsampled2, planar2, objects2, filtered2, red2, green2, blue2, yellow2, concat_clusters2, concat_hulls2;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> color_clouds, cluster_clouds, hull_clouds;
	
//	fromROSMsg(*input, *cloud);
//	pub_input.publish(*input);

	// Downsample the input PointCloud
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	sor.setInputCloud (input);
	sor.setLeafSize (0.001, 0.001, 0.001);
	sor.filter (downsampled2);
	fromROSMsg(downsampled2,*downsampled);
//	pub_downsampled.publish (downsampled2);	
	
	// Segment the largest planar component from the downsampled cloud
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
//	pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.0085);
	seg.setInputCloud (downsampled);
	seg.segment (*inliers, *coeffs);
	
	// Extract the planar inliers from the input cloud
	pcl::ExtractIndices<pcl::PointXYZRGB> extract; 
	extract.setInputCloud (downsampled);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (*cloud_planar);
//	toROSMsg(*cloud_planar,planar2);
//	pub_planar.publish (planar2);

	// Remove the planar inliers, extract the rest
	extract.setNegative (true);
	extract.filter (*cloud_objects);
//	toROSMsg(*cloud_objects,objects2);
//	pub_objects.publish (objects2);

	// PassThrough Filter
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (cloud_objects);
	pass.setFilterFieldName ("z");	//all duplos in pcd1
	pass.setFilterLimits (0.8, 1.0);
	pass.filter (*cloud_filtered);
	toROSMsg(*cloud_filtered,filtered2);
	pub_filtered.publish (filtered2);

	// Segment filtered PointCloud by color (red, green, blue, yellow)
	for (size_t i = 0 ; i < cloud_filtered->points.size () ; i++)
	{
		if ( (int(cloud_filtered->points[i].r) > 2*int(cloud_filtered->points[i].g)) && (cloud_filtered->points[i].r > cloud_filtered->points[i].b) )
			cloud_red->points.push_back(cloud_filtered->points[i]);
		if ( (cloud_filtered->points[i].g > cloud_filtered->points[i].r) && (cloud_filtered->points[i].g > cloud_filtered->points[i].b) )
			cloud_green->points.push_back(cloud_filtered->points[i]);
		if ( (cloud_filtered->points[i].b > cloud_filtered->points[i].r) && (cloud_filtered->points[i].b > cloud_filtered->points[i].g) )
			cloud_blue->points.push_back(cloud_filtered->points[i]);
		if ( (cloud_filtered->points[i].r > cloud_filtered->points[i].g) && (int(cloud_filtered->points[i].g) - int(cloud_filtered->points[i].b) > 30) )
			cloud_yellow->points.push_back(cloud_filtered->points[i]);
	}
	cloud_red->header.frame_id = "base_link";
	cloud_red->width = cloud_red->points.size ();
	cloud_red->height = 1;
	color_clouds.push_back(cloud_red);
	toROSMsg(*cloud_red,red2);
	pub_red.publish (red2);
	cloud_green->header.frame_id = "base_link";
	cloud_green->width = cloud_green->points.size ();
	cloud_green->height = 1;
	color_clouds.push_back(cloud_green);
	toROSMsg(*cloud_green,green2);
	pub_green.publish (green2);
	cloud_blue->header.frame_id = "base_link";
	cloud_blue->width = cloud_blue->points.size ();
	cloud_blue->height = 1;
	color_clouds.push_back(cloud_blue);
	toROSMsg(*cloud_blue,blue2);
	pub_blue.publish (blue2);
	cloud_yellow->header.frame_id = "base_link";
	cloud_yellow->width = cloud_yellow->points.size ();
	cloud_yellow->height = 1;
	color_clouds.push_back(cloud_yellow);
	toROSMsg(*cloud_yellow,yellow2);
	pub_yellow.publish (yellow2);
	

	// Extract Euclidian clusters from color-segmented PointClouds
	int j(0), num_red (0), num_green(0), num_blue(0), num_yellow(0);
	for (size_t cit = 0 ; cit < color_clouds.size() ; cit++)
	{
		pcl::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
		tree->setInputCloud (color_clouds[cit]);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
		ec.setClusterTolerance (0.0075);
		ec.setMinClusterSize (100);
		ec.setMaxClusterSize (4000);
		ec.setSearchMethod (tree);
		ec.setInputCloud (color_clouds[cit]);
		ec.extract (cluster_indices);

		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{	
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
			for (std::vector<int>::const_iterator pit = it->indices.begin () ; pit != it->indices.end () ; pit++)
				cloud_cluster->points.push_back (color_clouds[cit]->points[*pit]);
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			cloud_cluster->header.frame_id = "base_link";
			cluster_clouds.push_back(cloud_cluster);
			labels.push_back(cloud_cluster->points[0]);
			if (cit == 0)	num_red++;
			if (cit == 1)	num_green++;
			if (cit == 2)	num_blue++;
			if (cit == 3)	num_yellow++;
			
			// Create ConvexHull for cluster (keep points on perimeter of cluster)
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
  			pcl::ConvexHull<pcl::PointXYZRGB> chull;
  			chull.setInputCloud (cloud_cluster);
  			chull.reconstruct (*cloud_hull);
			cloud_hull->width = cloud_hull->points.size ();
			cloud_hull->height = 1;
			cloud_hull->header.frame_id = "base_link";
			hull_clouds.push_back(cloud_hull);

			j++;
		}
	}
	std::cout << "Number of RED clusters: " << num_red << std::endl;
	std::cout << "Number of GREEN clusters: " << num_green << std::endl;
	std::cout << "Number of BLUE clusters: " << num_blue << std::endl;
	std::cout << "Number of YELLOW clusters: " << num_yellow << std::endl;
	std::cout << "TOTAL number of clusters: " << j << std::endl;

	// Concatenate PointCloud clusters and convex hulls
	for (size_t k = 0 ; k < cluster_clouds.size() ; k++)
	{
		for (size_t l = 0 ; l < cluster_clouds[k]->size() ; l++)
		{
			cloud_concat_clusters->points.push_back(cluster_clouds[k]->points[l]);
			cloud_concat_hulls->points.push_back(hull_clouds[k]->points[l]);
		}
	}
	cloud_concat_clusters->header.frame_id = "base_link";
	cloud_concat_clusters->width = cloud_concat_clusters->points.size ();
	cloud_concat_clusters->height = 1;
	toROSMsg(*cloud_concat_clusters,concat_clusters2);
	pub_concat_clusters.publish (concat_clusters2);
	cloud_concat_hulls->header.frame_id = "base_link";
	cloud_concat_hulls->width = cloud_concat_hulls->points.size ();
	cloud_concat_hulls->height = 1;
	toROSMsg(*cloud_concat_hulls,concat_hulls2);
	pub_concat_hulls.publish (concat_hulls2);

	// Estimate the volume of each cluster
	double height, width;
	std::vector <double> heights, widths;
	std::vector <int>  height_ids, width_ids;
	for (size_t k = 0 ; k < cluster_clouds.size() ; k++)
	{
		// Calculate cluster height
		double tallest(0), shortest(1000), widest(0) ;
		for (size_t l = 0 ; l < cluster_clouds[k]->size() ; l++)
		{
			double point_to_plane_dist;
			point_to_plane_dist = 	   coeffs->values[0] * cluster_clouds[k]->points[l].x + 
						   coeffs->values[1] * cluster_clouds[k]->points[l].y +
						   coeffs->values[2] * cluster_clouds[k]->points[l].z + coeffs->values[3] / 
						   sqrt( pow(coeffs->values[0], 2) + pow(coeffs->values[1], 2)+ pow(coeffs->values[2], 2) );
			if (point_to_plane_dist < 0)		point_to_plane_dist = 0;
			if (point_to_plane_dist > tallest)	tallest = point_to_plane_dist;
			if (point_to_plane_dist < shortest)	shortest = point_to_plane_dist;
		}
		// Calculate cluster width
		for (size_t m = 0 ; m < hull_clouds[k]->size() ; m++)
		{	
			double parallel_vector_dist;
			for (size_t n = m ; n < hull_clouds[k]->size()-1 ; n++)
			{
				parallel_vector_dist = sqrt( pow(hull_clouds[k]->points[m].x - hull_clouds[k]->points[n+1].x,2) + 
  							     pow(hull_clouds[k]->points[m].y - hull_clouds[k]->points[n+1].y,2) +
							     pow(hull_clouds[k]->points[m].z - hull_clouds[k]->points[n+1].z,2) );
				if (parallel_vector_dist > widest)	widest = parallel_vector_dist;	
			}
		}
		// Classify block heights (error +/- .005m)
		height = (shortest < .01) ? tallest : tallest - shortest;	//check for stacked blocks	
		heights.push_back(height);
		if (height > .020 && height < .032)		height_ids.push_back(0);	//0: standing flat
		else if (height > .036 && height < .043)	height_ids.push_back(1);	//1: standing side
		else if (height > .064)				height_ids.push_back(2);	//2: standing long
		else						height_ids.push_back(-1);	//height not classified
		// Classify block widths (error +/- .005m)
		width = widest;
		widths.push_back(widest);
		if (width > .032-.005 && width < .0515+.005)		width_ids.push_back(1);	//1: short
		else if (width > .065-.005 && width < .0763+.005)	width_ids.push_back(2);	//2: medium
		else if (width > .1275-.005 && width < .1554+.005)	width_ids.push_back(4);	//4: long
		else						width_ids.push_back(-1);	//width not classified
	}

	// Classify block size using width information
	std::vector<int> idx_1x1, idx_1x2, idx_1x4, idx_unclassified;
	int num_1x1(0), num_1x2(0), num_1x4(0), num_unclassified(0);
	for (size_t p = 0 ; p < width_ids.size() ; p++)	
	{
		if (width_ids[p] == 1)		
		{
			block_ids.push_back(1);		//block is 1x1
			idx_1x1.push_back(p);
			num_1x1++;
		}
		else if (width_ids[p] == 2)	
		{
			block_ids.push_back(2);         //block is 1x2
			idx_1x2.push_back(p);
			num_1x2++;
		}
		else if (width_ids[p] == 4)     
		{
			block_ids.push_back(4);         //block is 1x4
			idx_1x4.push_back(p);
			num_1x4++;
		}
		else	
		{
			block_ids.push_back(-1);	//block not classified
			idx_unclassified.push_back(p);
			num_unclassified++;
		}
	}

	// Determine Duplos of the same size
	std::cout << "\nThere are " << num_1x1 << " blocks of size 1x1 ";
	if (num_1x1>0)	std::cout << "(cluster index: ";
	for (size_t q = 0 ; q < idx_1x1.size() ; q++)
		std::cout << idx_1x1[q] << ", ";
	if (num_1x1>0)  std::cout << ")";
	std::cout << "\nThere are " << num_1x2 << " blocks of size 1x2 ";
        if (num_1x2>0)  std::cout << "(cluster index: ";
	for (size_t q = 0 ; q < idx_1x2.size() ; q++)
		std::cout << idx_1x2[q] << ", ";
	if (num_1x2>0)  std::cout << ")";
	std::cout << "\nThere are " << num_1x4 << " blocks of size 1x4 ";
        if (num_1x4>0)  std::cout << "(cluster index: ";
	for (size_t q = 0 ; q < idx_1x4.size() ; q++)
		std::cout << idx_1x4[q] << ", ";
	if (num_1x4>0)  std::cout << ")";
	std::cout << "\nThere are " << num_unclassified << " unclassified blocks ";
        if (num_unclassified>0)  std::cout << "(cluster index: ";
	for (size_t q = 0 ; q < idx_unclassified.size() ; q++)
		std::cout << idx_unclassified[q] << ", ";
	if (num_unclassified>0)  std::cout << ")";
	std::cout << std::endl;


//	viewer = resultsVis(cloud_concat_clusters, cluster_clouds);
	viewer = resultsVis(downsampled, cluster_clouds);
	while (!viewer->wasStopped ())
  	{
    		viewer->spinOnce (100);
    		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  	}

	return;
}


int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/camera/rgb/points", 1, cloud_cb); //Kinect data
	//ros::Subscriber sub = nh.subscribe ("cloud_pcd", 1, cloud_cb);	//broadcasted group2_1.pcd

	// Create a ROS publisher for the output point cloud
	pub_input = nh.advertise<sensor_msgs::PointCloud2> ("raw_cloud", 1);
	pub_downsampled = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_cloud", 1);
	pub_planar = nh.advertise<sensor_msgs::PointCloud2> ("planar_cloud", 1);
	pub_objects = nh.advertise<sensor_msgs::PointCloud2> ("objects_cloud", 1);
	pub_filtered = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1);
	pub_red = nh.advertise<sensor_msgs::PointCloud2> ("red_cloud", 1);
	pub_green = nh.advertise<sensor_msgs::PointCloud2> ("green_cloud", 1);
	pub_blue = nh.advertise<sensor_msgs::PointCloud2> ("blue_cloud", 1);
	pub_yellow = nh.advertise<sensor_msgs::PointCloud2> ("yellow_cloud", 1);
	pub_concat_clusters = nh.advertise<sensor_msgs::PointCloud2> ("concat_clusters_cloud", 1);
	pub_concat_hulls = nh.advertise<sensor_msgs::PointCloud2> ("concat_hulls_cloud", 1);

	ros::spin ();
}
