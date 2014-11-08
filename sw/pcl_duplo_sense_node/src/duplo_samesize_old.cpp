#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <pcl/io/pcd_io.h>	//for loading PCD file

#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>//
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <math.h>//sqrt()
#include <pcl/ModelCoefficients.h>//euclid cluster extraction
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>//convex_hull
//#include <../include/my_pcl_tutorial/convex_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
//Duplo Sizes: 33x33x25 (1x1), 33x65x25 (1x2), 33x130x25 (1x4) 

ros::Publisher pub_input;
ros::Publisher pub_downsampled;
ros::Publisher pub_planar;
ros::Publisher pub_objects;
ros::Publisher pub_duplos;
ros::Publisher pub_filtered;
ros::Publisher pub_cluster;
ros::Publisher pub_projected;
ros::Publisher pub_hull;
ros::Publisher pub_cluster_planar;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	//filter, segment by color, determine Duplos, determine size, find matches
	//probability distribution of matches, sizes
	//RGB or HSV for red, yellow, green, blue
	//dimensions for 3 types of blocks
	//make less computationally intensive
	//extend to manipulation
	std::cout << "Received point cloud!\n";


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_planar (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_objects (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_duplos (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_planar (new pcl::PointCloud<pcl::PointXYZRGB>);

//	sensor_msgs::PointCloud2::Ptr cloud2 (new sensor_msgs::PointCloud2 ());
	sensor_msgs::PointCloud2 downsampled2, planar2, objects2, duplos2, filtered2, cluster2, cluster_planar2, projected2, hull2;
	
	
	fromROSMsg(*input, *cloud);
	pub_input.publish(*input);


	// Create the filtering object
	//*cloud2 = filtered2;
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	sor.setInputCloud (input);
	sor.setLeafSize (0.01, 0.01, 0.01);
	sor.filter (downsampled2);
	fromROSMsg(downsampled2,*downsampled);
	pub_downsampled.publish (downsampled2);	
//Downsampling a PointCloud using a VoxelGrid filter
//downsample the dataset with leaf size 1cm
//PLAY AROUND WITH LEAF SIZE FOR DOWNSAMPLING THROUGH VOXEL FILTER
//use the SACMODEL_PLANE to segment this PointCloud, and the method used to find this model is SAC_RANSAC
	
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.01);
	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud (downsampled);
	seg.segment (*inliers, *coefficients);
	
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGB> extract; 
	// Extract the planar inliers from the input cloud
	extract.setInputCloud (downsampled);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (*cloud_planar);
	toROSMsg(*cloud_planar,planar2);
	pub_planar.publish (planar2);

	// Remove the planar inliers, extract the rest
	extract.setNegative (true);
	extract.filter (*cloud_objects);
	toROSMsg(*cloud_objects,objects2);
	pub_objects.publish (objects2);

	// PassThrough Filter
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (cloud_objects);
	//pass.setFilterFieldName ("z");//x, rgb
	//pass.setFilterLimits (0.8, 1.0);//-0.2, -0.12
	pass.setFilterFieldName ("x");//x, rgb
	pass.setFilterLimits (-0.2, -0.12);//-0.2, -0.12
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);
	toROSMsg(*cloud_filtered,filtered2);
	pub_filtered.publish (filtered2);
/*
	// Project the model inliers 
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cloud_filtered);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected);
	toROSMsg(*cloud_projected,projected2);
	pub_objects.publish (projected2);

	// Create a Convex Hull representation of the projected inliers
	pcl::ConvexHull<pcl::PointXYZRGB> chull;
	chull.setInputCloud (cloud_projected);
	chull.reconstruct (*cloud_hull);
	toROSMsg(*cloud_hull,hull2);
	pub_objects.publish (hull2);
	std::cout << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;
*/
/*	
	// segment those points that are in the polygonal prism
	pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> ex;
	ex.setInputCloud (downsampled);//outliers
	ex.setInputPlanarHull (cloud_hull);
	pcl::PointIndices::Ptr output (new pcl::pointIndices ());
	ex.segment (*output);
*/

//Filtering a PointCloud using a PassThrough filter, 
//passthrough filters out points that have x-values outside the range (-0.4, 0.4)
//filter out RGB data?
/*I copied the color data of a pointcloud to a cv::Mat structure.
Then using HSV color filter, i extracted the indices of desired points.
Finally, using the extract indices, the points in desired color range can be determined.*/
//be able to filter out other RGB too (cylinder jar), just want the duplo blocks

/*
	//Print information
	std::cerr << "Filtered PointCloud has: " << cloud_filtered->width * cloud_filtered->height 
       		<< " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;	

	std::cerr << "CLOUD FILTERED DATA: " << std::endl;
	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		std::cerr << "    " << cloud_filtered->points[i].x << " " 
                          << cloud_filtered->points[i].y << " " 
                          << cloud_filtered->points[i].z << " " 
			  << cloud_filtered->points[i].rgb << std::endl;
*/



	pcl::PCDWriter writer;  
	// Creating the KdTree object for the search method of the extraction
	pcl::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
	tree->setInputCloud (cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance (0.01); // 2cm
	ec.setMinClusterSize (15);
	ec.setMaxClusterSize (75);
	ec.setSearchMethod (tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract (cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		toROSMsg(*cloud_cluster,cluster2);
		pub_filtered.publish (cluster2);//read cluster.pcd file and publish it to topic

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
		j++;
/*
		// Create a Convex Hull representation of the projected inliers
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::ConvexHull<pcl::PointXYZRGB> chull;
		chull.setInputCloud (cloud_cluster);
		chull.reconstruct (*cloud_hull);
	//	chull.setComputeAreaVolume(true);
	//	chull.setKeepInformation(true);
		toROSMsg(*cloud_hull,hull2);
		pub_objects.publish (hull2);
		std::cout << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;
		writer.write ("cloud_hull.pcd", *cloud_hull, false);	
	//	std::cout << "Convex hull area: " << chull.getTotalArea() << std::endl;
	//	std::cout << "Convex hull volume: " << chull.getTotalVolume() << std::endl;
*/
		
		//determine dimensions of point cloud, store in unique variable (new) find corners in 3 dimensions
		pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGB> seg2;
		// Optional
		seg2.setOptimizeCoefficients (true);
		// Mandatory
		seg2.setModelType (pcl::SACMODEL_PLANE);
		seg2.setMethodType (pcl::SAC_RANSAC);
		seg2.setDistanceThreshold (0.0025);
		seg2.setInputCloud (cloud_cluster);
		seg2.segment (*inliers2, *coefficients2);
	
		// Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZRGB> extract2; 
		// Extract the planar inliers from the input cloud
		extract2.setInputCloud (cloud_cluster);
		extract2.setIndices (inliers);
		extract2.setNegative (false);
		extract2.filter (*cloud_cluster_planar);
		toROSMsg(*cloud_cluster_planar,cluster_planar2);
		pub_planar.publish (cluster_planar2);

		float max_x=0, min_x=1000, max_y=0, min_y=1000;
		float cluster_length, cluster_width, planar_area;
		int lc(0), rc(0), bc(0), tc(0);
		for (size_t k=0 ; k<cloud_cluster->size() ; ++k)
		{
			if(cloud_cluster->points[k].x < min_x)	lc = k;
			if(cloud_cluster->points[k].x > max_x)	rc = k;
			if(cloud_cluster->points[k].y < min_y)	bc = k;
			if(cloud_cluster->points[k].y > max_y)	tc = k;
		}
//			find distances between point1 to point1, point2 to point2
//			multiply to get area
//			find height off table plane (use coefficients) computePointToPlane()
		cluster_width = sqrt( pow(cloud_cluster->points[lc].x-cloud_cluster->points[tc].x, 2) + pow(cloud_cluster->points[lc].y-cloud_cluster->points[tc].y, 2) + pow(cloud_cluster->points[lc].z-cloud_cluster->points[tc].z, 2) );
		cluster_length = sqrt( pow(cloud_cluster->points[rc].x-cloud_cluster->points[tc].x, 2) + pow(cloud_cluster->points[rc].y-cloud_cluster->points[tc].y, 2) + pow(cloud_cluster->points[rc].z-cloud_cluster->points[tc].z, 2) );
		planar_area = cluster_width * cluster_length;
		std::cout << "tc: " << cloud_cluster->points[tc] << "\t lc: " << cloud_cluster->points[lc] << std::endl;
		std::cout << "bc: " << cloud_cluster->points[bc] << "\t rc: " << cloud_cluster->points[rc] << std::endl;
		std::cout << "planar area = " << cluster_width << " * " << cluster_length << " = " << planar_area << std::endl;

		std::cout << "Cluster Size = " << cloud_cluster->points.size() << std::endl;
		for (size_t i = 0; i < cloud_cluster->points.size (); ++i)
    			std::cout << "    " << cloud_cluster->points[i].x
        	      		<< " "    << cloud_cluster->points[i].y
	              		<< " "    << cloud_cluster->points[i].z << std::endl;

	}
  	std::cout << "Number of clusters: " << j << std::endl;
//	std::cout << "Raw Height = " << cloud->width << std::endl;
//	std::cout << "Downsampled Height = " << downsampled->width << std::endl;
//	std::cout << "Planar Height = " << cloud_planar->width << std::endl;
//	std::cout << "Objects Height = " << cloud_objects->width << std::endl;
//	std::cout << "Filtered Height = " << cloud_filtered->width << std::endl;
/*	std::cout << "Planar Cluster Size = " << cloud_cluster_planar->points.size() << std::endl;

	for (size_t i = 0; i < cloud_cluster_planar->points.size (); ++i)
    		std::cout << "    " << cloud_cluster_planar->points[i].x
              		<< " "    << cloud_cluster_planar->points[i].y
           		<< " "    << cloud_cluster_planar->points[i].z << std::endl;

	//read all cloud_cluster_j.pcd's and fit planes to determine size (or calculate volume)
	pcl::PCDReader reader;
	for(int num_clusters = j ; num_clusters != 0 ; j--)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		reader.read ("cloud_cluster_j.pcd", *cloud);
		std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; 
		
	}
*/
/*
	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
	{
		
		float distance = sqrt( (x-x)^2 + (y-y)^2 +(z-z)^2 );	//calculate the Euclidean distance between two points
	}
*/
//group blocks based on color and distance of points away from each other
//within block groups, find "corners" and use xyz to calc size
//2 (3) different sizes, in different orientations
//find normal vector and calculate volume
//segment same color blocks in contact with each other
//find height of planar_cluster by getDistancesToModel() or pcl::pointToPlaneDistance() of table plane
	return;
}


int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;

/*	
	sensor_msgs::PointCloud2 cloud_blob;
	if (pcl::io::loadPCDFile ("group2_1.pcd", cloud_blob) == -1)
	{	
		std::cout << "Error loading .pcd file!\n";
		return (-1);
	}
	std::cout << "Loaded " << cloud_blob.width * cloud_blob.height
		  << " data points from PCD file.\n";
// */
	
	// Create a ROS subscriber for the input point cloud
	//ros::Subscriber sub = nh.subscribe ("/camera/rgb/points", 1, cloud_cb);	//Kinect data
	ros::Subscriber sub = nh.subscribe ("cloud_pcd", 1, cloud_cb);			//broadcasted group2_1.pcd

	// Create a ROS publisher for the output point cloud
	pub_input = nh.advertise<sensor_msgs::PointCloud2> ("raw_cloud", 1);
	pub_downsampled = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_cloud", 1);
	pub_planar = nh.advertise<sensor_msgs::PointCloud2> ("planar_cloud", 1);
	pub_objects = nh.advertise<sensor_msgs::PointCloud2> ("objects_cloud", 1);
	pub_filtered = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1);
	pub_cluster = nh.advertise<sensor_msgs::PointCloud2> ("cluster_cloud", 1);
	pub_projected = nh.advertise<sensor_msgs::PointCloud2> ("projected_cloud", 1);
	pub_hull = nh.advertise<sensor_msgs::PointCloud2> ("hull_cloud", 1);
	pub_cluster_planar = nh.advertise<sensor_msgs::PointCloud2> ("cluster_planar_cloud", 1);

//	pub_duplos = nh.advertise<isensor_msgs::PointCloud2> ("duplos_cloud", 1);
//publish bounding boxes, filter by size (colored, numbered)

	// Spin
	ros::spin ();
}
