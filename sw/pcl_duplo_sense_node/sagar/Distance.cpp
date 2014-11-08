#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <math.h>
#include <pcl/visualization/cloud_viewer.h>

pcl::PointXYZRGB o1;
pcl::PointXYZRGB o2;
std::vector<pcl::PointXYZRGB> labels;

void  viewerOneOff ( pcl::visualization::PCLVisualizer& viewer)
{
	viewer.removeShape("line", 0);
	viewer.addArrow(o1, o2, 1.0, 0.0, 0.0, "line", 0);
}

double findDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr c2)
{
  float min = -1.0;
  float dist = 0.0;
  float x, y, z;
  
	for (int c1_point = 0; c1_point < c1->points.size(); c1_point++)
	{
		for (int c2_point = 0; c2_point < c2->points.size(); c2_point++)
		{
		  x = pow((c1->points[c1_point].x) - (c2->points[c2_point].x), 2);
		  y = pow((c1->points[c1_point].y) - (c2->points[c2_point].y), 2);
		  z = pow((c1->points[c1_point].z) - (c2->points[c2_point].z), 2);
		  
		  dist = sqrt(x + y + z);
		  
		  if (dist < min || min == -1.0) {
		    min = dist;
		    
		    o1.x = c1->points[c1_point].x;
		    o1.y = c1->points[c1_point].y;
		    o1.z = c1->points[c1_point].z;
		
		    o2.x = c2->points[c2_point].x;
		    o2.y = c2->points[c2_point].y;
		    o2.z = c2->points[c2_point].z;            
		  }
		}
	}
	return min;
} 

void label (pcl::visualization::PCLVisualizer& viewer)
{
	for(size_t i = 0; i < labels.size(); i++)
	{
	  labels[i].z = labels[i].z - .05;
	  char f[5];
	  sprintf(f,"%d",i);
	  viewer.addText3D (f, labels[i], 0.02,0,0,0,f,0);
	}
}

int  main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read (argv[1], *RGBcloud);

  std::cout << "RGBPointCloud before filtering has: " << RGBcloud->points.size () << " data points." << std::endl; //*

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudF (new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read (argv[1], *cloudF);

	//pcl::visualization::CloudViewer viewer("Cloud Viewer");

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
  clouds.reserve(10);
  labels.reserve(10);

  for (size_t i=0; i < RGBcloud->points.size();++i)
  {
    if (!((RGBcloud->points[i].x > -.35 && RGBcloud->points[i].x < .35) && (RGBcloud->points[i].y > -.35 && RGBcloud->points[i].y < .35)))///Bounding
    {
      RGBcloud->points[i].x = 0;
      RGBcloud->points[i].y = 0;
      RGBcloud->points[i].z = 0;
    }
  }
 
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);	
  vg.setInputCloud (RGBcloud);
  vg.setLeafSize (0.001f, 0.001f, 0.001f);
  vg.filter (*cloud_filtered);
	//cloud_filtered = cloud;
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

	//pcl::visualization::CloudViewer viewer("Cloud Viewer");

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PCDWriter writer;
  
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.0085);//lesser the values, more points seep into the table

  // Segment the largest planar component from the remaining cloud until 30% of the points remain
  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    seg.setInputCloud(cloud_filtered);
    seg.segment (*inliers, *coefficients); //*

    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Write the planar inliers to disk
    extract.filter (*cloud_plane); //*
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
		//viewer.showCloud(cloud_plane, "cloud_name");
		//std::cin.get();
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered); //*
    std::cerr <<" The Coefficients are: " << coefficients->values[0]<< " "<< coefficients->values[1]<< " "<< coefficients->values[2]<< " " << coefficients->values[3]<< " "<< std::endl;
  }

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	//cloud_filtered2 = cloud_filtered;


	////////////////////////Table has been awesomely removed ///////////////////////
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Rcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Gcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Bcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Ycloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	//Rcloud=cloud_filtered;
	//Gcloud=cloud_filtered;
	//Bcloud=cloud_filtered;
	//Rcloud->points.resize(cloud_filtered->height * cloud_filtered->width);

	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it=cloud_filtered->begin(); it != cloud_filtered->end(); /* it gets updated conditionally */ )
	{
		/* this is a bad 'red' classifier... it also removes yellows for example */
		if ( (*it).r > (*it).g && (*it).r > (*it).b ) 
		{
			cloud_filtered->erase(it);
		} else {
			it++;
		}
	}
  std::cout << "PointCloud after Red filtering: " << cloud_filtered->points.size ()  << " data points." << std::endl;

  Ycloud  =  Rcloud;

	/*
  for (size_t i=0; i < cloud_filtered->points.size();++i)//Red extraction
	{
    if ((Rcloud->points[i].r > Rcloud->points[i].g && (int(Rcloud->points[i].g) - int(Rcloud->points[i].b)) > 30 ))///Bounding
    {
      Rcloud->points[i].r = 0;
      Rcloud->points[i].g = 0;
      Rcloud->points[i].b = 0;
      Rcloud->points[i].x = 0;
      Rcloud->points[i].y = 0;
      Rcloud->points[i].z = 0;
    }
	}
 
	for (size_t i=0; i < cloud_filtered->points.size();++i)//yellow extraction
  {
    if (!(Rcloud->points[i].r > Rcloud->points[i].g && (int(Rcloud->points[i].g) - int(Rcloud->points[i].b)) > 30 ))///Bounding
    {
      Ycloud->points[i].r = 0;Rcloud
      Ycloud->points[i].g = 0;
      Ycloud->points[i].b = 0;
      Ycloud->points[i].x = 0;
      Ycloud->points[i].y = 0;
      Ycloud->points[i].z = 0;
    }
  }
  
	for (size_t i=0; i < cloud_filtered->points.size();++i)///green extraction
  {
    if (!(cloud_filtered->points[i].g > cloud_filtered->points[i].r && cloud_filtered->points[i].g > cloud_filtered->points[i].b))///Bounding
    {
      Gcloud->points[i].r = 0;
      Gcloud->points[i].g = 0;
      Gcloud->points[i].b = 0;
      Gcloud->points[i].x = 0;
      Gcloud->points[i].y = 0;
      Gcloud->points[i].z = 0;
	
    }
  }

  for (size_t i=0; i < cloud_filtered->points.size();++i)//blue extraction
  {
    if (!(cloud_filtered->points[i].b > cloud_filtered->points[i].g && cloud_filtered->points[i].b > cloud_filtered->points[i].r))///Bounding
    {
      Bcloud->points[i].r = 0;
      Bcloud->points[i].g = 0;
      Bcloud->points[i].b = 0;
      Bcloud->points[i].x = 0;
      Bcloud->points[i].y = 0;
      Bcloud->points[i].z = 0;
    }
  }
 */
  
	std::cerr<<"Waiting 1 "<<std::endl;

	// Creating the KdTree object for the search method of the extraction
  pcl::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
  tree->setInputCloud ( cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.01); // 1cm /// decreasing makes more clusters
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (4000);
  ec.setSearchMethod (tree);
  ec.setInputCloud(  cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back ( cloud_filtered->points[*pit]); //*
    
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    clouds.push_back(cloud_cluster);
    labels.push_back(cloud_cluster->points[0]);
    j++;
  }

	std::cerr<<"Waiting 3 "<<std::endl;
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	//viewer.showCloud(cloudF, "Full cloud");
	
  viewer.runOnVisualizationThreadOnce (label);
	
	char c_name[] = "Cloud No: ";
	char cloud_name [20];
	std::cerr<<"Total Clusters: "<<j<<std::endl;

	for (size_t k = 0; k < j ; k++)
	{	
		std::sprintf(cloud_name,"%s%d",c_name,k);
		viewer.showCloud(clouds[k], cloud_name);
	}

	std::cin.get();
  int xx, yy;
  while(1)
  {
    cout << "Enter the label of the first cloud: ";
		cin >> xx;
	
		cout << "Enter the label of the second cloud: ";
		cin >> yy;
		
		cout << "Finding distance...";
		cout << findDistance(clouds[xx], clouds[yy]) << endl;
		
		viewer.runOnVisualizationThreadOnce (viewerOneOff);
  }

	// busy wait
  while (!viewer.wasStopped ())
  {
  }

  return 0;
}
