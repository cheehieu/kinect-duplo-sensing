/*
 * convex_hull_tests.cpp
 *
 *  Created on: Sep 19, 2011
 *      Author: aitor
 */

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

int
main (int argc, char ** argv)
{

  typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
  typedef Cloud::Ptr CloudPtr;
  CloudPtr cube (new Cloud ());
  CloudPtr plane (new Cloud ());

  //generate cloud (CUBE)
  int side = 10 + 1;
  cube->points.resize (pow (side, 3));
  cube->width = cube->points.size ();
  cube->height = 1;

  int p = 0;
  for (size_t i = 0; i < side; i++)
    for (size_t j = 0; j < side; j++)
      for (size_t k = 0; k < side; k++, p++)
      {
        cube->points[p].getVector3fMap () = Eigen::Vector3f (i, j, k);
      }

  //generate plane
  side = 6;
  plane->points.resize (pow (side, 2));
  plane->width = plane->points.size ();
  plane->height = 1;

  p = 0;
  for (size_t i = 0; i < side; i++)
    for (size_t j = 0; j < side; j++,p++)
    {
      plane->points[p].getVector3fMap () = Eigen::Vector3f (i, j, 0);
    }

  {
    pcl::PointCloud < pcl::PointXYZ > hull;
    std::vector < pcl::Vertices > polygons;
    pcl::ConvexHull < pcl::PointXYZ > chull;
    chull.setInputCloud (cube);
    chull.setComputeAreaVolume (true);
    chull.reconstruct (hull, polygons);

    std::cout << "Area:" << chull.getTotalArea () << " Volume:" << chull.getTotalVolume () << std::endl;

    sensor_msgs::PointCloud2 msg_alpha;
    pcl::toROSMsg (hull, msg_alpha);

    pcl::PolygonMesh mesh;
    mesh.cloud = msg_alpha;
    mesh.polygons = polygons;

    pcl::visualization::PCLVisualizer vis ("Convex Hull");
    vis.addPointCloud (cube);
    vis.addPolygonMesh (mesh);
    vis.spin ();
  }

  {
    pcl::PointCloud < pcl::PointXYZ > hull;
    std::vector < pcl::Vertices > polygons;
    pcl::ConvexHull < pcl::PointXYZ > chull;
    chull.setInputCloud (plane);
    chull.setComputeAreaVolume (true);
    chull.reconstruct (hull, polygons);

    std::cout << "Area:" << chull.getTotalArea () << " Volume:" << chull.getTotalVolume () << std::endl;

    sensor_msgs::PointCloud2 msg_alpha;
    pcl::toROSMsg (hull, msg_alpha);

    pcl::PolygonMesh mesh;
    mesh.cloud = msg_alpha;
    mesh.polygons = polygons;

    pcl::visualization::PCLVisualizer vis ("Convex Hull");
    vis.addPointCloud (plane);
    vis.addPolygonMesh (mesh);
    vis.spin ();
  }

  {

    CloudPtr cloud(new Cloud());
    pcl::io::loadPCDFile("../data/ascii-unihill-w5.pcd", *cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1000.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud);


    pcl::PointCloud < pcl::PointXYZ > hull;
    std::vector < pcl::Vertices > polygons;
    pcl::ConvexHull < pcl::PointXYZ > chull;
    chull.setInputCloud (cloud);
    chull.setComputeAreaVolume (true);
    chull.reconstruct (hull, polygons);

    std::cout << "Area:" << chull.getTotalArea () << " Volume:" << chull.getTotalVolume () << std::endl;

    sensor_msgs::PointCloud2 msg_alpha;
    pcl::toROSMsg (hull, msg_alpha);

    pcl::PolygonMesh mesh;
    mesh.cloud = msg_alpha;
    mesh.polygons = polygons;

    pcl::visualization::PCLVisualizer vis ("Convex Hull");
    vis.addPointCloud (cloud);
    vis.addPolygonMesh (mesh);
    vis.spin ();
  }
}
