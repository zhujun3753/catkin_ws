#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/moment_of_inertia_estimation.h>

struct color
{
    double r;
    double g;
    double b;
    double size; // 点的大小

    // 使用了构造函数的参数初始化列表，可以只对部分成员变量初始化，这里有个好处是可以使用默认参数,这样结构体就可以赋予默认值
    color (double r =0,double g=0, double b=0,double size=1) : r (r),g (g),b (b),size (size){}
};

int 
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read (argv[1], *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
//   pcl::VoxelGrid<pcl::PointXYZ> vg;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//   vg.setInputCloud (cloud);
//   vg.setLeafSize (0.01f, 0.01f, 0.01f);
//   vg.filter (*cloud_filtered);
//   std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
//   pcl::SACSegmentation<pcl::PointXYZ> seg;
//   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
//   seg.setOptimizeCoefficients (true);
//   seg.setModelType (pcl::SACMODEL_PLANE);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setMaxIterations (100);
//   seg.setDistanceThreshold (0.02);

//   int i=0, nr_points = (int) cloud_filtered->size ();
//   while (cloud_filtered->size () > 0.3 * nr_points)
//   {
    // Segment the largest planar component from the remaining cloud
//     seg.setInputCloud (cloud_filtered);
//     seg.segment (*inliers, *coefficients);
//     if (inliers->indices.size () == 0)
//     {
//       std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//       break;
//     }

    // Extract the planar inliers from the input cloud
//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     extract.setInputCloud (cloud_filtered);
//     extract.setIndices (inliers);
//     extract.setNegative (false);

    // Get the points associated with the planar surface
//     extract.filter (*cloud_plane);
//     std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
//     extract.setNegative (true);
//     extract.filter (*cloud_f);
//     *cloud_filtered = *cloud_f;
//   }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.2); // 2cm
  ec.setMinClusterSize (1000);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  pcl::visualization::PCLVisualizer viewer ("Cluster cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_filtered_color_handler (cloud, 255, 255, 255);
  viewer.addPointCloud (cloud, cloud_filtered_color_handler, "original_cloud");
  viewer.addCoordinateSystem (1.0);
  int j = 0;
  std::cout << "Find " << cluster_indices.size () << "  clusters." << std::endl;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->push_back ((*cloud)[*pit]); //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j ;//<< ".pcd";
    // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moment;
    moment.setInputCloud(cloud_cluster);
    moment.compute();
    Eigen::Vector3f center, vx, vy, vz;
    float l1,l2,l3;
    moment.getEigenVectors(vx, vy, vz);
    moment.getEigenValues(l1,l2,l3);
    std::cout<<"Major: "<<l1<<"middle: "<<l2<<" "<<l3<<std::endl;
    int r,g,b,gap;
    gap=256*256*256/cluster_indices.size () ;
    // std::cout<<"gap"<<gap<<std::endl;
    // r=(j*gap)%256;
    // g=((j*gap)/256)%256;
    // b=(((j*gap)/256)/256)%256;
    r=(l2>l3*100)?(j*gap)%256:255;
    g=(l2>l3*100)?((j*gap)/256)%256:0;
    b=(l2>l3*100)?(((j*gap)/256)/256)%256:0;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>cloud_cluster_color_handler (cloud_cluster, r, g, b);
    viewer.addPointCloud (cloud_cluster, cloud_cluster_color_handler, ss.str ());
    

  }
  while (!viewer.wasStopped ()) 
  { 
    viewer.spinOnce ();
  }

  return (0);
}