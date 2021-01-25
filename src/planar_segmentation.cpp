#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>


int main (int argc, char** argv)
{
  // 读取点云数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[1], *cloud) ;
  std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.6, 5);
  pass.filter (*cloud);


  // 计算点云法向量
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  // Use all neighbors in a sphere of radius 1cm
  ne.setRadiusSearch(1); 
  //ne.setKSearch(20);
  ne.compute(*normals);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

  //采用RANSAC提取平面
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
  pcl::PCDWriter writer;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients(true);//设置对估计的模型系数需要进行优化
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE); //设置分割模型
  seg.setNormalDistanceWeight(0.1);//设置表面法线权重系数
  seg.setMethodType(pcl::SAC_RANSAC);//设置采用RANSAC作为算法的参数估计方法
  seg.setMaxIterations(500); //设置迭代的最大次数
  seg.setDistanceThreshold(0.5); //设置内点到模型的距离允许最大值
  seg.setInputCloud(cloud);
  seg.setInputNormals(normals);
  // Obtain the plane inliers and coefficients
  seg.segment(*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
  // Extract the planar inliers from the input cloud
  extract.setInputCloud(cloud);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);




  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg1;
  // Optional
  seg1.setOptimizeCoefficients (true);
  // Mandatory
  seg1.setModelType (pcl::SACMODEL_PLANE);
  seg1.setMethodType (pcl::SAC_RANSAC);
  seg1.setDistanceThreshold (0.01);

  seg1.setInputCloud (cloud);
  seg1.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  //   for (std::size_t i = 0; i < inliers->indices.size (); ++i)
  //   for (const auto& idx: inliers->indices)
  //     std::cerr << idx << "    " << cloud->points[idx].x << " "
  //                                << cloud->points[idx].y << " "
  //                                << cloud->points[idx].z << std::endl;
  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
    // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 0, 0);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");
    pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (coefficients->values[0]);
  coeffs.values.push_back (coefficients->values[1]);
  coeffs.values.push_back (coefficients->values[2]);
  coeffs.values.push_back (coefficients->values[3]);
  viewer.addPlane (coeffs, "plane");
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }
  return (0);
}