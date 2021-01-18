#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
struct cyl_seg_input
{
  pcl::PointCloud<PointT>::Ptr cloud_filtered ;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals ;

  
};
struct cyl_seg_output
{
  bool exist_cly;
  pcl::ModelCoefficients::Ptr coefficients_cylinder;
  pcl::PointIndices::Ptr inliers_cylinder; 
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 ;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 ;
  pcl::PointCloud<PointT>::Ptr cloud_cylinder;
  
};

cyl_seg_output cly_seg(const cyl_seg_input & parain)
{
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  
  cyl_seg_output  resultout;

  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0.3, 0.35);
  seg.setInputCloud (parain.cloud_filtered);
  seg.setInputNormals (parain.cloud_normals);
  seg.segment (*inliers_cylinder, *coefficients_cylinder);// Obtain the cylinder inliers and coefficients指向点的索引的指针，并不是指向点的指针
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
  
  extract.setInputCloud (parain.cloud_filtered);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
  {
    std::cerr << "Can't find the cylindrical component." << std::endl;
    resultout.exist_cly=false;
    return resultout;
  } 
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size () << " data points." << std::endl;
    resultout.exist_cly=true;
  }

  // 从输入点云中提取去除圆柱点云的数据
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  // 从输入法向量中提取去除圆柱点云法向量的数据
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (parain.cloud_normals);
  extract_normals.setIndices (inliers_cylinder);
  extract_normals.filter (*cloud_normals2);
  // std::cerr << "Test end!!!." << std::endl;

  resultout.coefficients_cylinder=coefficients_cylinder;
  resultout.inliers_cylinder=inliers_cylinder;
  resultout.cloud_filtered2=cloud_filtered2;
  resultout.cloud_normals2=cloud_normals2;
  resultout.cloud_cylinder=cloud_cylinder;
  return resultout;
}






int main (int argc, char** argv)
{
  // All the objects needed
  pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients), coefficients_cylinder2(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices), inliers_cylinder2 (new pcl::PointIndices);

  // Read in the cloud data
  reader.read (argv[1], *cloud);
  std::cerr << "PointCloud has: " << cloud->size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 3);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-2, 1.5);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-16, 0);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;

  // 估计点的法向量
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03); // 0.03
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  seg.segment (*inliers_plane, *coefficients_plane);// 获得指向平面点的索引的指针以及平面点的参数ax+by+cz+d=0
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // 从输入点云中提取平面点云
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
  // writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0.3, 0.35);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  seg.segment (*inliers_cylinder, *coefficients_cylinder);// Obtain the cylinder inliers and coefficients指向点的索引的指针，并不是指向点的指针
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
  // cylinder_coeff.values[0] = pt_on_axis.x ();
  // cylinder_coeff.values[1] = pt_on_axis.y ();
  // cylinder_coeff.values[2] = pt_on_axis.z ();
  // cylinder_coeff.values[3] = axis_direction.x ();
  // cylinder_coeff.values[4] = axis_direction.y ();
  // cylinder_coeff.values[5] = axis_direction.z ();
    // Write the cylinder inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size () << " data points." << std::endl;
	  // writer.write ("cloud_cylinder.pcd", *cloud_cylinder, false);
  }
  // 从输入点云中提取去除圆柱点云的数据
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  // 从输入法向量中提取去除圆柱点云法向量的数据
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_cylinder);
  extract_normals.filter (*cloud_normals2);

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0.3, 0.35);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);
  seg.segment (*inliers_cylinder2, *coefficients_cylinder2);// Obtain the cylinder inliers and coefficients指向点的索引的指针，并不是指向点的指针
  std::cerr << "Cylinder coefficients2: " << *coefficients_cylinder2 << std::endl;
  pcl::PointCloud<PointT>::Ptr cloud_cylinder2 (new pcl::PointCloud<PointT> ());
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder2);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder2);
  if (cloud_cylinder2->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder2->size () << " data points." << std::endl;
	  // writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
  }



  // resultout.coefficients_cylinder=coefficients_cylinder;
  // resultout.inliers_cylinder=inliers_cylinder;
  // resultout.cloud_filtered2=cloud_filtered2;
  // resultout.cloud_normals2=cloud_normals2;
  // resultout.cloud_cylinder=cloud_cylinder;
  cyl_seg_input parain;
  parain.cloud_filtered=cloud_filtered;
  parain.cloud_normals=cloud_normals;
  cyl_seg_output resultout=cly_seg(parain);
  // 可视化
 pcl::visualization::PCLVisualizer viewer ("Cylinder Segmentation");
   // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_filtered_color_handler (cloud_filtered, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_plane_color_handler (cloud_plane, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_cylinder_color_handler (cloud_cylinder, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_cylinder2_color_handler (cloud_cylinder, 0, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (cloud_filtered, cloud_filtered_color_handler, "original_cloud");
  viewer.addPointCloud (cloud_plane, cloud_plane_color_handler, "cloud_plane");
  viewer.addPointCloud (resultout.cloud_cylinder, cloud_cylinder_color_handler, "cloud_cylinder");
  viewer.addPointCloud (cloud_cylinder2, cloud_cylinder2_color_handler, "cloud_cylinder2");
  viewer.addCylinder (*coefficients_cylinder);
  viewer.addCoordinateSystem (1.0);
  // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "cloud_cylinder");
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }



  return (0);
}