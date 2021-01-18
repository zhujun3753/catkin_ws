#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
    
int 
main ()
{
    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);
    
    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
//     enum NormalEstimationMethod
// {
//   COVARIANCE_MATRIX, 创建9个积分图像以从其局部邻域的协方差矩阵计算特定点的法线
//   AVERAGE_3D_GRADIENT, 创建6个积分图像，以计算水平和垂直3D渐变的平滑版本，并使用这两个渐变之间的叉积计算法线。
//   AVERAGE_DEPTH_CHANGE 仅创建单个积分图像，并根据平均深度变化计算法线
// };
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
//      viewer.addCoordinateSystem ( 0);
//      viewer.initCameraParameters ();
       // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 0, 0);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");
    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }
    return 0;
}