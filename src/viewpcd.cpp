#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZ PointT;
void showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}
bool descend (pcl::PointXYZ a,pcl::PointXYZ b) { return (a.y>b.y); }
// This is the main function
int main (int argc, char** argv)
{

  // Show help
  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
    showHelp (argv[0]);
    return 0;
  }

  // Fetch point cloud filename in arguments | Works with PCD and PLY files
  std::vector<int> filenames;
  bool file_is_pcd = false;

  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
        // 返回:
        // a vector with file names indices 返回文件名的索引
  if (filenames.size () != 1)  {
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

    if (filenames.size () != 1) {
      showHelp (argv[0]);
      return -1;
    } else {
      file_is_pcd = true;
    }
  }

  // Load file | Works with PCD and PLY files
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  if (file_is_pcd) {
    if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  } else {
    if (pcl::io::loadPLYFile (argv[filenames[0]], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  }


  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.translation() << 0, 0.0, 0.0;
  // char *the=argv[2];
  double theta =-M_PI/50;
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  std::cout << transform_2.matrix() << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);
  source_cloud=transformed_cloud;

  pcl::visualization::PCLVisualizer viewer ("View PCL");
   // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");
  viewer.addCoordinateSystem (1.0);
  
  std::sort(source_cloud->begin(),source_cloud->end(),descend);
  
  std::vector<double> X,Y,Z,n;
  int i=0;
  for(const auto & point:*source_cloud )
  {
    n.push_back(i);
    X.push_back(point.x);
    Y.push_back(point.y);
    Z.push_back(point.z);
    i++;
  }
  pcl::PointXYZ a(0,Y[Y.size()-1],0),b(0,0,0);
  a.y=0;
  std::string sphere_name="sphere";
  for(int j=0; j<10;j++)
  {
    a.y+=Y[Y.size()-1]/10;
    viewer. addSphere(a,0.1,1,0,0,sphere_name+std::to_string(j));
  }
  viewer.addLine(a,b,0,1,0,"line");
  viewer.addCube(0,1,Y[Y.size()-1],0,0,1,0,0,1,"cube");
  // addCube (float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,
  //                double r = 1.0, double g = 1.0, double b = 1.0, const std::string &id = "cube", int viewport = 0);
  //头文件：#include <pcl/visualization/pcl_plotter.h>
	pcl::visualization::PCLPlotter *plot_(new pcl::visualization::PCLPlotter("Elevation and Point Number Breakdown Map"));
	plot_->setBackgroundColor(1, 1, 1);
	plot_->setTitle("Elevation and Point Number Breakdown Map");
	plot_->setXTitle("Elevation");
	plot_->setYTitle("Point number");
	plot_->addPlotData(n, Y, "display", vtkChart::LINE);//X,Y均为double型的向量
	plot_->plot();//绘制曲线


  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

  return 0;
}