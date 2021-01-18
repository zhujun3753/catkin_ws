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
#include <vector>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_plotter.h>

typedef pcl::PointXYZ PointT;
bool ascend (pcl::PointXYZ a,pcl::PointXYZ b) { return (a.y<b.y); }
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
pcl::visualization::PCLPlotter * plot2d(const cyl_seg_input & parain)
{
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  // cloud=parain.cloud_filtered;
  pcl::copyPointCloud(*parain.cloud_filtered,*cloud); // 这种方式两个指针指向不同的地址
  std::sort(cloud->begin(),cloud->end(),ascend);
  
  std::vector<double> X,Y,Z,n;
  int i=0;
  for(const auto & point:*cloud )
  {
    n.push_back(i);
    X.push_back(point.x);
    Y.push_back(point.y);
    Z.push_back(point.z);
    i++;
  }
  // std::vector<double> X1,Y1,Z1,n1;
  // int i1=0;
  // for(const auto & point:*parain.cloud_filtered )
  // {
  //   n1.push_back(i1);
  //   X1.push_back(point.x);
  //   Y1.push_back(point.y);
  //   Z1.push_back(point.z);
  //   i1++;
  // }
  // pcl::PointXYZ a(0,Y[Y.size()-1],0),b(0,0,0);
  // a.y=0;
  // std::string sphere_name="sphere";
  // for(int j=0; j<10;j++)
  // {
  //   a.y+=Y[Y.size()-1]/10;
  //   viewer. addSphere(a,0.1,1,0,0,sphere_name+std::to_string(j));
  // }
  // viewer.addLine(a,b,0,1,0,"line");
  // viewer.addCube(0,1,Y[Y.size()-1],0,0,1,0,0,1,"cube");
  // addCube (float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,
  //                double r = 1.0, double g = 1.0, double b = 1.0, const std::string &id = "cube", int viewport = 0);
  //头文件：#include <pcl/visualization/pcl_plotter.h>
	pcl::visualization::PCLPlotter *plot_(new pcl::visualization::PCLPlotter("Elevation and Point Number Breakdown Map"));
	plot_->setBackgroundColor(1, 1, 1);
	plot_->setTitle("Elevation and Point Number Breakdown Map");
	plot_->setXTitle("Elevation");
	plot_->setYTitle("Point number");
	plot_->addPlotData(Y,n, "display", vtkChart::POINTS);//X,Y均为double型的向量
  // plot_->addPlotData(n1, Y1, "display", vtkChart::LINE);//X,Y均为double型的向量
	plot_->plot();//绘制曲线
  return plot_;
}
cyl_seg_output cly_seg(const cyl_seg_input & parain)
{
  cyl_seg_output  resultout;
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC); // 随机抽样一致算法
    // * SAC_RANSAC - RANdom SAmple Consensus
    // * SAC_LMEDS - Least Median of Squares
    // * SAC_MSAC - M-Estimator SAmple Consensus
    // * SAC_RRANSAC - Randomized RANSAC
    // * SAC_RMSAC - Randomized MSAC
    // * SAC_MLESAC - Maximum LikeLihood Estimation SAmple Consensus
    // * SAC_PROSAC - PROgressive SAmple Consensus
  seg.setNormalDistanceWeight (0.1); // Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal.
  seg.setMaxIterations (10000); 
  seg.setDistanceThreshold (0.04);
  seg.setRadiusLimits (0.32, 0.35);
  seg.setInputCloud (parain.cloud_filtered);
  seg.setInputNormals (parain.cloud_normals);
  seg.segment (*inliers_cylinder, *coefficients_cylinder);// Obtain the cylinder inliers and coefficients指向点的索引的指针，并不是指向点的指针
  
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
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
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

  // 下面这些等号是左边的指针指向右边指针所指向的地址，两个指针同时指向一段地址，因此对任意一指针的操作都会影响数据
  // 好在后面指针的作用域只在此函数，如果在循环中，那很有可能出问题！
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

  // 读取数据
  reader.read (argv[1], *cloud);
  std::cerr << "PointCloud has: " << cloud->size () << " data points." << std::endl;

  // 旋转处理
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 0, 0.0, 0.0;
  double theta =-M_PI/40;// char *the=argv[2]; strtod(the,NULL);// 读取数值
  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
  cloud=transformed_cloud;

  // 统计滤波
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  // 估计点的法向量
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (25);
  ne.compute (*cloud_normals);
  std::cerr << "法向量: " << cloud_normals->size()<<"           "<< cloud_filtered->size()<<std::endl;
// 可视化
 pcl::visualization::PCLVisualizer viewer ("Cylinder Segmentation");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_filtered_color_handler (cloud_filtered, 255, 0, 0);
  viewer.addPointCloud (cloud_filtered, cloud_filtered_color_handler, "original_cloud");
  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_filtered, cloud_normals);
  viewer.addCoordinateSystem (1.0);

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }
  return (0);
}