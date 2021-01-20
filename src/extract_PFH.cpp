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
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/segmentation/extract_clusters.h>
#include <fstream>
#include <limits.h>
#include <pcl/visualization/pcl_plotter.h>

/*PFH(Point Feature Histograms ) 点特征直方图           k近邻（距离小于半径r的点）
 * 1. PFH公式的目标是通过使用多维值直方图概括点周围的平均曲率来编码点的k邻域几何特性。
 * 这个高维的超空间为特征表示提供了一个信息性的特征，对下垫面的6D姿态是不变的(旋转，平移不变)，
 * 并且能够很好地处理不同的采样密度或噪声水平。
 * 
 * 2. 点特征直方图表示是基于k邻域中的点与其估计的曲面法线之间的关系。
 * 它试图通过考虑估计法线方向之间的所有相互作用，尽可能最好地捕捉采样的曲面变化。
 * 因此，生成的超空间依赖于每个点的曲面法线估计的质量。
 * 
 * 3. 最后的PFH描述符被计算为邻域中所有点对之间关系的直方图，计算复杂度为O(K^2)
 * 计算完中心点领域内 n 个点之间的所有三元组，一共会得到 C^2_n 个三元组。
 * 通常情况下，三元组中的每个特征值会被分成 b 等分，所以该三元组会形成一个 b^3 维的直方图，
 * 每一个维代表了其中某个值的某一个范围区间。然后，再去统计出现在各个子区间的频率即可。
 * 在实际项目中计算 PFH 时，我们的设置 b=5，即把每个特征值分为5等分，因此 PFH 是一个125维的向量。
 * 
 */
typedef pcl::PointXYZ PointT;
struct Data
{
  std::vector<double> n,his;
};

bool descend (pcl::PointXYZ a,pcl::PointXYZ b) { return (a.z>b.z); }
pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
pcl::NormalEstimation<PointT, pcl::Normal> ne;

Data plot_pfh(pcl::PointCloud<pcl::PointXYZ>::Ptr source,  int bias=0)
{
  Data data;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
  
  
  // std::sort(source->begin(),source->end(),descend);
// 计算法向量
  ne.setSearchMethod (tree);
  ne.setInputCloud (source);
  ne.setKSearch (25);
  ne.compute (*normals);

   // 估计点特征直方图
  
  pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());
  pfh.setInputCloud (source);
  pfh.setInputNormals (normals);
  pfh.setSearchMethod (tree);
  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  pfh.setRadiusSearch (0.05);
  pfh.compute (*pfhs);  //  // pfhs->points.size ()应该与input cloud->points.size ()有相同的大小，即每个点都有一个pfh特征向量
  // std::cerr<<std::endl<<*pfhs->begin()<<std::endl<<*pfhs->end()<<std::endl;
  float histogram[125];
  // ofstream OutFile("Test.txt"); 
  for(const auto & pf:*pfhs)
  {
    for(int ele=0;ele<125;ele++)
    {
        // OutFile <<std::to_string(pf.histogram[ele])<<"  ";;
        histogram[ele]=(histogram[ele]+pf.histogram[ele]>FLT_MAX)?FLT_MAX:histogram[ele]+pf.histogram[ele];
    }
    // OutFile <<std::endl<<std::endl;;
  }
  // OutFile.close();

  std::vector<double> n,his;
  std::cerr<<std::endl;
  for(int ele=0;ele<125;ele++)  
  {
    if(histogram[ele]>1)
    {
      std::cerr<<histogram[ele]<<"  ";
      n.push_back(ele+bias);
      his.push_back(histogram[ele]);
    }
  }
  data.n=n;
  data.his=his;
  // plot_->addPlotData(n, his, name, vtkChart::LINE);//X,Y均为double型的向量
  return data;
}

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr source2 (new pcl::PointCloud<pcl::PointXYZ> ());
  
  // 读取文件
  pcl::io::loadPCDFile (argv[1], *source);
  pcl::io::loadPCDFile (argv[2], *source2);

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 1, 0.0, 0.0;
  double theta =-M_PI/2;  // char *the=argv[2];
  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*source, *transformed_cloud, transform); 

  pcl::visualization::PCLPlotter *plot_(new pcl::visualization::PCLPlotter("Elevation "));
  pcl::visualization::PCLPlotter *plot_2(new pcl::visualization::PCLPlotter("Elevation and Point Number Breakdown Map"));
  
	// plot_->setBackgroundColor(1, 1, 1);
  // plot_->setShowLegend(true);
  // plot_->plot();
  
  // while (plot_->wasStopped())
  // {
    Data data,data2;
    data=plot_pfh(source);	
    data2=plot_pfh(transformed_cloud,0);	
    plot_->addPlotData(data.n,data.his);
    plot_->plot();
    plot_2->addPlotData(data2.n,data2.his);
    plot_2->plot();
	  //绘制曲线
    // plot_->spinOnce(2000);
    // plot_->clearPlots();
  // }
  
  // plot_->wasStopped()

  // pcl::visualization::PCLPlotter *plot2(new pcl::visualization::PCLPlotter("Elevation and Point Number Breakdown Map"));
	// plot2->setBackgroundColor(1, 1, 1);
	// plot2->setTitle("Elevation and Point Number Breakdown Map");
	// plot2->setXTitle("Elevation");
	// plot2->setYTitle("Point number");
	// plot2->addPlotData(n, his, "display", vtkChart::LINE);//X,Y均为double型的向量
	// plot2->plot();//绘制曲线
  // plot_->wasStopped()



  // while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
  //   viewer.spinOnce ();
  // }
  




  // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  // transform.translation() << 0, 0.0, 0.0;
  // double theta =-M_PI/50;  // char *the=argv[2];
  // transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // pcl::transformPointCloud (*source, *transformed_cloud, transform);
  // source=transformed_cloud; // 更换指针指向的地址


  // pcl::visualization::PCLVisualizer viewer ("View PCD");
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color_handler (source, 255, 255, 255);
  // viewer.addPointCloud (source, source_color_handler, "original_cloud");
  // viewer.addCoordinateSystem (1.0);
  // while (!viewer.wasStopped ()) { 
  //   viewer.spinOnce ();
  // }
  return 0;
}