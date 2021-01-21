#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl/features/fpfh.h>    //FPFH
#include <pcl/visualization/pcl_plotter.h>//显示描述子 
#include <pcl/common/transforms.h>
using namespace pcl;
void writepcd(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string filename="cloud")
{
       pcl::PCDWriter writer;
       std::string pcd=".pcd";
       writer.write<pcl::PointXYZ> (filename+pcd, *cloud, false);
}
pcl::PointCloud<pcl::FPFHSignature33> cal_fpfh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
        //估计法线
       pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
       ne.setInputCloud(cloud);
       pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
       ne.setSearchMethod(tree);
       pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
       ne.setRadiusSearch(0.06);      //使用半径在查询点周围0.6范围内的所有邻元素
       ne.compute(*cloud_normals);     //计算法线

        //PHFH
       pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
       fpfh.setInputCloud(cloud);
       fpfh.setInputNormals(cloud_normals);
       pcl::search::KdTree<PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);//创建一个空的kd树表示法
       fpfh.setSearchMethod(tree1);//输出的数据集
       pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
       //使用半径内0.8里面范围所有元素
       fpfh.setRadiusSearch(0.08);//使用半径必须大于估计法线时使用的半径
       fpfh.compute(*fpfhs);
       return *fpfhs;
}
int main(int argc, char **argv)
{
       //读取点云
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
       if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
       {
              PCL_ERROR("Cloudn't read file!");
              system("pause");
              return -1;
       }

       pcl::visualization::PCLPlotter plotter;
       Eigen::Affine3f transform = Eigen::Affine3f::Identity();
       transform.translation() << 1, 0.0, 0.0;
       double theta =-M_PI/2;  // char *the=argv[2];
       transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
       pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
       pcl::transformPointCloud (*cloud, *transformed_cloud, transform); 
       // std::stringstream filename;
       // std::string inputname=argv[1];
       // size_t pos=inputname.find(".");
       // inputname=inputname.substr(0,pos);
       // filename<<inputname<<"_trans";
       // writepcd(transformed_cloud,filename.str());
       // return 0;

       
       pcl::PointCloud<pcl::FPFHSignature33> fpfh_cloud=cal_fpfh(cloud);
       pcl::PointCloud<pcl::FPFHSignature33> fpfh_cloud2=cal_fpfh(transformed_cloud);
        pcl::PointCloud<pcl::FPFHSignature33> onep=fpfh_cloud;
       float histogram[33];
       float histogram2[33];
       for(const auto & pf:fpfh_cloud)
              for(int ele=0;ele<33;ele++)
                     histogram[ele]=(histogram[ele]+pf.histogram[ele]>FLT_MAX)?FLT_MAX:histogram[ele]+pf.histogram[ele];
       for(const auto & pf:fpfh_cloud2)
              for(int ele=0;ele<33;ele++)
                     histogram2[ele]=(histogram2[ele]+pf.histogram[ele]>FLT_MAX)?FLT_MAX:histogram2[ele]+pf.histogram[ele];

       std::vector<double> n,his;
       std::cerr<<"reach!"<<std::endl;
       for(int ele=0;ele<33;ele++)
       {
              onep[0].histogram[ele]=histogram[ele];
              onep[1].histogram[ele]=histogram2[ele];
       }
              
       
       // for(int ele=0;ele<33;ele++)  
       // {
       //        if(histogram[ele]>1)
       //        {
       //               // std::cerr<<histogram[ele]<<"  ";
       //               n.push_back(ele);
       //               his.push_back(histogram[ele]);
       //        }
       // }
       // plotter.addFeatureHistogram<pcl::FPFHSignature33>(fpfh_cloud,"fpfh", 60); 
       // plotter.addFeatureHistogram<pcl::FPFHSignature33>(fpfh_cloud2,"fpfh", 59);    
       plotter.addFeatureHistogram<pcl::FPFHSignature33>(fpfh_cloud,"fpfh", 0);    
       plotter.addFeatureHistogram<pcl::FPFHSignature33>(fpfh_cloud,"fpfh", 1);    

       //可视化
       pcl::visualization::PCLVisualizer viewer("PCL Viewer");
       viewer.setBackgroundColor(0.0, 0.0, 0.0);
//        viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals, 1, 0.4, "normals");
       viewer.addPointCloud(cloud,"cloud1");
       viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2, "cloud1");
       while (!viewer.wasStopped())
       {
              plotter.plot();
              viewer.spinOnce(100);
       }
       return 0;
}