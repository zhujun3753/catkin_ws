 
#include <pcl/io/pcd_io.h>
 
#include <pcl/point_cloud.h>
 
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>
#include <vector>
 
using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
 
pcl::PointCloud<PointT>::Ptr cloud(new PointCloudT);
pcl::PointCloud<PointT>::Ptr clicked_points_3d(new PointCloudT);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
int num = 0;
 
vector<PointT> v;
PointT p;
 
//存储索引
vector< int > indices;
 
void Write_points()
{
	ofstream out("slect_points.txt");
	for (vector<PointT> ::iterator it = v.begin(); it != v.end(); ++it)
	{
		out << (*it).x << "  " << (*it).y << "  " << (*it).z << endl;
	}
}
 
 
void pp_callback_AreaSelect(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	if (event.getPointsIndices(indices) == -1)
		return;
	for (int i = 0; i < indices.size(); ++i)
	{
		clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
	}
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);
	std::stringstream ss;
	std::string cloudName;
	ss << num++;
	ss >> cloudName;
	cloudName += "_cloudName";
	viewer->addPointCloud(clicked_points_3d, red, cloudName);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
	//保存点云
	pcl::io::savePCDFileBinary(cloudName += ".pcd", *clicked_points_3d);
}
 
void
pp_callback_PointsSelect(const pcl::visualization::PointPickingEvent& event, void* args)
 
{
	if (event.getPointIndex() == -1)
		return;
	PointT current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	//将选中的点云存在Points里面
    clicked_points_3d->points.push_back(current_point);
 
	// 将选中的点赋为红色
 
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(clicked_points_3d, 255, 0, 0);
	viewer->removePointCloud("clicked_points");
	viewer->addPointCloud(clicked_points_3d, red, "clicked_points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
	p.x = current_point.x;
	p.y = current_point.y;
	p.z = current_point.z;
	v.push_back(p);
}
 
int main(int argc, char** argv)
{
	std::string filename(argv[1]);
	if (pcl::io::loadPCDFile(filename, *cloud))
	{
		std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
		return 0;
	}
	std::cout << cloud->points.size() << std::endl;
	viewer->addPointCloud(cloud, "bunny");
	viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
	viewer->registerPointPickingCallback(pp_callback_PointsSelect, (void*)&cloud);
	viewer->registerAreaPickingCallback(pp_callback_AreaSelect, (void*)&cloud);
	std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;
	viewer->spin();
	Write_points();//写出点数据
	std::cout << "done." << std::endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
 
}