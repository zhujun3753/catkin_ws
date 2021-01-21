// Feature.cpp : 定义控制台应用程序的入口点。
//
//  用于特征提取学习 

// NARF (Normal Aligned Radial Feature 法向径向特征对齐）

// FPFH (Fast Point Feature histograms 快速点特征直方图)

// SIFT (Scale-invariant Feature Transform 尺寸不变特征变换)

// BRIEF （Binary Robust Independent Elementary Features 二进制健壮的独立的基本特性)



#include <iostream>
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>  //FPFH
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>//omp加速计算
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

//为了使用fpfh特征匹配，声明一个计算fpfh特征点的函数
fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);

int main(int argc, char* argv[])
{

	clock_t start, end; //long
	start = clock();//开始时间
	pointcloud::Ptr source(new pointcloud);
	pointcloud::Ptr target(new pointcloud);
	pcl::io::loadPCDFile(argv[1], *target);
	pcl::io::loadPCDFile(argv[2], *source);
	cout << target->size() << endl;

	//精简
	pcl::PointCloud<pcl::PointXYZ> ::Ptr target_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> ::Ptr source_filtered(new pcl::PointCloud<pcl::PointXYZ>());

	// pcl::VoxelGrid<pcl::PointXYZ> sor;
	// sor.setInputCloud(target);
	// sor.setLeafSize(0.005, 0.005, 0.005);
	// sor.filter(*target_filtered);
        target_filtered=target; //      不必滤波
        source_filtered=source;


	// pcl::VoxelGrid<pcl::PointXYZ> sor1;
	// sor1.setInputCloud(source);
	// sor1.setLeafSize(0.005, 0.005, 0.005);
	// sor1.filter(*source_filtered);
	// cout << source_filtered->size() << "  " << target_filtered->size() << endl;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());//创建搜索树
	fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source_filtered, tree);//计算点云点特征直方图
	fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target_filtered, tree);

	//对齐  //输入参数 ①源点云+源点特征直方图 ②目标点云+目标点特征直方图
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(source_filtered);
	sac_ia.setSourceFeatures(source_fpfh);
	sac_ia.setInputTarget(target_filtered);
	sac_ia.setTargetFeatures(target_fpfh);
	pointcloud::Ptr final(new pointcloud);//
	
	//对齐参数设置
	sac_ia.setNumberOfSamples(3);     // 估计6DOF需要3个点		20
	sac_ia.setCorrespondenceRandomness(5);	// 匹配最近的5个描述子		6
	sac_ia.setMaximumIterations(50000);			// 100
	sac_ia.setEuclideanFitnessEpsilon(0.001);
	sac_ia.setTransformationEpsilon(1e-10);
	sac_ia.setRANSACIterations(30);
	// Set the maximum distance threshold between two correspondent points in source and target.
	// If the distance is larger, the points will be ignored in the alignment process.
	sac_ia.setMaxCorrespondenceDistance(1.0f);  // 允许的最大距离
	sac_ia.align(*final);
	if (sac_ia.hasConverged())
	{
		Eigen::Matrix4f transformation = sac_ia.getFinalTransformation();
		Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
		Eigen::Vector3f translation = transformation.block<3, 1>(0, 3);

		std::cout << "Transformation matrix:" << std::endl << std::endl;
		printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		std::cout << std::endl;
		printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	}
    	else std::cout << "Did not converge." << std::endl;
	cout <<"has converged:"<< sac_ia.hasConverged() <<"     score"<<sac_ia.getFitnessScore()<< endl;
	
	end = clock();
	cout << "calculate time is " << float(end - start) / CLOCKS_PER_SEC << endl;

	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh test"));
	int v1 = 0;
	int v2 = 1;
	view->createViewPort(0, 0, 0.5, 1, v1);
	view->createViewPort(0.5, 0, 1, 1, v2);
	view->setBackgroundColor(0, 0, 0, v1);
	view->setBackgroundColor(0.05, 0, 0, v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color(source_filtered, 255, 0, 0);
	view->addPointCloud(source_filtered, source_cloud_color, "sources_cloud_v1", v1);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target_filtered, 0, 255, 0);
	view->addPointCloud(target_filtered, target_cloud_color, "target_cloud_v1", v1);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1", v1);

	//
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligend_cloud_color(final, 255, 0, 0);
	view->addPointCloud(final, aligend_cloud_color, "aligend_cloud_v2", v2);

	view->addPointCloud(target_filtered, target_cloud_color, "target_cloud_v2", v2);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligend_cloud_v2");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v2");

	//对应关系可视化
	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
	boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
	crude_cor_est.setInputSource(source_fpfh);
	crude_cor_est.setInputTarget(target_fpfh);
	//crude_cor_est.determineCorrespondences(*cru_correspondences);
	crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
	cout << "crude size is " << cru_correspondences->size() << endl;
	view->addCorrespondences<pcl::PointXYZ>(source, target, *cru_correspondences,"correspondence", v1);
	view->initCameraParameters();
	while (!view->wasStopped())
	{
		view->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	system("pause");
	return 0;
}

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	//法向量
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(10);
	//est_normal.setSearchSurface();
	est_normal.compute(*point_normal);

	//fpfh估计
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_target_fpfh;
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(4);//指定4核计算

	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(10);
	est_fpfh.compute(*fpfh);

	return fpfh;

}