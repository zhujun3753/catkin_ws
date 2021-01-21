
/* 目标:对输入的model,scene计算.匹配关系,并估计位姿
 * 下采样->计算关键点(iss)->计算特征(fpfh)
 * 可视化点云以及关键点,并可视化其特征直方图
 */
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>
#include <iostream>
#include <pcl/keypoints/iss_3d.h>
#include <cstdlib>
#include <pcl/visualization/pcl_plotter.h>// 直方图的可视化 方法2
#include <pcl/registration/sample_consensus_prerejective.h>   // pose estimate
#include <pcl/keypoints/sift_keypoint.h>   // shift关键点相关
#include <pcl/pcl_macros.h>
#include <pcl/keypoints/harris_3d.h>


using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;

typedef pcl::SHOT352 DescriptorType;
float descr_rad_ (3.0f);  // shot描述子的搜索半径


// sift相关
namespace pcl
{
    template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
        inline float
        operator () (const PointXYZ &p) const
        {
            return p.z;
        }
    };
}


double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (! pcl_isfinite((*cloud)[i].x))
            continue;

        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2)
        {
            resolution += sqrt(squaredDistances[1]);
            ++numberOfPoints;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;

    return resolution;
}

void compute_harris(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_result)
{
    pcl::HarrisKeypoint3D <pcl::PointXYZ, pcl::PointXYZI> detector;
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
    detector.setNonMaxSupression (true);
    detector.setInputCloud (cloud);
    detector.setRadius(0.1f);
    detector.setRadiusSearch(0.1f);
    pcl::StopWatch watch;
    detector.compute (*keypoints);
    pcl::console::print_highlight ("Detected %zd points in %lfs\n", keypoints->size (), watch.getTimeSeconds ());
    pcl::PointIndicesConstPtr keypoints_indices = detector.getKeypointsIndices ();
    if (!keypoints_indices->indices.empty ())
    {
        pcl::io::savePCDFile ("keypoints.pcd", *cloud, keypoints_indices->indices, true);
        pcl::copyPointCloud(*cloud,keypoints_indices->indices,*keypoints_result);
        pcl::console::print_info ("Saved keypoints to keypoints.pcd\n");
    }
    else
        pcl::console::print_warn ("Keypoints indices are empty!\n");
}
// 计算shift特征
void
compute_shift(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints)
{
    clock_t start = clock();

    const float min_scale = 1;             //设置尺度空间中最小尺度的标准偏差
    const int n_octaves = 3;               //设置高斯金字塔组（octave）的数目
    const int n_scales_per_octave =1;     //设置每组（octave）计算的尺度
    const float min_contrast = 0.02;          //设置限制关键点检测的阈值

    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;//创建sift关键点检测对象
    pcl::PointCloud<pcl::PointWithScale> result;
    sift.setInputCloud(cloud);//设置输入点云
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    sift.setSearchMethod(tree);//创建一个空的kd树对象tree，并把它传递给sift检测对象
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);//指定搜索关键点的尺度范围
    sift.setMinimumContrast(min_contrast);//设置限制关键点检测的阈值
    sift.compute(result);//执行sift关键点检测，保存结果在result

    copyPointCloud(result, *keypoints);//将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZ的数据
    clock_t end = clock();

    cout << "sift关键点提取时间：" << (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout << "sift关键点数量" << keypoints->size() << endl;
}

// 计算iss特征
void
compute_iss(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints)
{
    clock_t start = clock();
    // 计算关键点
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_det;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    // 计算分辨率
    double model_resolution = computeCloudResolution(cloud);
    cout<<"resolution: "<<model_resolution<<endl;

    //iss公共参数设置
    iss_det.setMinNeighbors(10);
    iss_det.setThreshold21(0.975);
    iss_det.setThreshold32(0.975);
    iss_det.setNumberOfThreads(4);

    // 计算model关键点
    iss_det.setInputCloud(cloud);
    iss_det.setSearchMethod(tree);
    iss_det.setSalientRadius(6*model_resolution);  // 0.5
    iss_det.setNonMaxRadius(4*model_resolution);
    iss_det.compute(*keypoints);


    clock_t end = clock();
    cout << "iss关键点提取时间：" << (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout << "iss关键点数量" << keypoints->size() << endl;

}


// 估计法线
// input:cloud
// output:normals
void
est_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::Normal>::Ptr normals)
{

    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setNumberOfThreads(4);   //手动设置线程数
    norm_est.setKSearch (10);         //设置k邻域搜索阈值为10个点
    norm_est.setInputCloud (cloud_in);   //设置输入模型点云
    norm_est.compute (*normals);//计算点云法线

}

// 计算fpfh特征
// input: keypoints ,cloud , normals
// output: FPFH descriptors
void
compute_fpfh(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals,pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors)
{
    clock_t start = clock();
    // FPFH estimation object.
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ> );
    fpfh.setInputCloud(keypoints);  // 计算keypoints处的特征
    fpfh.setInputNormals(normals);   // cloud的法线
    fpfh.setSearchSurface(cloud); // 计算的平面是cloud 而不是keypoints
    fpfh.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    fpfh.setKSearch(10);
    fpfh.compute(*descriptors);
    clock_t end = clock();
    cout<<"Time fpfh: "<< (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout<<"Get fpfh: "<<descriptors->points.size()<<endl;

}

// 计算pfh特征
// input: keypoints ,cloud , normals
// output: FPFH descriptors
void
compute_pfh(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
             pcl::PointCloud<pcl::Normal>::Ptr normals,pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors)
{
    clock_t start = clock();
    // FPFH estimation object.
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ> );
    pfh.setInputCloud(keypoints);  // 计算keypoints处的特征
    pfh.setInputNormals(normals);   // cloud的法线
    pfh.setSearchSurface(cloud); // 计算的平面是cloud 而不是keypoints
    pfh.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    pfh.setRadiusSearch(0.5);
    pfh.compute(*descriptors);
    clock_t end = clock();
    cout<<"Time pfh: "<< (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout<<"Get pfh: "<<descriptors->points.size()<<endl;

}

// 计算SHOT特征
// input:model_keypoint/scene_keypoint,  model/scene,  model_normals/scene_noemals
// outut:model_shot/scene_shot
void
compute_shot(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::PointCloud<pcl::Normal>::Ptr normals,pcl::PointCloud<pcl::SHOT352>::Ptr descriptors)
{
    clock_t start=clock();

    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setNumberOfThreads(4);   // 设置线程数 默认为0
    descr_est.setRadiusSearch (descr_rad_);     //设置搜索半径

    descr_est.setInputCloud (keypoints);  //模型点云的关键点
    descr_est.setInputNormals (normals);  //模型点云的法线
    descr_est.setSearchSurface (cloud);         //模型点云
    descr_est.compute(*descriptors);   // 计算,保存

    clock_t end=clock();
    cout<<"Time SHOT: "<< (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout<<"Get SHOT: "<<descriptors->points.size()<<endl;
}




// fpfh match
// input: modelDescriptors,sceneDescriptors
// output: pcl::CorrespondencesPtr
void
find_match(pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_descriptors,pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_descriptors,pcl::CorrespondencesPtr model_scene_corrs)
{
    clock_t start = clock();

    pcl::KdTreeFLANN<pcl::FPFHSignature33> matching;
    matching.setInputCloud(model_descriptors);

    for (size_t i = 0; i < scene_descriptors->size(); ++i)
    {
        std::vector<int> neighbors(1);
        std::vector<float> squaredDistances(1);
        // Ignore NaNs.
        if (pcl_isfinite(scene_descriptors->at(i).histogram[0]   ))
        {
            // Find the nearest neighbor (in descriptor space)...
            int neighborCount = matching.nearestKSearch(scene_descriptors->at(i), 1, neighbors, squaredDistances);
            // ...and add a new correspondence if the distance is less than a threshold
            // (SHOT distances are between 0 and 1, other descriptors use different metrics).
            if (neighborCount == 1 && squaredDistances[0] < 0.1f)
            {
                pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
                model_scene_corrs->push_back(correspondence);
                cout<<"( "<<correspondence.index_query<<","<<correspondence.index_match<<" )"<<endl;
            }
        }
    }

    std::cout << "Found " << model_scene_corrs->size() << " FPFH correspondences." << std::endl;
    clock_t end = clock();
    cout<<"Time match: "<< (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout<<"-----------------------------"<<endl;
}


// shot match
// input: modelDescriptors,sceneDescriptors
// output: pcl::CorrespondencesPtr
void
find_match_shot(pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors,pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors,
        pcl::CorrespondencesPtr model_scene_corrs)
{
    clock_t start = clock();

    pcl::KdTreeFLANN<pcl::SHOT352> matching;
    matching.setInputCloud(model_descriptors);

    for (size_t i = 0; i < scene_descriptors->size(); ++i)
    {
        std::vector<int> neighbors(1);
        std::vector<float> squaredDistances(1);
        // Ignore NaNs.
        if (pcl_isfinite(scene_descriptors->at(i).descriptor[0]   ))
        {
            // Find the nearest neighbor (in descriptor space)...
            int neighborCount = matching.nearestKSearch(scene_descriptors->at(i), 1, neighbors, squaredDistances);
            // ...and add a new correspondence if the distance is less than a threshold
            // (SHOT distances are between 0 and 1, other descriptors use different metrics).
            if (neighborCount == 1 && squaredDistances[0] < 0.05f)
            {
                pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
                model_scene_corrs->push_back(correspondence);
//                cout<<"( "<<correspondence.index_query<<","<<correspondence.index_match<<" )"<<endl;   // 打印对应点对
            }
        }
    }

    std::cout << "Found " << model_scene_corrs->size() << " SHOT correspondences." << std::endl;
    clock_t end = clock();
    cout<<"Time match: "<< (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout<<"-----------------------------"<<endl;
}


// pfh match
// input: modelDescriptors,sceneDescriptors
// output: pcl::CorrespondencesPtr
void
find_match_pfh(pcl::PointCloud<pcl::PFHSignature125>::Ptr model_descriptors,pcl::PointCloud<pcl::PFHSignature125>::Ptr scene_descriptors,
                pcl::CorrespondencesPtr model_scene_corrs)
{
    clock_t start = clock();

    pcl::KdTreeFLANN<pcl::PFHSignature125> matching;
    matching.setInputCloud(model_descriptors);

    for (size_t i = 0; i < scene_descriptors->size(); ++i)
    {
        std::vector<int> neighbors(1);
        std::vector<float> squaredDistances(1);
        // Ignore NaNs.
        if (pcl_isfinite(scene_descriptors->at(i).histogram[0]   ))
        {
            // Find the nearest neighbor (in descriptor space)...
            int neighborCount = matching.nearestKSearch(scene_descriptors->at(i), 1, neighbors, squaredDistances);
            // ...and add a new correspondence if the distance is less than a threshold
            // (SHOT distances are between 0 and 1, other descriptors use different metrics).
            if (neighborCount == 1 && squaredDistances[0] < 0.1f)
            {
                pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
                model_scene_corrs->push_back(correspondence);
//                cout<<"( "<<correspondence.index_query<<","<<correspondence.index_match<<" )"<<endl;   // 打印对应点对
            }
        }
    }

    std::cout << "Found " << model_scene_corrs->size() << " PFH  correspondences." << std::endl;
    clock_t end = clock();
    cout<<"Time match: "<< (double)(end - start) / CLOCKS_PER_SEC <<endl;
    cout<<"-----------------------------"<<endl;
}

// 位姿估计
// input: model,scene,model_descriptors,scene_descriptors
// output: R,t
void
estimationPose(pcl::PointCloud<pcl::PointXYZ>::Ptr model,pcl::PointCloud<pcl::PointXYZ>::Ptr scene,
         pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_descriptors,pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_descriptors,
         pcl::PointCloud<pcl::PointXYZ>::Ptr alignedModel)
{
    // Object for pose estimation.
    pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> pose;
    pose.setInputSource(model);
    pose.setInputTarget(scene);
    pose.setSourceFeatures(model_descriptors);
    pose.setTargetFeatures(scene_descriptors);
    // Instead of matching a descriptor with its nearest neighbor, choose randomly between
    // the N closest ones, making it more robust to outliers, but increasing time.
    pose.setCorrespondenceRandomness(5);   // 匹配最近的5个描述子
    // Set the fraction (0-1) of inlier points required for accepting a transformation.
    // At least this number of points will need to be aligned to accept a pose.
    pose.setInlierFraction(0.01f);    //内点的数量
    // Set the number of samples to use during each iteration (minimum for 6 DoF is 3).
    pose.setNumberOfSamples(3);     // 估计6DOF需要3个点
    // Set the similarity threshold (0-1 between edge lengths of the polygons. The
    // closer to 1, the more strict the rejector will be, probably discarding acceptable poses.
    pose.setSimilarityThreshold(0.2f);
    // Set the maximum distance threshold between two correspondent points in source and target.
    // If the distance is larger, the points will be ignored in the alignment process.
    pose.setMaxCorrespondenceDistance(1.0f);  // 允许的最大距离

    pose.setMaximumIterations(50000);   // 迭代次数

    pose.align(*alignedModel);

    if (pose.hasConverged())
    {
        Eigen::Matrix4f transformation = pose.getFinalTransformation();
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
}

//点云可视化
// 显示model+scene以及他们的keypoints
void
visualize_pcd(PointCloud::Ptr model, PointCloud::Ptr model_keypoints,PointCloud::Ptr scene, PointCloud::Ptr scene_keypoints)
{
    pcl::visualization::PCLVisualizer viewer("registration Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_color(model, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_color(scene, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_keypoint_color(model_keypoints, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_keypoint_color(scene_keypoints, 0, 0, 255);
    viewer.setBackgroundColor(0, 0, 0);
    // viewer.addPointCloud(model, model_color, "model");
    viewer.addPointCloud(scene, scene_color, "scene");
    viewer.addPointCloud(model_keypoints, model_keypoint_color, "model_keypoints");
    viewer.addPointCloud(scene_keypoints, scene_keypoint_color, "scene_keypoints");
    viewer.addPointCloud(model, model_color, "model");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "model_keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "scene_keypoints");


    while(!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

// 可视化对应关系
//input: model_keypoints, scene_keypoints, model_scene_corrs
void
visualize_corrs(pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints,pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints,
        pcl::PointCloud<pcl::PointXYZ>::Ptr model,pcl::PointCloud<pcl::PointXYZ>::Ptr scene,
        pcl::CorrespondencesPtr model_scene_corrs)
{
    // 添加关键点
    pcl::visualization::PCLVisualizer viewer("corrs Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_color(model, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_color(scene, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_keypoint_color(model_keypoints, 0, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_keypoint_color(scene_keypoints, 0, 0, 255);
   viewer.addPointCloud(model, model_color, "model");
   viewer.addPointCloud(scene, scene_color, "scene");
    viewer.addPointCloud(model_keypoints, model_keypoint_color, "model_keypoints");
    viewer.addPointCloud(scene_keypoints, scene_keypoint_color, "scene_keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "model_keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "scene_keypoints");
    
	// 可视化对应关系
    viewer.addCorrespondences<pcl::PointXYZ>(model_keypoints,scene_keypoints,*model_scene_corrs);
    viewer.initCameraParameters();
//    //添加对应关系
   int i=1;
//    std::cerr<<"reach here!"<<std::endl;
//    std::cerr<<model_scene_corrs->size()<<std::endl;
   for(auto iter=model_scene_corrs->begin();iter!=model_scene_corrs->end();++iter)
   {
       std::stringstream ss_line;
       ss_line << "correspondence_line" << i ;
    //    std::cerr<<ss_line.str()<<std::endl;
       i++;
       PointType& model_point = model_keypoints->at (iter->index_query);  // 从corrs中获取对应点
       PointType& scene_point = scene_keypoints->at (iter->index_match);
       viewer.addLine<PointType, PointType> (model_point, scene_point, 255, 0, 0, ss_line.str ());
       viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, ss_line.str ());   // 设置线宽
   }
    // std::cerr<<"reach here2!"<<std::endl;
    // 显示
    while(!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

}

// 可视化直方图
void
visualize_Histogram( pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_feature, pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_feature)
{
    pcl::visualization::PCLPlotter plotter;
    plotter.addFeatureHistogram(*model_feature, 33); //设置的很坐标长度，该值越大，则显示的越细致
    plotter.addFeatureHistogram(*scene_feature, 33); //设置的很坐标长度，该值越大，则显示的越细致
    plotter.plot();
}

int main(int argc, char** argv)
{
    PointCloud::Ptr aligned_model(new PointCloud);    // 变换之后的点云
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());  // model-scene的匹配关系

    pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors_shot(new pcl::PointCloud<pcl::SHOT352>());  // shot特征
    pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors_shot(new pcl::PointCloud<pcl::SHOT352>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());  // fpfh特征
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::PFHSignature125>::Ptr model_descriptors_pfh(new pcl::PointCloud<pcl::PFHSignature125>());  // pfh特征
    pcl::PointCloud<pcl::PFHSignature125>::Ptr scene_descriptors_pfh(new pcl::PointCloud<pcl::PFHSignature125>());

    PointCloud::Ptr model_keypoint(new PointCloud);                 // iss关键点
    PointCloud::Ptr scene_keypoint(new PointCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints_shift(new pcl::PointCloud<pcl::PointXYZ>);  // shift关键点
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints_shift(new pcl::PointCloud<pcl::PointXYZ>);

    PointCloud::Ptr cloud_src_model(new PointCloud);    //model点云  读入
    PointCloud::Ptr cloud_src_scene(new PointCloud);    //scene点云
    PointCloud::Ptr model(new PointCloud);    //model点云  降采样后,一切计算是以这个为基础的
    PointCloud::Ptr scene(new PointCloud);    //scene点云
    pcl::PointCloud<pcl::Normal>::Ptr model_normals(new pcl::PointCloud<pcl::Normal>);  // 法向量
    pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>);


    //加载点云
    if (argc<3)
    {
        cout<<".exe model.pcd scene.pcd"<<endl;
        return -1;
    }

    pcl::io::loadPCDFile(argv[1], *cloud_src_model);
    pcl::io::loadPCDFile(argv[2], *cloud_src_scene);
    cout << "/" << endl;
    cout << "原始model点云数量：" << cloud_src_model->size() << endl;
    cout << "原始scene点云数量：" << cloud_src_scene->size() << endl;


    model=cloud_src_model;
    scene=cloud_src_scene;

//    //均匀采样对象
//    pcl::VoxelGrid<pcl::PointXYZ> filter;
//    filter.setLeafSize(2,2,2);
//    filter.setInputCloud(cloud_src_model);
//    filter.filter(*model);
//    cout << "降采样后点云数量：" << model->size() << endl;
//
//    filter.setInputCloud(cloud_src_scene);
//    filter.filter(*scene);
//    cout << "降采样后点云数量：" << scene->size() << endl;

    // 计算降采样后的点云的法线
    est_normals(model,model_normals);
    est_normals(scene,scene_normals);

    // 计算关键点
    compute_iss(model,model_keypoint);
    compute_iss(scene,scene_keypoint);
    // compute_harris(model,model_keypoint);
    // compute_harris(scene,scene_keypoint);
    std::cerr<<"model 关键点数量 "<<model_keypoint->size()
        <<"scene 关键点数量 "<<scene_keypoint->size()<<std::endl;
    if(model_keypoint->size()==0||scene_keypoint->size()==0) return 0;

    // 根据关键点计算特征
    compute_fpfh(model_keypoint,model,model_normals,model_descriptors);   // fpfh特征
    compute_fpfh(scene_keypoint,scene,scene_normals,scene_descriptors);
    compute_shot(model_keypoint,model,model_normals,model_descriptors_shot);   // shot特征
    compute_shot(scene_keypoint,scene,scene_normals,scene_descriptors_shot);
    find_match(model_descriptors,scene_descriptors,model_scene_corrs);        // fpfh
    find_match_shot(model_descriptors_shot,scene_descriptors_shot,model_scene_corrs);           // shot

    compute_shift(model,model_keypoints_shift);
    compute_shift(scene,scene_keypoints_shift);
    if(model_keypoints_shift->size()>0&&scene_keypoints_shift->size()>0)
    {
        compute_pfh(model_keypoints_shift,model,model_normals,model_descriptors_pfh);   // pfh
        compute_pfh(scene_keypoints_shift,scene,scene_normals,scene_descriptors_pfh);
        if(model_descriptors_pfh->size()>0&&scene_descriptors_pfh->size()>0)
            find_match_pfh(model_descriptors_pfh,scene_descriptors_pfh,model_scene_corrs); 
    }
    else
    {
        std::cerr<<"No SHIFT keypoints !!"<<std::endl;
    }
    
    estimationPose(model_keypoint,scene_keypoint,model_descriptors,scene_descriptors,aligned_model);

    visualize_corrs(model_keypoint,scene_keypoint,model,scene,model_scene_corrs);  // 可视化对应关系
    visualize_pcd(model,model_keypoint,scene, scene_keypoint);  // 降采样后的点云 , 特征点
    visualize_Histogram(model_descriptors,scene_descriptors);   // 可视化fpfh直方图

    return 0;
}


