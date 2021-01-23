#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h> 
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

struct CloudStyle
{
    double r;
    double g;
    double b;
    double size; // 点的大小

    // 使用了构造函数的参数初始化列表，可以只对部分成员变量初始化，这里有个好处是可以使用默认参数,这样结构体就可以赋予默认值
    CloudStyle (double r =0,double g=0, double b=0,double size=0) : r (r),g (g),b (b),size (size){}
};

CloudStyle style_white (255.0, 255.0, 255.0, 4.0);
CloudStyle style_red (255.0, 0.0, 0.0, 3.0);
CloudStyle style_green (0.0, 255.0, 0.0, 5.0);
CloudStyle style_cyan (93.0, 200.0, 217.0, 4.0);
CloudStyle style_violet (255.0, 0.0, 255.0, 8.0);

std::string model_filename_;
std::string scene_filename_;

//Algorithm params 
bool show_keypoints_ (false);
bool use_hough_ (true);
float model_radius_search (0.02f);
float scene_radius_search (0.02f);
float ref_frame_rad_ (0.015f);
float descr_rad_ (0.05f);   // 0.02f
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);
int icp_max_iter_ (5);
float icp_corr_distance_ (0.005f);
float hv_resolution_ (0.005f);
float hv_occupancy_grid_resolution_ (0.01f);
float hv_clutter_reg_ (5.0f);
float hv_inlier_th_ (0.005f);
float hv_occlusion_th_ (0.01f);
float hv_rad_clutter_ (0.03f);
float hv_regularizer_ (3.0f);
float hv_rad_normals_ (0.05);
bool hv_detect_clutter_ (true);

/**
 * Prints out Help message
 * @param filename Runnable App Name
 */
void
showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*          Global Hypothese Verification Tutorial - Usage Guide          *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                          Show this help." << std::endl;
  std::cout << "     -k:                          Show keypoints." << std::endl;
  std::cout << "     --algorithm (Hough|GC):      Clustering algorithm used (default Hough)." << std::endl;
  std::cout << "     --model_ss val:              Model uniform sampling radius (default " << model_radius_search << ")" << std::endl;
  std::cout << "     --scene_ss val:              Scene uniform sampling radius (default " << scene_radius_search << ")" << std::endl;
  std::cout << "     --rf_rad val:                Reference frame radius (default " << ref_frame_rad_ << ")" << std::endl;
  std::cout << "     --descr_rad val:             Descriptor radius (default " << descr_rad_ << ")" << std::endl;
  std::cout << "     --cg_size val:               Cluster size (default " << cg_size_ << ")" << std::endl;
  std::cout << "     --cg_thresh val:             Clustering threshold (default " << cg_thresh_ << ")" << std::endl << std::endl;
  std::cout << "     --icp_max_iter val:          ICP max iterations number (default " << icp_max_iter_ << ")" << std::endl;
  std::cout << "     --icp_corr_distance val:     ICP correspondence distance (default " << icp_corr_distance_ << ")" << std::endl << std::endl;
  std::cout << "     --hv_clutter_reg val:        Clutter Regularizer (default " << hv_clutter_reg_ << ")" << std::endl;
  std::cout << "     --hv_inlier_th val:          Inlier threshold (default " << hv_inlier_th_ << ")" << std::endl;
  std::cout << "     --hv_occlusion_th val:       Occlusion threshold (default " << hv_occlusion_th_ << ")" << std::endl;
  std::cout << "     --hv_rad_clutter val:        Clutter radius (default " << hv_rad_clutter_ << ")" << std::endl;
  std::cout << "     --hv_regularizer val:        Regularizer value (default " << hv_regularizer_ << ")" << std::endl;
  std::cout << "     --hv_rad_normals val:        Normals radius (default " << hv_rad_normals_ << ")" << std::endl;
  std::cout << "     --hv_detect_clutter val:     TRUE if clutter detect enabled (default " << hv_detect_clutter_ << ")" << std::endl << std::endl;
}

/**
 * Parses Command Line Arguments (Argc,Argv)
 * @param argc
 * @param argv
 */
void parseCommandLine (int argc,char *argv[])
{
	//Show help
	//   bool pcl::console::find_switch(int argc, char **argv, const char *argument_name)
	// Finds whether the argument with name "argument_name" is in the argument list "argv".
	if (pcl::console::find_switch (argc, argv, "-h")) 
	{
		showHelp (argv[0]);
		exit (0);
	}
	//Model & scene filenames
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	if (filenames.size () != 2)
	{
		std::cout << "Filenames missing.\n";
		showHelp (argv[0]);
		exit (-1);
	}
	model_filename_ = argv[filenames[0]];
	scene_filename_ = argv[filenames[1]]; // 包含.pcd
	// std::cerr<<"model filename:"<<model_filename_<<std::endl;
	//Program behavior
	if (pcl::console::find_switch (argc, argv, "-k")) show_keypoints_ = true;
	std::string used_algorithm;
	if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
	{
		if (used_algorithm.compare ("Hough") == 0) use_hough_ = true;
		else if (used_algorithm.compare ("GC") == 0) use_hough_ = false;
		else
		{
			std::cout << "Wrong algorithm name.\n";
			showHelp (argv[0]);
			exit (-1);
		}
	}

	//General parameters
	pcl::console::parse_argument (argc, argv, "--model_ss", model_radius_search);
	pcl::console::parse_argument (argc, argv, "--scene_ss", scene_radius_search);
	pcl::console::parse_argument (argc, argv, "--rf_rad", ref_frame_rad_);
	pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad_);
	pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
	pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
	pcl::console::parse_argument (argc, argv, "--icp_max_iter", icp_max_iter_);
	pcl::console::parse_argument (argc, argv, "--icp_corr_distance", icp_corr_distance_);
	pcl::console::parse_argument (argc, argv, "--hv_clutter_reg", hv_clutter_reg_);
	pcl::console::parse_argument (argc, argv, "--hv_inlier_th", hv_inlier_th_);
	pcl::console::parse_argument (argc, argv, "--hv_occlusion_th", hv_occlusion_th_);
	pcl::console::parse_argument (argc, argv, "--hv_rad_clutter", hv_rad_clutter_);
	pcl::console::parse_argument (argc, argv, "--hv_regularizer", hv_regularizer_);
	pcl::console::parse_argument (argc, argv, "--hv_rad_normals", hv_rad_normals_);
	pcl::console::parse_argument (argc, argv, "--hv_detect_clutter", hv_detect_clutter_);
}
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


int main (int argc, char *argv[])
{
  parseCommandLine (argc, argv); // 这个函数还是挺值得借鉴的

  pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr model_downsampled (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_downsampled (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

  /**
   * 加载pcd文件
   */
  if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }
  if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
  {
    std::cout << "Error loading scene cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }

  /**
   * 计算法向量
   */
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;	//	使用openOMP标准计算法向量
  norm_est.setKSearch (10);
  norm_est.setInputCloud (model); 	//	输入只是传递指针
  norm_est.compute (*model_normals); //模型法向量

  norm_est.setInputCloud (scene);
  norm_est.compute (*scene_normals);	//	场景法向量

  /**
   *  下采样点云
   */
  pcl::UniformSampling<PointType> uniform_sampling;	//	均匀采样
  uniform_sampling.setInputCloud (model);
  uniform_sampling.setRadiusSearch (model_radius_search);
  uniform_sampling.filter (*model_downsampled);
  // model_downsampled=model;
  std::cout << "Model total points: " << model->size () << "; Downsampled points: " << model_downsampled->size () << std::endl;

  uniform_sampling.setInputCloud (scene);
  uniform_sampling.setRadiusSearch (scene_radius_search);
  uniform_sampling.filter (*scene_downsampled);
  // scene_downsampled=scene;
  std::cout << "Scene total points: " << scene->size () << "; Downsampled points: " << scene_downsampled->size () << std::endl;

  /**
   *  通过下采样得到的点作为关键点，计算其方位直方图特征
   * 方位直方图特征是一种基于局部特征的描述子，在特征点处建立局部坐标系，将邻域点的空间位置信息和几何特征统计信息结合起来描述特征点
   */
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);

  descr_est.setInputCloud (model_downsampled);
  descr_est.setInputNormals (model_normals);
  descr_est.setSearchSurface (model);
  descr_est.compute (*model_descriptors);

  descr_est.setInputCloud (scene_downsampled);
  descr_est.setInputNormals (scene_normals);
  descr_est.setSearchSurface (scene);
  descr_est.compute (*scene_descriptors);

  /**
   * 用KdTree查找模型场景对应关系
   */
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);
  std::vector<int> model_good_keypoints_indices;
  std::vector<int> scene_good_keypoints_indices;

  for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!std::isfinite (scene_descriptors->at (i).descriptor[0]))  //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    // 设定k值，以寻找待搜索点的k个最近临点, 这里k=1
    // k个最近临点的索引
    // k个最近临点到搜索点距离的平方
    // std::cerr<<found_neighs<<"个"<<std::endl;
    if (found_neighs == 1 && neigh_sqr_dists[0] < 0.5)  //  0.25
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
      model_good_keypoints_indices.push_back (corr.index_query);
      scene_good_keypoints_indices.push_back (corr.index_match);
    }
  }
  pcl::PointCloud<PointType>::Ptr model_good_kpt (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_good_kpt (new pcl::PointCloud<PointType> ());
  pcl::copyPointCloud (*model_downsampled, model_good_keypoints_indices, *model_good_kpt);
  pcl::copyPointCloud (*scene_downsampled, scene_good_keypoints_indices, *scene_good_kpt);

  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
  std::cout << "Model total points: " << model->size () << "; Downsampled points: " << model_downsampled->size () << std::endl;
  std::cout << "Scene total points: " << scene->size () << "; Downsampled points: " << scene_downsampled->size () << std::endl;
  visualize_corrs(model_downsampled,scene_downsampled,model,scene,model_scene_corrs);
  /**
   *  Clustering
   */
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector < pcl::Correspondences > clustered_corrs;

  if (!use_hough_)
  {
    pcl::PointCloud<RFType>::Ptr model_ref_frame (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_ref_frame (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> ref_frame_esti;
    // 局部参考帧估计的边界感知可重复方向算法
    ref_frame_esti.setFindHoles (true);
    ref_frame_esti.setRadiusSearch (ref_frame_rad_);

    ref_frame_esti.setInputCloud (model_downsampled);
    ref_frame_esti.setInputNormals (model_normals);
    ref_frame_esti.setSearchSurface (model);
    ref_frame_esti.compute (*model_ref_frame);

    ref_frame_esti.setInputCloud (scene_downsampled);
    ref_frame_esti.setInputNormals (scene_normals);
    ref_frame_esti.setSearchSurface (scene);
    ref_frame_esti.compute (*scene_ref_frame);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_); //  Sets the size of each bin into the Hough space
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_downsampled);
    clusterer.setInputRf (model_ref_frame);
    clusterer.setSceneCloud (scene_downsampled);
    clusterer.setSceneRf (scene_ref_frame);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    clusterer.recognize (rototranslations, clustered_corrs);
  }
  else
  {
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (0.01f);  //  float cg_size_ (0.01f);
    gc_clusterer.setInputCloud (model_downsampled);
    gc_clusterer.setSceneCloud (scene_downsampled);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }

  /**
   * Stop if no instances
   */
  if (rototranslations.size () <= 0)
  {
    std::cout << "*** No instances found! ***" << std::endl;
    return (0);
  }
  else
  {
    std::cout << "Recognized Instances: " << rototranslations.size () << std::endl << std::endl;
  }

  /**
   * Generates clouds for each instances found 
   */
  std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;

  for (std::size_t i = 0; i < rototranslations.size (); ++i)
  {
    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
    instances.push_back (rotated_model);
  }

  /**
   * ICP 迭代最近点 Iterative Closest Point  点集对点集配准方法
   */
  std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
  if (true)
  {
    std::cout << "--- ICP ---------" << std::endl;

    for (std::size_t i = 0; i < rototranslations.size (); ++i)
    {
      pcl::IterativeClosestPoint<PointType, PointType> icp;
      icp.setMaximumIterations (icp_max_iter_);
      icp.setMaxCorrespondenceDistance (icp_corr_distance_);
      icp.setInputTarget (scene);
      icp.setInputSource (instances[i]);
      pcl::PointCloud<PointType>::Ptr registered (new pcl::PointCloud<PointType>);
      icp.align (*registered);
      std::cout << "size of :" <<registered->size()<< std::endl;
      registered_instances.push_back (registered);
      std::cout << "Instance " << i << " ";
      if (icp.hasConverged ())
      {
        std::cout << "Aligned!" << std::endl;
      }
      else
      {
        std::cout << "Not Aligned!" << std::endl;
      }
    }

    std::cout << "-----------------" << std::endl << std::endl;
  }

  /**
   * Hypothesis Verification
   */
  // std::cout << "--- Hypotheses Verification ---" << std::endl;
  // std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

  // pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;

  // GoHv.setSceneCloud (scene);  // Scene Cloud
  // GoHv.addModels (registered_instances, true);  //Models to verify
  // GoHv.setResolution (hv_resolution_);
  // GoHv.setResolution (hv_occupancy_grid_resolution_);
  // GoHv.setInlierThreshold (hv_inlier_th_);
  // GoHv.setOcclusionThreshold (hv_occlusion_th_);
  // GoHv.setRegularizer (hv_regularizer_);
  // GoHv.setRadiusClutter (hv_rad_clutter_);
  // GoHv.setClutterRegularizer (hv_clutter_reg_);
  // GoHv.setDetectClutter (hv_detect_clutter_);
  // GoHv.setRadiusNormals (hv_rad_normals_);

  // GoHv.verify ();
  // GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses

  // for (int i = 0; i < hypotheses_mask.size (); i++)
  // {
  //   if (hypotheses_mask[i])
  //   {
  //     std::cout << "Instance " << i << " is GOOD! <---" << std::endl;
  //   }
  //   else
  //   {
  //     std::cout << "Instance " << i << " is bad!" << std::endl;
  //   }
  // }
  std::cout << "-------------------------------" << std::endl;

  /**
   *  Visualization
   */
  pcl::visualization::PCLVisualizer viewer ("Hypotheses Verification");
  viewer.addPointCloud (scene, "scene_cloud");

  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_downsampled (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_model_good_kpt (new pcl::PointCloud<PointType> ());

  pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
  pcl::transformPointCloud (*model_downsampled, *off_scene_model_downsampled, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
  pcl::transformPointCloud (*model_good_kpt, *off_model_good_kpt, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));

  if (show_keypoints_)
  {
    CloudStyle modelStyle = style_white;
    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, modelStyle.r, modelStyle.g, modelStyle.b);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, modelStyle.size, "off_scene_model");
  }

  if (show_keypoints_)
  {
    CloudStyle goodKeypointStyle = style_violet;
    pcl::visualization::PointCloudColorHandlerCustom<PointType> model_good_keypoints_color_handler (off_model_good_kpt, goodKeypointStyle.r, goodKeypointStyle.g,
                                                                                                    goodKeypointStyle.b);
    viewer.addPointCloud (off_model_good_kpt, model_good_keypoints_color_handler, "model_good_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "model_good_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_good_keypoints_color_handler (scene_good_kpt, goodKeypointStyle.r, goodKeypointStyle.g,
                                                                                                    goodKeypointStyle.b);
    viewer.addPointCloud (scene_good_kpt, scene_good_keypoints_color_handler, "scene_good_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "scene_good_keypoints");
  }

  for (std::size_t i = 0; i < instances.size (); ++i)
  {
    std::stringstream ss_instance;
    ss_instance << "instance_" << i;

    CloudStyle clusterStyle = style_red;
    pcl::visualization::PointCloudColorHandlerCustom<PointType> instance_color_handler (instances[i], clusterStyle.r, clusterStyle.g, clusterStyle.b);
    viewer.addPointCloud (instances[i], instance_color_handler, ss_instance.str ());
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, clusterStyle.size, ss_instance.str ());

    // CloudStyle registeredStyles = hypotheses_mask[i] ? style_green : style_cyan;
    CloudStyle registeredStyles =  style_green ;
    ss_instance << "_registered" << std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<PointType> registered_instance_color_handler (registered_instances[i], registeredStyles.r,
                                                                                                   registeredStyles.g, registeredStyles.b);
    viewer.addPointCloud (registered_instances[i], registered_instance_color_handler, ss_instance.str ());
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, registeredStyles.size, ss_instance.str ());
  }

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return (0);
}