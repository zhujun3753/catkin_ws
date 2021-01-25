// 尝试用平面法向量的叉乘，但是失败了。。。。。。。。

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <Eigen/Core>

struct color
{
    double r;
    double g;
    double b;
    double size; // 点的大小

    // 使用了构造函数的参数初始化列表，可以只对部分成员变量初始化，这里有个好处是可以使用默认参数,这样结构体就可以赋予默认值
    color (double r =0,double g=0, double b=0,double size=1) : r (r),g (g),b (b),size (size){}
};
struct univect  //  单位向量 
{
  double x, y, z,w;
  univect(double x=1, double y=0, double z=0, double w=0):x(x),y(y),z(z),w(w){}
};
bool ascend (univect a,univect b) { return (a.x<b.x); }
struct seg_input
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input ;
  // pcl::PointCloud<pcl::Normal>::Ptr normal_input ;

  
};
struct seg_output
{
  bool exist_model;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model;
  pcl::ModelCoefficients::Ptr coefficients_model;
  pcl::PointIndices::Ptr indices_model; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remained ;
  pcl::PointCloud<pcl::Normal>::Ptr normal_remained ;
  
};

seg_output mdl_seg(const seg_input & parain)
{
  seg_output  resultout;
  pcl::PointIndices::Ptr indices_model (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_model (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remained (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normal_remained (new pcl::PointCloud<pcl::Normal>);
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; 
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::PCDWriter writer;

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (parain.cloud_input);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setSearchMethod (tree);
  ne.setInputCloud (parain.cloud_input);
  ne.setKSearch (25);
  ne.compute (*cloud_normals);


  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
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
  if(seg.getMethodType()==pcl::SACMODEL_CYLINDER)
    seg.setRadiusLimits (0.32, 0.35); // 0.32,0.35
  seg.setInputCloud (parain.cloud_input);
  seg.setInputNormals (cloud_normals);
  seg.segment (*indices_model, *coefficients_model);// Obtain the cylinder inliers and coefficients指向点的索引的指针，并不是指向点的指针
  
  extract.setInputCloud (parain.cloud_input);
  extract.setIndices (indices_model);
  extract.setNegative (false);
  extract.filter (*cloud_model);
  if (cloud_model->points.empty ()) 
  {
    std::cerr << "Can't find the model component." << std::endl;
    resultout.exist_model=false;
    return resultout;
  } 
  else
  {
    std::cerr << "model coefficients: " << *coefficients_model << std::endl;
	  std::cerr << "PointCloud representing the model component: " << cloud_model->size () << " data points." << std::endl;
    // writer.write ("cloud_model.pcd", *cloud_model, false);
    resultout.exist_model=true;
  }

  // 从输入点云中提取去除圆柱点云的数据
  extract.setNegative (true);
  extract.filter (*cloud_remained);
  // 从输入法向量中提取去除圆柱点云法向量的数据
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (indices_model);
  extract_normals.filter (*normal_remained);
  // std::cerr << "Test end!!!." << std::endl;

  // 下面这些等号是左边的指针指向右边指针所指向的地址，两个指针同时指向一段地址，因此对任意一指针的操作都会影响数据
  // 好在后面指针的作用域只在此函数，如果在循环中，那很有可能出问题！
  resultout.coefficients_model=coefficients_model;
  resultout.indices_model=indices_model;
  resultout.cloud_remained=cloud_remained;
  resultout.normal_remained=normal_remained;
  resultout.cloud_model=cloud_model;
  return resultout;
}
void extractEuclideanClusters (
    const pcl::PointCloud<pcl::PointXYZ> &cloud, const pcl::PointCloud<pcl::Normal> &normals, 
    float tolerance, const pcl::search::Search<pcl::PointXYZ>::Ptr &tree, 
    std::vector<pcl::PointIndices> &clusters, double eps_angle, 
    unsigned int min_pts_per_cluster = 1, 
    unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
{
  if (tree->getInputCloud ()->points.size () != cloud.points.size ())
  {
    PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
    return;
  }
  if (cloud.points.size () != normals.points.size ())
  {
    PCL_ERROR ("[pcl::extractEuclideanClusters] Number of points in the input point cloud (%lu) different than normals (%lu)!\n", cloud.points.size (), normals.points.size ());
    return;
  }

  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (cloud.points.size (), false);

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  // Process all points in the indices vector
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    if (processed[i])
      continue;

    std::vector<unsigned int> seed_queue;
    int sq_idx = 0;
    seed_queue.push_back (static_cast<int> (i));

    processed[i] = true;

    while (sq_idx < static_cast<int> (seed_queue.size ()))
    {
      // Search for sq_idx
      if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances))
      {
        sq_idx++;
        continue;
      }

      for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
      {
        if (processed[nn_indices[j]])                         // Has this point been processed before ?
          continue;

        //processed[nn_indices[j]] = true;
        // [-1;1]
        double dot_p = normals.points[i].normal[0] * normals.points[nn_indices[j]].normal[0] +
                        normals.points[i].normal[1] * normals.points[nn_indices[j]].normal[1] +
                        normals.points[i].normal[2] * normals.points[nn_indices[j]].normal[2];
        if (  nn_distances[j]<tolerance && fabs (acos (dot_p)) < eps_angle)   //  fabs (acos (dot_p)) < eps_angle
        {
          processed[nn_indices[j]] = true;
          seed_queue.push_back (nn_indices[j]);
        }
      }

      sq_idx++;
    }

    // If this queue is satisfactory, add to the clusters
    if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
    {
      pcl::PointIndices r;
      r.indices.resize (seed_queue.size ());
      for (size_t j = 0; j < seed_queue.size (); ++j)
        r.indices[j] = seed_queue[j];

      // These two lines should not be needed: (can anyone confirm?) -FF
      std::sort (r.indices.begin (), r.indices.end ());
      r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

      r.header = cloud.header;
      clusters.push_back (r);   // We could avoid a copy by working directly in the vector
    }
  }
}

double findCorners (
    const pcl::PointCloud<pcl::PointXYZ> &cloud, 
    double radius, const pcl::search::Search<pcl::PointXYZ>::Ptr &tree, 
    std::vector<pcl::PointIndices> &clusters)
{
  pcl::console::print_warn("The size of cloud: %d\n", cloud.points.size());
  if (tree->getInputCloud ()->points.size () != cloud.points.size ())
  {
    PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
    return 0;
  }
 // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (cloud.points.size (), false);
  std::vector<univect> unit_vectors;
  univect unit_vector;
  double max_angle=0;
  bool changed=false;

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  std::vector<unsigned int> seed_queue;
  // Process all points in the indices vector
  for (unsigned int i = 0; i < cloud.points.size (); ++i)
  {
    if (!tree->radiusSearch (i, radius, nn_indices, nn_distances)) continue;
    for (unsigned int j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
    {
      double nn_dist=sqrt(nn_distances[j]);
       if(nn_dist<0.95*radius&&nn_dist>0.2*radius) continue;  //  滤掉距离较近的点
      unit_vector.x = ( cloud.points[nn_indices[j]].x- cloud.points[i].x ) / nn_dist;
      unit_vector.y = ( cloud.points[nn_indices[j]].y- cloud.points[i].y ) / nn_dist;
      unit_vector.z = ( cloud.points[nn_indices[j]].z- cloud.points[i].z ) / nn_dist;
      double dist=sqrt(unit_vector.x*unit_vector.x+unit_vector.y*unit_vector.y+unit_vector.z*unit_vector.z);
      // if(i<1) pcl::console::print_warn("The norm of vector: %f, the distance: %f\n", dist,nn_dist);
      // if(i<1) pcl::console::print_warn("nn_indices[j]= %d, i= %d, nn_indices[0]=%d.\n", nn_indices[j],i, nn_indices[0]);
      for(size_t k=0;k<unit_vectors.size();k++)
      {
          double dot_p=unit_vector.x*unit_vectors[k].x+unit_vector.y*unit_vectors[k].y+unit_vector.z*unit_vectors[k].z;
          double angle= fabs (acos (dot_p)) ;
          max_angle=(max_angle<angle)?angle:max_angle;
          changed=true;
      }
      unit_vectors.push_back(unit_vector);
    }
    if(max_angle<M_PI*0.6&&changed) seed_queue.push_back(i);
    unit_vectors.clear();
    max_angle=0;
    changed=false;
  }
  // If this queue is satisfactory, add to the clusters
  // if(seed_queue.size () ==0) 
  // {
  //   pcl::console::print_warn("No corner points!!");
  //   // return max_angle;
  // }
  pcl::console::print_warn("Found %d corner points!!",seed_queue.size());
  pcl::PointIndices r;
  r.indices.resize (seed_queue.size ());
  for (size_t j = 0; j < seed_queue.size (); ++j)
    r.indices[j] = seed_queue[j];

  // These two lines should not be needed: (can anyone confirm?) -FF
  std::sort (r.indices.begin (), r.indices.end ());
  r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

  r.header = cloud.header;
  clusters.push_back (r);   // We could avoid a copy by working directly in the vector
  return max_angle;
}

int main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read (argv[1], *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*

  

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.1); // 2cm
  ec.setMinClusterSize (1000);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  // ec.extract (cluster_indices);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (25);
  ne.compute (*cloud_normals);

  double eps_angle=M_PI/15;
  float tolerance=0.2;
  std::vector<pcl::PointIndices> clusters;
  extractEuclideanClusters (*cloud, *cloud_normals, tolerance, tree, cluster_indices, eps_angle,1000,4000);


  pcl::visualization::PCLVisualizer viewer ("Cluster cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_filtered_color_handler (cloud, 255, 255, 255);
  // viewer.addPointCloud (cloud, cloud_filtered_color_handler, "original_cloud");
  viewer.addCoordinateSystem (1.0);
  int j = 0;
  std::cout << "Find " << cluster_indices.size () << "  clusters." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<univect> normal_vectors;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        cloud_cluster->push_back ((*cloud)[*pit]); //*
        cloud_clusters->push_back ((*cloud)[*pit]); //*
      }
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j ;//<< ".pcd";
    // pcl::PCDWriter writer;
    // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
    
    
    // PCL函数计算质心
    Eigen::Vector4f centroid_eigen;					// 质心
    pcl::compute3DCentroid(*cloud_cluster, centroid_eigen);	// 齐次坐标，（c0,c1,c2,1）
    pcl::PointXYZ centroid(centroid_eigen(0),centroid_eigen(1),centroid_eigen(2));
    
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moment;
    moment.setInputCloud(cloud_cluster);
    moment.compute();
    Eigen::Vector3f center, vx, vy, vz;
    float l1,l2,l3;
    moment.getEigenVectors(vx, vy, vz);
    moment.getEigenValues(l1,l2,l3);
    // std::cout<<"Major: "<<l1<<" middle: "<<l2<<" minor:  "<<l3<<std::endl;
    std::cout<<"rate: "<<l2/l1<<std::endl;
    int r,g,b,gap;
    gap=256*256*256/cluster_indices.size () ;
    // std::cout<<"gap"<<gap<<std::endl;
    // r=(j*gap)%256;
    // g=((j*gap)/256)%256;
    // b=(((j*gap)/256)/256)%256;
    r=(l2/l1>0.5)?(j*gap)%256:255;
    g=(l2/l1>0.5)?((j*gap)/256)%256:255;
    b=(l2/l1>0.5)?(((j*gap)/256)/256)%256:255;
    // r=255;
    // g=0;
    // b=0;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>cloud_cluster_color_handler (cloud_cluster, r, g, b);
    viewer.addPointCloud (cloud_cluster, cloud_cluster_color_handler, ss.str ());
    std::stringstream centd;
    centd<<"centroid"<<j;
    // viewer.addSphere (centroid,1.6, 1, 1, 1, centd.str());
    // 提取平面法线
    seg_input parain;
    seg_output resultout;
    parain.cloud_input=cloud_cluster;
    resultout=mdl_seg(parain);
    univect normal_vector;
    if(resultout.exist_model&&l2/l1>0.5)
    {
      normal_vector.x=resultout.coefficients_model->values[0];
      normal_vector.y=resultout.coefficients_model->values[1];
      normal_vector.z=resultout.coefficients_model->values[2];
      normal_vectors.push_back(normal_vector);
    }
    

  }
  cloud_clusters->width = cloud_clusters->size ();
  cloud_clusters->height = 1;
  cloud_clusters->is_dense = true;
  std::stringstream ss;
  // ss << "cloud_clusters.pcd" ;//<< j ;//<< ".pcd";
  // writer.write<pcl::PointXYZ> (ss.str (), *cloud_clusters, false); //*
  std::vector<pcl::PointIndices> cluster_indices2;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
  tree2->setInputCloud (cloud_clusters);
  double radius=0.5;
  // findCorners (*cloud_clusters,  radius, tree2, cluster_indices2);
  // std::stringstream sphere;
  // // pcl::console::print_warn("Found %d corner points!!",cluster_indices2[0].indices.size());
  // for(size_t i=0;i<cluster_indices2[0].indices.size();i++)
  // {
  //   sphere<<"sphere"<<i;
  //   viewer.addSphere (cloud_clusters->points[cluster_indices2[0].indices[i]],radius, 1, 1, 1, sphere.str());
  // }
  std::vector<univect> dir_vects;
  for(int i=0; i<normal_vectors.size()-1;i++)
  {
    for(int j=i+1; j<normal_vectors.size(); j++)
    {
      univect dir_vect;
      dir_vect.x=normal_vectors[i].y*normal_vectors[j].z-normal_vectors[i].z*normal_vectors[j].y;
      dir_vect.y=normal_vectors[i].z*normal_vectors[j].x-normal_vectors[i].x*normal_vectors[j].z;
      dir_vect.z=normal_vectors[i].x*normal_vectors[j].y-normal_vectors[i].y*normal_vectors[j].x;
      double dir_norm=sqrt(dir_vect.x*dir_vect.x+dir_vect.y*dir_vect.y+dir_vect.z*dir_vect.z);
      if(dir_vect.x<0) dir_norm=-dir_norm;
      dir_vect.x=dir_vect.x/dir_norm;
      dir_vect.y=dir_vect.y/dir_norm;
      dir_vect.z=dir_vect.z/dir_norm;
      dir_vect.w=fabs(dir_norm);
      if(dir_vect.w>0.2)
        dir_vects.push_back(dir_vect);   
    }
  }
  double aver_x=0,aver_y=0,aver_z=0;
  std::sort(dir_vects.begin(),dir_vects.end(),ascend);
  for(int i=0; i<dir_vects.size(); i++)
  {
    std::cout<<"dir_vector: "<<dir_vects[i].x<<" "<<dir_vects[i].y<<" "<<dir_vects[i].z<<" "<<dir_vects[i].w<<std::endl;
    aver_x+=dir_vects[i].x;
    aver_y+=dir_vects[i].y;
    aver_z+=dir_vects[i].z;
  }
  aver_x/=dir_vects.size();
  aver_y/=dir_vects.size();
  aver_z/=dir_vects.size();
  pcl::PointXYZ origin(0,0,0),extreme(aver_x,aver_y,aver_z);
  viewer.addLine(origin,extreme,"line");
  std::stringstream line_dots;
  for(int i =1;i<50;i++)
  {
    pcl::PointXYZ dot_line(aver_x*i,aver_y*i,aver_z*i);
    line_dots<<"line_dot"<<i;
    viewer.addSphere (dot_line,0.1, 1, 1, 1, line_dots.str());
  }
  
    
  while (!viewer.wasStopped ()) 
  { 
    viewer.spinOnce ();
  }

  return (0);
}