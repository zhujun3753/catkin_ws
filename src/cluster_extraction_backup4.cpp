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
#include <pcl/common/transforms.h>

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
  double x, y, z;
};


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
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read (argv[1], *cloud_source);
  std::cout << "PointCloud before filtering has: " << cloud_source->size () << " data points." << std::endl; //*

  // 坐标变换
  // float theta = M_PI/4; // The angle of rotation in radians
  // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  // transform.translation() << 10, 10.0, 0.0;
  // transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  // printf ("\nUsing an Affine3f\n");
  // std::cout << transform.matrix() << std::endl;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // pcl::transformPointCloud (*cloud_source, *transformed_cloud, transform);
  // transform.translation() << 0, 0.0, 0.0;
  // transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
  // pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, transform);
  // cloud=transformed_cloud;
  cloud=cloud_source;

  pcl::visualization::PCLVisualizer viewer ("Cluster cloud");
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_source_color_handler (cloud, 0, 255, 0);
  // viewer.addPointCloud (cloud_source, cloud_source_color_handler, "original_cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_trans_color_handler (cloud, 255, 255, 255);
   viewer.addPointCloud (cloud, cloud_trans_color_handler, "trans_cloud");

  viewer.addCoordinateSystem (10.0);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (25);
  ne.compute (*cloud_normals);

  // 欧式聚类
  double eps_angle=M_PI/15;
  float tolerance=0.2;
  std::vector<pcl::PointIndices> clusters;
  extractEuclideanClusters (*cloud, *cloud_normals, tolerance, tree, cluster_indices, eps_angle,1000,4000); 
  int j = 0;
  std::cout << "Find " << cluster_indices.size () << "  clusters." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clusters (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        cloud_cluster->push_back ((*cloud)[*pit]); //*
        // cloud_clusters->push_back ((*cloud)[*pit]); //*
      }
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
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
    // std::cout<<"rate: "<<l2/l1<<std::endl;
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
    // viewer.addPointCloud (cloud_cluster, cloud_cluster_color_handler, ss.str ());
    // std::stringstream centd;
    // centd<<"centroid"<<j;
    // viewer.addSphere (centroid,1.6, 1, 1, 1, centd.str());

    if(l2/l1<=0.5) continue;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        // cloud_cluster->push_back ((*cloud)[*pit]); //*
        cloud_clusters->push_back ((*cloud)[*pit]); //*
      }
    

  }
  cloud_clusters->width = cloud_clusters->size ();
  cloud_clusters->height = 1;
  cloud_clusters->is_dense = true;
  std::stringstream ss;
  // ss << "cloud_clusters.pcd" ;//<< j ;//<< ".pcd";
  // writer.write<pcl::PointXYZ> (ss.str (), *cloud_clusters, false); //*

  // 二次聚类
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
  tree2->setInputCloud (cloud_clusters);

  std::vector<pcl::PointIndices> cluster_indices2;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne2;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  ne.setSearchMethod (tree2);
  ne.setInputCloud (cloud_clusters);
  ne.setKSearch (25);
  ne.compute (*cloud_normals2);

  double eps_angle2=M_PI;
  float tolerance2=1.5;
  // std::vector<pcl::PointIndices> clusters;
  extractEuclideanClusters (*cloud_clusters, *cloud_normals2, tolerance2, tree2, cluster_indices2, eps_angle2,30000,40000);
  std::cout << "Find " << cluster_indices2.size () << "  clusters." << std::endl;
  j=0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster2 (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices2.begin (); it != cluster_indices2.end (); ++it)
  {
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        cloud_cluster2->push_back ((*cloud_clusters)[*pit]); //*
      }
      j++;
      std::stringstream ss;
      ss << "cloud_cluster" << j ;
      int r,g,b,gap;
      gap=256*256*128/cluster_indices2.size () ;
      // std::cout<<"gap"<<gap<<std::endl;
      r=(j*gap)%256;
      g=((j*gap)/256)%256;
      b=(((j*gap)/256)/256)%256;
      // r=(l2/l1>0.5)?(j*gap)%256:255;
      // g=(l2/l1>0.5)?((j*gap)/256)%256:255;
      // b=(l2/l1>0.5)?(((j*gap)/256)/256)%256:255;
      // r=255;
      // g=0;
      // b=0;
      std::cout << "PointCloud representing the Cluster: " << cloud_cluster2->size () << " data points." << std::endl;
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>cloud_cluster_color_handler (cloud_cluster2, r, g, b);
      viewer.addPointCloud (cloud_cluster2, cloud_cluster_color_handler, ss.str ());
  }
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moment;
  moment.setInputCloud(cloud_cluster2);
  moment.compute();
  Eigen::Vector3f center, vx, vy, vz;
  float l1,l2,l3;
  moment.getEigenVectors(vx, vy, vz);
  moment.getEigenValues(l1,l2,l3);
  std::stringstream line_dots;
  for(int i =-50;i<50;i++)
  {
    pcl::PointXYZ dot_line(vx[0]*i,vx[1]*i,vx[2]*i);
    line_dots<<"line_dot"<<i;
    viewer.addSphere (dot_line,0.4, 1, 0, 0, line_dots.str());
  }

  


    
  while (!viewer.wasStopped ()) 
  { 
    viewer.spinOnce ();
  }

  return (0);
}