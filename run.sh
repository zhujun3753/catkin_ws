catkin_make
source ./devel/setup.bash
roscore

rosrun chapter6_tutorials pcl_create # 对roscore有依赖
rosrun rviz rviz

# 下面的命令catkin_make之后可以直接运行，不需要依赖roscore
# 写入点云文件
rosrun chapter6_tutorials pcd_write
cat test_pcd.pcd # 输出点云数据
# 读取并显示点云文件
 rosrun chapter6_tutorials viewpcd table_scene_lms400.pcd 
# 计算法向量
rosrun chapter6_tutorials normal_estimation_using_integral_images 
# 使用VoxelGrid过滤器对PointCloud进行下采样
rosrun chapter6_tutorials voxel_grid 

rosrun chapter6_tutorials cylinder_segmentation support_dense_inliers_downsampled.pcd 

rosrun chapter6_tutorials global_hypothesis_verification milk.pcd milk_cartoon_all_small_clorox.pcd

# 切分支架
rosrun chapter6_tutorials cluster_extraction ver_trans.pcd
