#include <ros/ros.h> 
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 

main (int argc, char **argv) 
{ 
	float table[8]={-2,-1.5,-1,-0.5,0.5,1,1.5,2};
    int point_num;
	ros::init (argc, argv, "pcl_create"); 
	ros::NodeHandle nh; 
	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);     
	pcl::PointCloud<pcl::PointXYZ> cloud; 
	sensor_msgs::PointCloud2 output; 
	
	// Fill in the cloud data 
	cloud.width = 512; 
	cloud.height = 1; 
	cloud.points.resize(cloud.width * cloud.height); 
	for(int a=0;a<8;++a)
	{
		float width = table[a];
		for(int i=0;i<8;++i)
		{
			float length = table[i];
			for(int c=0;c<8;++c)
			{
				 point_num = a*64+i*8+c;
				 cloud.points[point_num].x = width; 
				 cloud.points[point_num].y = length; 
				 cloud.points[point_num].z = table[c]; 
			}
		}	
	}
	// 创建点云文件
	pcl::io::savePCDFileASCII ("cube.pcd", cloud);
  	std::cerr << "Saved " << cloud.size () << " data points to cube.pcd." << std::endl;
	//Convert the cloud to ROS message 
	pcl::toROSMsg(cloud, output); 
	output.header.frame_id = "odom"; 
	
	ros::Rate loop_rate(1); 
	while (ros::ok()) 
	{ 
		 pcl_pub.publish(output);
		 ros::spinOnce(); 
		 loop_rate.sleep(); 
	 } 
	return 0; 
}

