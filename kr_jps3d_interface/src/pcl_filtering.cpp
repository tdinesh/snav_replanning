#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>

ros::Publisher cloud_pub;

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
	ROS_INFO("pointcloud received");
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; 
	kdtree.setInputCloud (cloud); 
	pcl::PointXYZ searchPoint; 
	searchPoint.x = 0; 
	searchPoint.y = 0; 
	searchPoint.z = 2; 

	double rad = 8; // search radius 
	std::vector<int> pointIndicesOut; 
	std::vector<float> pointRadiusSquaredDistance; 
	kdtree.radiusSearch(searchPoint, rad, pointIndicesOut, pointRadiusSquaredDistance); 
	for (int i : pointIndicesOut) {
	  cloud_filtered->push_back(cloud->points[i]);
	} 
	sensor_msgs::PointCloud2 output_cloud;
	pcl::toROSMsg(*cloud_filtered,output_cloud);
	output_cloud.header.frame_id = "map";
	cloud_pub.publish(output_cloud);

}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "pcl_filtering");
  ros::NodeHandle nh("~");

  ros::Subscriber sub = nh.subscribe("/octomap_point_cloud_centers", 1, cloudCallback);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);

  ros::spin();

  return (0);
}
