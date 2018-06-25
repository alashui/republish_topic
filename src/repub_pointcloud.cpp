#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <fstream>

#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h> 

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZRGB PointT;

class SubPointCloud
{
	ros::NodeHandle nh_;
	
	ros::Publisher cloud_downsampled_pub_;
	pcl::PointCloud<PointT>::Ptr cloud_receieved_;
	ros::Subscriber pointcloud_sub;
	
	constexpr static const float HEIGHT_DOWNSAMPLING = 0.03;
	
	public:
		
		SubPointCloud(ros::NodeHandle node):nh_(node),cloud_receieved_(new pcl::PointCloud<PointT>)
		{
			cloud_downsampled_pub_=node.advertise<pcl::PointCloud<PointT> >("/cloud_downsampled", 1); //downsampled cloud
			pointcloud_sub=node.subscribe("/camera/depth/points",1,&SubPointCloud::pointCloud_callback,this);
		}
		
		void pointCloud_callback(const sensor_msgs::PointCloud2& input);
};


void SubPointCloud::pointCloud_callback(const sensor_msgs::PointCloud2& input)
{
	std::cout<<"pointCloud_callback and downsample!" <<std::endl;       				
	pcl::fromROSMsg(input, *cloud_receieved_); 
	
	static pcl::VoxelGrid<PointT> filter_downsample;
	static pcl::PointCloud<PointT>::Ptr cloud_downsampled(new pcl::PointCloud<PointT>);

		
	//downsample cloud
	filter_downsample.setInputCloud(cloud_receieved_);
	filter_downsample.setLeafSize(  HEIGHT_DOWNSAMPLING, 
									HEIGHT_DOWNSAMPLING,
									HEIGHT_DOWNSAMPLING   );
	filter_downsample.filter(*cloud_downsampled);
	
	
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_downsampled, output); 
	
	output.header.frame_id=input.header.frame_id;
	output.header.stamp = output.header.stamp;
	
	cloud_downsampled_pub_.publish(output);
				
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "sub_pointcloud"); //string here is the node name
	ros::NodeHandle node("sub_pointcloud"); //string here is the namespace for parameters
	
	SubPointCloud sub_pointcloud_temp(node);
			
	ros::spin(); //blocks until the node is interrupted
}
