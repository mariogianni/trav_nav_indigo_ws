#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <DistancePartition.h>

ros::Publisher pcl_pub_in, pcl_pub_out;

void pointCloudCallback(const sensor_msgs::PointCloud2& pcl_msg) {
	pcl::PointCloud<pcl::PointXYZ> pcl, pcl_inside, pcl_outside;
	pcl::fromROSMsg(pcl_msg, pcl);

	ROS_INFO("Received a new PointCloud2 message (%ld points)", pcl.size());

	pcl::PointXYZ center(0.0, 0.0, 0.0);
	double distance = 1.0;
	ros::param::get("~center/x", center.x);
	ros::param::get("~center/y", center.y);
	ros::param::get("~center/z", center.z);
	ros::param::get("~distance", distance);
	distancePartition(pcl, pcl_inside, pcl_outside, center, 1.0);

	ROS_INFO("After partition, pcl_inside has %ld points, pcl_outside has %ld points", pcl_inside.size(), pcl_outside.size());

	sensor_msgs::PointCloud2 pcl_inside_msg, pcl_outside_msg;
	pcl::toROSMsg(pcl_inside, pcl_inside_msg);
	pcl::toROSMsg(pcl_outside, pcl_outside_msg);

	pcl_inside_msg.header = pcl_msg.header;
	pcl_outside_msg.header = pcl_msg.header;

	pcl_pub_in.publish(pcl_inside_msg);
	pcl_pub_out.publish(pcl_outside_msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_distance_partition");
	ros::NodeHandle n("~");
	ros::Subscriber sub_pcl = n.subscribe("/dynamic_point_cloud", 1, pointCloudCallback);
	pcl_pub_in = n.advertise<sensor_msgs::PointCloud2>("cloud_inside", 1, false);
	pcl_pub_out = n.advertise<sensor_msgs::PointCloud2>("cloud_outside", 1, false);
	ros::spin();
	return 0;
}

