#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <SphericalPartition.h>

ros::Publisher pcl_pub;

uint8_t col_r[2][2] = {{0xFF,0x00},{0xFF,0x00}};
uint8_t col_g[2][2] = {{0x00,0xFF},{0xFF,0x7F}};
uint8_t col_b[2][2] = {{0x00,0x00},{0x00,0xFF}};

void pointCloudCallback(const sensor_msgs::PointCloud2& pcl_msg) {
	pcl::PointCloud<pcl::PointXYZ> pcl;
	pcl::fromROSMsg(pcl_msg, pcl);

	ROS_INFO("Received a new PointCloud2 message (%ld points)", pcl.size());

	pcl::PointXYZ center(0.0, 0.0, 0.0);
	int subdivisions = 8;
	ros::param::get("~center/x", center.x);
	ros::param::get("~center/y", center.y);
	ros::param::get("~center/z", center.z);
	ros::param::get("~subdivisions", subdivisions);

	pcl::PointCloud<pcl::PointXYZ> pcl_out[subdivisions * subdivisions];
	sphericalPartition(pcl, pcl_out, center, subdivisions);

	sensor_msgs::PointCloud2 pcl_out_msg;
	pcl::PointCloud<pcl::PointXYZRGB> pcl_color;
	for(size_t i = 0; i < subdivisions * subdivisions; i++)
	{
		ROS_INFO(" bucket #%ld has %ld points", i, pcl_out[i].size());

		srand(i);
		for(size_t j = 0; j < pcl_out[i].size(); j++)
		{
			pcl::PointXYZRGB p;
			p.x = pcl_out[i].points[j].x;
			p.y = pcl_out[i].points[j].y;
			p.z = pcl_out[i].points[j].z;
			p.r = col_r[(i / subdivisions) % 2][i % 2];
			p.g = col_g[(i / subdivisions) % 2][i % 2];
			p.b = col_b[(i / subdivisions) % 2][i % 2];
			pcl_color.push_back(p);
		}
	}
	pcl::toROSMsg(pcl_color, pcl_out_msg);

	pcl_out_msg.header = pcl_msg.header;

	pcl_pub.publish(pcl_out_msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_spherical_partition");
	ros::NodeHandle n("~");
	ros::Subscriber sub_pcl = n.subscribe("/dynamic_point_cloud", 1, pointCloudCallback);
	pcl_pub = n.advertise<sensor_msgs::PointCloud2>("cloud_color", 1, false);
	ros::spin();
	return 0;
}

