#include <ros/ros.h>
#include <DynamicJoinPcl.h>
#include <ClusterPcl.h>
#include <ConversionPcl.h>
#include <NormalEstimationPcl.h>
#include <ColorNormalsPcl.h>
#include <MakeNormalsMarkers.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>

#include "KdTreeFLANN.h"


DynamicJoinPcl<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> dynjoinpcl;
NormalEstimationPcl<pcl::PointXYZRGBNormal> normal_estimator;
ConversionPcl<pcl::PointXYZ> conv_pcl;
ClusterPcl<pcl::PointXYZRGBNormal> clustering_pcl;

ros::Publisher pcl_normal_pub;
ros::Publisher marker_normal_pub;
ros::Publisher pcl_pub_dyn;
ros::Publisher pcl_pub_nowall;
ros::Publisher pcl_pub_borders;
ros::Publisher pcl_pub_segmented;

pcl::PointCloud<pcl::PointXYZRGBNormal> map_pcl, nowall_pcl, border_pcl, segmented_pcl, wall_pcl;

void visualizeNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcl_norm)
{
	geometry_msgs::PoseArray poseArray;
	makeNormalsMarkers(pcl_norm, poseArray);
	ROS_INFO("normals poseArray size: %ld",poseArray.poses.size());
	marker_normal_pub.publish(poseArray);
}

void pointCloudCallback(const sensor_msgs::PointCloud2& scan_msg)
{
	ROS_INFO("Received a new PointCloud2 message");

	DynamicJoinPclConfig dynjoinpcl_config = dynjoinpcl.getConfig();

	// normal estimation

	pcl::PointCloud<pcl::PointXYZ> scan_pcl;
	pcl::PointCloud<pcl::PointXYZRGBNormal> scan_norm_pcl;
	conv_pcl.transform(scan_msg, scan_pcl);
	pcl::copyPointCloud(scan_pcl, scan_norm_pcl);

	pp::KdTreeFLANN<pcl::PointXYZRGBNormal> scan_norm_kdtree;
	scan_norm_kdtree.setInputCloud(scan_norm_pcl.makeShared());

	tf::Transform t;
	pcl::PointXYZ laser_center;
	conv_pcl.getLastTransform(t);
	conv_pcl.getFrameOrigin(dynjoinpcl_config.laser_frame, laser_center);
	normal_estimator.computeNormals(scan_norm_pcl, scan_norm_kdtree, laser_center);

	sensor_msgs::PointCloud2 scan_norm_msg;
	pcl::toROSMsg(scan_norm_pcl, scan_norm_msg);
	pcl_normal_pub.publish(scan_norm_msg);


	// dynamic join

	pcl::PointCloud<pcl::PointXYZRGBNormal> map_new_pcl;

	map_new_pcl.header = map_pcl.header;

	dynjoinpcl.joinPCL(scan_norm_pcl, map_pcl, map_new_pcl, laser_center);
	map_pcl.swap(map_new_pcl);

	//colorNormalsPCL(map_pcl);
	visualizeNormals(map_pcl);

	sensor_msgs::PointCloud2 map_msg_out;
	pcl::toROSMsg(map_pcl, map_msg_out);

	pcl_pub_dyn.publish(map_msg_out);

	if (map_pcl.size() > 0)
	{
		pp::KdTreeFLANN<pcl::PointXYZRGBNormal> input_kdtree;
		input_kdtree.setInputCloud(map_pcl.makeShared());

		clustering_pcl.setInputPcl(map_pcl);
		clustering_pcl.setKdtree(input_kdtree);
		clustering_pcl.clustering(nowall_pcl, border_pcl, segmented_pcl,wall_pcl);

		nowall_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		border_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		segmented_pcl.header.frame_id = dynjoinpcl_config.global_frame;

		pcl_pub_nowall.publish(nowall_pcl);
		pcl_pub_borders.publish(border_pcl);
		pcl_pub_segmented.publish(segmented_pcl);
	}
}

void dynjoinpclConfigCallback(DynamicJoinPclConfig& config, uint32_t level)
{
	dynjoinpcl.setConfig(config);
	conv_pcl.setOutputFrame(config.global_frame);
	map_pcl.header.frame_id = config.global_frame;
}

void normalConfigCallback(NormalEstimationPclConfig& config, uint32_t level)
{
	normal_estimator.setConfig(config);
}

void clusteringpclConfigCallback(ClusterPclConfig& config, uint32_t level)
{
	clustering_pcl.setConfig(config);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_cluster");

	tf::TransformListener tf_listener(ros::Duration(10.0));

	ros::NodeHandle n("~");

	dynamic_reconfigure::Server<DynamicJoinPclConfig> dynjoinpcl_config_server(ros::NodeHandle("~/DynamicJoinPcl"));
	dynjoinpcl_config_server.setCallback(boost::bind(&dynjoinpclConfigCallback, _1, _2));

	dynamic_reconfigure::Server<NormalEstimationPclConfig> normal_config_server(ros::NodeHandle("~/NormalEstimationPcl"));
	normal_config_server.setCallback(boost::bind(&normalConfigCallback, _1, _2));

	dynamic_reconfigure::Server<ClusterPclConfig> clusteringpcl_config_server(ros::NodeHandle("~/ClusteringPcl"));
	clusteringpcl_config_server.setCallback(boost::bind(&clusteringpclConfigCallback, _1, _2));

	conv_pcl.setTFListener(tf_listener);

	ros::Subscriber sub_pcl = n.subscribe("/dynamic_point_cloud", 1, pointCloudCallback);

	pcl_pub_dyn = n.advertise<sensor_msgs::PointCloud2>("/dynjoinpcl", 1, true);
	pcl_pub_nowall = n.advertise<sensor_msgs::PointCloud2>("/clustered_pcl/no_wall", 1, true);
	pcl_pub_borders = n.advertise<sensor_msgs::PointCloud2>("/clustered_pcl/borders", 1, true);
	pcl_pub_segmented = n.advertise<sensor_msgs::PointCloud2>("/clustered_pcl/segmented", 1, true);
	pcl_normal_pub = n.advertise<sensor_msgs::PointCloud2>("/normals_pcl", 1, true);
	marker_normal_pub = n.advertise<geometry_msgs::PoseArray>("/normals_marker",1);

	ros::spin();
	return 0;
}
