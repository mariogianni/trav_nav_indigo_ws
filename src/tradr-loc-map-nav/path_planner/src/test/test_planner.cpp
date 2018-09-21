#include <ros/ros.h>
#include <DynamicJoinPcl.h>
#include <ClusterPcl.h>
#include <ConversionPcl.h>
#include <NormalEstimationPcl.h>
#include <ColorNormalsPcl.h>
#include <MakeNormalsMarkers.h>
#include <TravAnalyzer.h>
#include <PathPlanner.h>
#include <MarkerController.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/thread.hpp>

#include "KdTreeFLANN.h"


typedef int ClusterInfo;

DynamicJoinPcl<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> dynjoinpcl;
NormalEstimationPcl<pcl::PointXYZRGBNormal> normal_estimator;
ConversionPcl<pcl::PointXYZ> conv_pcl;
ClusterPcl<pcl::PointXYZRGBNormal> clustering_pcl;
TravAnalyzer trav_analyzer;

bool threadRunning = false;


ros::Publisher pcl_normal_pub;
ros::Publisher marker_normal_pub;
ros::Publisher pcl_pub_dyn;
ros::Publisher pcl_pub_nowall;
ros::Publisher pcl_pub_wall;
ros::Publisher pcl_pub_borders;
ros::Publisher pcl_pub_segmented;
ros::Publisher pcl_pub_traversability;
PathPlanner* p_planner;
ros::Publisher pcl_pub_clearence;
ros::Publisher pcl_pub_density;
ros::Publisher pcl_pub_label;
ros::Publisher pcl_pub_roughness;

ros::Publisher robot_global_path_pub;

pcl::PointCloud<pcl::PointXYZRGBNormal> map_pcl, nowall_pcl, border_pcl, segmented_pcl, wall_pcl;

tf::TransformListener *p_tf;

bool is_goal_selected = false;

bool getRobotPose(tf::StampedTransform& robot_pose) {

	if (p_tf->waitForTransform("/map", "base_link", ros::Time(),
			ros::Duration(1.0))) {
		try {
			p_tf->lookupTransform("/map", "base_link", ros::Time(),
					robot_pose);
		} catch (tf::LookupException& ex) {
			ROS_INFO(
					"No Transform available Error looking up robot pose: %s\n", ex.what());
			return false;
		} catch (tf::ConnectivityException& ex) {
			ROS_INFO(
					"Connectivity Error looking up robot pose: %s\n", ex.what());
			return false;
		} catch (tf::ExtrapolationException& ex) {
			ROS_INFO(
					"Extrapolation Error looking up robot pose: %s\n", ex.what());
			return false;
		}

		return true;
	} else {
		ROS_INFO("Transformation is not available");
		return false;
	}
}

void visualizeNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcl_norm)
{
	geometry_msgs::PoseArray poseArray;
	makeNormalsMarkers(pcl_norm, poseArray);
	ROS_INFO("normals poseArray size: %ld",poseArray.poses.size());
	marker_normal_pub.publish(poseArray);
}

void pointCloudCallback(const sensor_msgs::PointCloud2& scan_msg) {

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

	if (map_pcl.size() > 0){

		PathPlanner::KdTreeFLANN trav_pcl_tree;
		pp::KdTreeFLANN<pcl::PointXYZRGBNormal> wall_kdTree, input_kdtree;

		input_kdtree.setInputCloud(map_pcl.makeShared());
		std::vector<ClusterInfo> cluster_info;
		pcl::PointCloud<pcl::PointXYZI> traversability_pcl, clearence_pcl, density_pcl, label_pcl, roughness_pcl;

		clustering_pcl.setInputPcl(map_pcl);
		clustering_pcl.setKdtree(input_kdtree);
		clustering_pcl.clustering(nowall_pcl, border_pcl, segmented_pcl,wall_pcl);
		clustering_pcl.getClusterInfo(cluster_info);
		cout<<"clusters_info size: "<<cluster_info.size()<<endl;

		nowall_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		wall_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		border_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		segmented_pcl.header.frame_id = dynjoinpcl_config.global_frame;


		pcl_pub_nowall.publish(nowall_pcl);
		pcl_pub_wall.publish(wall_pcl);
		pcl_pub_segmented.publish(segmented_pcl);


		trav_analyzer.setInput(cluster_info, wall_pcl, segmented_pcl);
		trav_analyzer.computeTrav(traversability_pcl);
		trav_analyzer.getPcl(clearence_pcl, density_pcl, label_pcl, roughness_pcl);

		traversability_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		density_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		label_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		roughness_pcl.header.frame_id = dynjoinpcl_config.global_frame;
		clearence_pcl.header.frame_id = dynjoinpcl_config.global_frame;

		trav_pcl_tree.setInputCloud(traversability_pcl.makeShared());
		wall_kdTree.setInputCloud(wall_pcl.makeShared());

		sensor_msgs::PointCloud2 trav_msg_out;
		pcl::toROSMsg(traversability_pcl, trav_msg_out);
		pcl_pub_traversability.publish(trav_msg_out);

		pcl::toROSMsg(clearence_pcl, trav_msg_out);
		pcl_pub_clearence.publish(trav_msg_out);

		pcl::toROSMsg(roughness_pcl, trav_msg_out);
		pcl_pub_roughness.publish(trav_msg_out);

		pcl::toROSMsg(label_pcl, trav_msg_out);
		pcl_pub_label.publish(trav_msg_out);

		pcl::toROSMsg(density_pcl, trav_msg_out);
		pcl_pub_density.publish(trav_msg_out);

		tf::StampedTransform robot_pose;
		getRobotPose(robot_pose);
		pcl::PointXYZI robot;
		robot.x = robot_pose.getOrigin().x();
		robot.y = robot_pose.getOrigin().y();
		robot.z = robot_pose.getOrigin().z();

		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		trav_pcl_tree.nearestKSearch(robot, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
		p_planner->setInput(traversability_pcl, wall_pcl, wall_kdTree, trav_pcl_tree, pointIdxNKNSearch[0]);
		ROS_INFO("path planner input set");

		if (is_goal_selected){

			nav_msgs::Path path;

			ROS_INFO("compute path");
			if(is_goal_selected){
				if(p_planner->planning(path)){
					robot_global_path_pub.publish(path);
					is_goal_selected = false;
				}
				else{
					ROS_INFO("no path exist for desired goal, please choose another goal");
					is_goal_selected = false;
				}
				ROS_INFO("path_computed");
			}
		}
	}
}

void goalSelectionCallback(geometry_msgs::PoseStamped goal_){
	pcl::PointXYZI  p;
	p.x = goal_.pose.position.x;
	p.y = goal_.pose.position.y;
	p.z = goal_.pose.position.z;
	p_planner->setGoal(p);
	is_goal_selected = true;
	ROS_INFO("goal selection");
}

void dynjoinpclConfigCallback(DynamicJoinPclConfig& config, uint32_t level)
{
	dynjoinpcl.setConfig(config);
	conv_pcl.setOutputFrame(config.global_frame);
	map_pcl.header.frame_id = config.global_frame;

	// tell about leaf_size to travanal:
	TravAnalyzerConfig travcfg = trav_analyzer.getConfig();
	travcfg.leaf_size = config.leaf_size;
	trav_analyzer.setConfig(travcfg);
}

void normalConfigCallback(NormalEstimationPclConfig& config, uint32_t level)
{
	normal_estimator.setConfig(config);
}

void clusteringpclConfigCallback(ClusterPclConfig& config, uint32_t level)
{
	clustering_pcl.setConfig(config);
}

void travConfigCallback(TravAnalyzerConfig& config, uint32_t level)
{
	trav_analyzer.setConfig(config);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_cluster");

	tf::TransformListener tf;
	p_tf = &tf;

	tf::TransformListener tf_listener(ros::Duration(10.0));

	ros::NodeHandle n("~");

	PathPlanner new_planner(n);
	p_planner = &new_planner;

	dynamic_reconfigure::Server<DynamicJoinPclConfig> dynjoinpcl_config_server(ros::NodeHandle("~/DynamicJoinPcl"));
	dynjoinpcl_config_server.setCallback(boost::bind(&dynjoinpclConfigCallback, _1, _2));

	dynamic_reconfigure::Server<NormalEstimationPclConfig> normal_config_server(ros::NodeHandle("~/NormalEstimationPcl"));
	normal_config_server.setCallback(boost::bind(&normalConfigCallback, _1, _2));

	dynamic_reconfigure::Server<ClusterPclConfig> clusteringpcl_config_server(ros::NodeHandle("~/ClusteringPcl"));
	clusteringpcl_config_server.setCallback(boost::bind(&clusteringpclConfigCallback, _1, _2));

	dynamic_reconfigure::Server<TravAnalyzerConfig> trav_config_server(ros::NodeHandle("~/TravAnal"));
	trav_config_server.setCallback(boost::bind(&travConfigCallback, _1, _2));

	conv_pcl.setTFListener(tf_listener);

	ros::Subscriber sub_pcl = n.subscribe("/dynamic_point_cloud", 1, pointCloudCallback);
	ros::Subscriber sub_goal = n.subscribe("/goal_topic", 1, goalSelectionCallback);

	pcl_pub_dyn = n.advertise<sensor_msgs::PointCloud2>("/dynjoinpcl", 1, true);
	pcl_pub_nowall = n.advertise<sensor_msgs::PointCloud2>("/clustered_pcl/no_wall", 1, true);
	pcl_pub_wall = n.advertise<sensor_msgs::PointCloud2>("/clustered_pcl/wall", 1, true);

	pcl_pub_traversability = n.advertise<sensor_msgs::PointCloud2>("/trav/traversability", 1, true);
	pcl_pub_clearence = n.advertise<sensor_msgs::PointCloud2>("/trav/clearence", 1, true);
	pcl_pub_density = n.advertise<sensor_msgs::PointCloud2>("/trav/density", 1, true);
	pcl_pub_label = n.advertise<sensor_msgs::PointCloud2>("/trav/label", 1, true);
	pcl_pub_roughness = n.advertise<sensor_msgs::PointCloud2>("/trav/roughness", 1, true);

	pcl_pub_segmented = n.advertise<sensor_msgs::PointCloud2>("/clustered_pcl/segmented", 1, true);
	pcl_normal_pub = n.advertise<sensor_msgs::PointCloud2>("/normals_pcl", 1, true);
	marker_normal_pub = n.advertise<geometry_msgs::PoseArray>("/normals_marker",1);

	robot_global_path_pub = n.advertise<nav_msgs::Path>("/robot_path",1);

	MarkerController marker;
	ros::spin();

	marker.reset();
	return 0;
}
