#include <ros/ros.h>
#include <DynamicJoinPcl.h>
#include <ConversionPcl.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>

DynamicJoinPcl<pcl::PointXYZ, pcl::PointXYZRGBNormal> dynjoinpcl;
ConversionPcl<pcl::PointXYZ> conv_pcl;
ros::Publisher pcl_pub, quads_marker_array_pub;

pcl::PointCloud<pcl::PointXYZRGBNormal> map_pcl;

std::string laser_frame_name;
std::string global_frame_name;

inline void addPoint(const pcl::PointXYZ& p, visualization_msgs::Marker& marker)
{
	geometry_msgs::Point q;
	q.x = p.x;
	q.y = p.y;
	q.z = p.z;
	marker.points.push_back(q);
}

size_t oldMarkersCount = 0;

void makeQuadMarker(const Quad& quad, visualization_msgs::Marker& marker)
{
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time();
	marker.ns = "quads";
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker.id = oldMarkersCount++;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.r = quad.color.r / 255.0;
	marker.color.g = quad.color.g / 255.0;
	marker.color.b = quad.color.b / 255.0;
	marker.color.a = 0.4;
	marker.colors.clear();
	marker.points.clear();
	static size_t quad_idxs[6] = {0, 1, 2, 0, 2, 3};
	for(size_t j = 0; j < 6; j++)
		addPoint(quad.p[quad_idxs[j]], marker);
}

void makeQuadsMarkerArray(const std::vector<Quad>& quads, visualization_msgs::MarkerArray& markerArray)
{
	for(size_t i = 0; i < oldMarkersCount; i++)
	{
		visualization_msgs::Marker m;
		m.id = i;
		m.action = visualization_msgs::Marker::DELETE;
	}
	oldMarkersCount = 0;
	for(size_t i = 0; i < quads.size(); i++)
	{
		visualization_msgs::Marker m;
		makeQuadMarker(quads[i], m);
		markerArray.markers.push_back(m);
	}
}

void pointCloudCallback(const sensor_msgs::PointCloud2& scan_msg) {
	ROS_INFO("Received a new PointCloud2 message");

	DynamicJoinPclConfig dynjoinpcl_config = dynjoinpcl.getConfig();

	std::vector<Quad> quads;

	pcl::PointCloud<pcl::PointXYZ> scan_pcl;
	pcl::PointCloud<pcl::PointXYZRGBNormal> map_new_pcl;
	pcl::PointXYZ laser_center;

	map_new_pcl.header = map_pcl.header;

	conv_pcl.transform(scan_msg, scan_pcl);
	conv_pcl.getFrameOrigin(dynjoinpcl_config.laser_frame, laser_center);

	dynjoinpcl.joinPCL(scan_pcl, map_pcl, map_new_pcl, laser_center, quads);
	map_pcl.swap(map_new_pcl);

	sensor_msgs::PointCloud2 map_msg_out;
	pcl::toROSMsg(map_pcl, map_msg_out);

	pcl_pub.publish(map_msg_out);

	visualization_msgs::MarkerArray quads_marker_array;
	makeQuadsMarkerArray(quads, quads_marker_array);
	quads_marker_array_pub.publish(quads_marker_array);
}

void dynjoinpclConfigCallback(DynamicJoinPclConfig& config, uint32_t level)
{
	dynjoinpcl.setConfig(config);
	conv_pcl.setOutputFrame(config.global_frame);
	map_pcl.header.frame_id = config.global_frame;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_dynjoinpcl");

	tf::TransformListener tf_listener(ros::Duration(10.0));

	ros::NodeHandle n("~");

	dynamic_reconfigure::Server<DynamicJoinPclConfig> dynjoinpcl_config_server(ros::NodeHandle("~/DynamicJoinPcl"));
	dynjoinpcl_config_server.setCallback(boost::bind(&dynjoinpclConfigCallback, _1, _2));

    conv_pcl.setTFListener(tf_listener);

	ros::Subscriber sub_pcl = n.subscribe("/dynamic_point_cloud", 1, pointCloudCallback);
	pcl_pub = n.advertise<sensor_msgs::PointCloud2>("/dynjoinpcl", 1, true);
	quads_marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("/quads_marker_array", 1, true);

	ros::spin();
	return 0;
}

