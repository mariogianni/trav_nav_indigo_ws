#include <ros/ros.h>

#include <NormalEstimationPcl.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <dynamic_reconfigure/server.h>

NormalEstimationPcl<pcl::PointXYZRGBNormal> normal_estimator;

ros::Publisher pcl_pub;

double distance_threshold = 0.05;
double leaf_size = 0.05;

void removeIndices(pcl::PointIndices& i, pcl::PointIndices& from)
{
	// from is already sorted
	std::sort(i.indices.begin(), i.indices.end());
	std::vector<int> v(from.indices.size());
	std::vector<int>::iterator it = std::set_difference(from.indices.begin(), from.indices.end(), i.indices.begin(), i.indices.end(), v.begin());
	v.resize(it - v.begin());
	from.indices.swap(v);
}

void pointCloudCallback(const sensor_msgs::PointCloud2& pcl_msg)
{
	sensor_msgs::PointCloud2 pclout_msg;
	pcl::PointCloud<pcl::PointXYZRGB> pcl, pclout;
	pcl::fromROSMsg(pcl_msg, pcl);
	ROS_INFO("Received a new PointCloud2 message (%ld points)", pcl.size());

#ifdef DONT_DOWNSAMPLE
	pcl::copyPointCloud(pcl, pclout);
#else
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(pcl.makeShared());
	sor.setLeafSize(leaf_size, leaf_size, leaf_size);
	sor.filter(pclout);
#endif

#if 0
	pcl::PointCloud<pcl::PointXYZRGBNormal> normals;
	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> input_kdtree;
	pcl::copyPointCloud(pcl, normals);
	input_kdtree.setInputCloud(normals.makeShared());
	normal_estimation->computeNormals(normals, input_kdtree);

	pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> mps;
	mps.setMinInliers(10000);
	mps.setAngularThreshold(0.017453 * 2.0); // 2 degrees
	mps.setDistanceThreshold(0.02); // 2cm
	mps.setInputNormals(normals.makeShared());
	mps.setInputCloud(pcl.makeShared());
	std::vector<pcl::PlanarRegion<pcl::PointXYZRGB> > regions;
	mps.segmentAndRefine(regions);

	for (size_t i = 0; i < regions.size (); i++)
	{
	  Eigen::Vector3f centroid = regions[i].getCentroid();
	  Eigen::Vector4f model = regions[i].getCoefficients();
	  pcl::PointCloud boundary_cloud;
	  boundary_cloud.points = regions[i].getContour();
	  //regions[i].
	 }
#endif

#if 1
	pcl::ModelCoefficients coefficients;
	pcl::PointIndices remaining_indices, inliers;

	for(size_t i = 0; i < pclout.size(); i++)
		remaining_indices.indices.push_back(i);

	for(size_t i = 0; i < pclout.size(); i++)
	{
		pclout[i].r = pclout[i].g = pclout[i].b = 255;
		pclout[i].a = 120;
	}

	srand(666);
	int min_inliers = 100;
	int iter = 0;
	while(iter++ < 200)
	{

		inliers.indices.clear();

		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(distance_threshold);

		seg.setInputCloud(pclout.makeShared());
		seg.setIndices(boost::make_shared<pcl::PointIndices>(remaining_indices));
		seg.segment(inliers, coefficients);

		ROS_INFO("Iteration #%d - Remaining indices: %ld - Found %ld inliers", iter, remaining_indices.indices.size(), inliers.indices.size());

		if(inliers.indices.size() < min_inliers)
		{
			//PCL_ERROR("Could not estimate a planar model for the given dataset.");
			break;
		}

		uint8_t r = rand() % 255, g = rand() % 255, b = rand() % 255;
		if((r + b) < 100) g = 255;

		for(size_t i = 0; i < inliers.indices.size(); i++)
		{
			pclout[inliers.indices[i]].r = r;
			pclout[inliers.indices[i]].g = g;
			pclout[inliers.indices[i]].b = b;
			pclout[inliers.indices[i]].a = 255;
		}

		removeIndices(inliers, remaining_indices);
	}
#endif

	pcl::toROSMsg(pclout, pclout_msg);
	pcl_pub.publish(pclout_msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_segmentation");
	ros::NodeHandle n("~");
	n.param("distance_threshold", distance_threshold, distance_threshold);
	n.param("leaf_size", leaf_size, leaf_size);
	ros::Subscriber sub_pcl = n.subscribe("/dynamic_point_cloud", 1, pointCloudCallback);
	pcl_pub = n.advertise<sensor_msgs::PointCloud2>("cloud_seg", 1, false);
	ros::spin();
	return 0;
}

