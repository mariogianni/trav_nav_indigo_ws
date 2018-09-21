#ifndef MAKE_NORMAL_MARKERS_H_
#define MAKE_NORMAL_MARKERS_H_

#include <cmath>
#include <vector>
#include <set>

#include <ros/ros.h>

#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <VoxelBinaryKey.h>

template<typename PointT>
void makeNormalsMarkers(const pcl::PointCloud<PointT>& pcl_norm, geometry_msgs::PoseArray& markers, double leaf_size = 0.1)
{
    // fill header
    markers.header.frame_id = pcl_norm.header.frame_id;
    markers.header.stamp = ros::Time::now();

    // X axis
    tf::Vector3 x = tf::Vector3(1, 0, 0);

    std::set<uint64_t> k;

    for (int i = 0; i < pcl_norm.size(); i++)
    {
        uint64_t ki = voxelBinaryKey(pcl_norm[i], leaf_size);
        if (k.count(ki)) continue;

        //if (!std::isnan(pcl_norm.points[i].normal_x) && !std::isnan(pcl_norm.points[i].normal_y) && !std::isnan(pcl_norm.points[i].normal_z) &&
        //        !std::isinf(pcl_norm.points[i].normal_x) && !std::isinf(pcl_norm.points[i].normal_y) && !std::isinf(pcl_norm.points[i].normal_z))
        if (std::isnormal(pcl_norm.points[i].normal_x) && std::isnormal(pcl_norm.points[i].normal_y) && std::isnormal(pcl_norm.points[i].normal_z) )
        {
            geometry_msgs::Pose pose;
            //tf::Vector3 v = tf::Vector3(pcl_norm.points[i].normal[0], pcl_norm.points[i].normal[1], pcl_norm.points[i].normal[2]);
            tf::Vector3 v = tf::Vector3(pcl_norm.points[i].normal_x, pcl_norm.points[i].normal_y, pcl_norm.points[i].normal_z);
            if (v.length2() > 1e-3)
            {
                tf::Vector3 cross = tf::tfCross(x, v);
                pose.position.x = pcl_norm.points[i].x;
                pose.position.y = pcl_norm.points[i].y;
                pose.position.z = pcl_norm.points[i].z;
                tf::Quaternion q = tf::Quaternion(cross.x(), cross.y(), cross.z(), sqrt(v.length2() * x.length2()) + tf::tfDot(x, v));
                q.normalize();
                tf::quaternionTFToMsg(q, pose.orientation);
                markers.poses.push_back(pose);
                k.insert(ki);
            }
        }
    }
}

#endif // MAKE_NORMAL_MARKERS_H_
