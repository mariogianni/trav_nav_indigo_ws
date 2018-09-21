#ifndef CONVERSIONPCL_H
#define CONVERSIONPCL_H

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <laser_geometry/laser_geometry.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>


///	\class ConversionPcl
///	\author Alcor
///	\brief A class for transforming point clouds from one frame to another
///	\note 
/// 	\todo 
///	\date
///	\warning
template<typename PointT>
class ConversionPcl
{
private:
    typedef pcl::PointCloud<PointT> PointCloudT;

    // /tf listener
    const tf::TransformListener* p_tf_listener_;

    // Name of the output frame
    std::string output_frame_;

    // Last transform from local to global frame
    tf::StampedTransform input_output_tf_;

public:
    ConversionPcl();
    ~ConversionPcl();

    inline void setOutputFrame(std::string f)
    {
        output_frame_ = f;
    }

    inline void setTFListener(const tf::TransformListener& tfl)
    {
        p_tf_listener_ = &tfl;
    }

    void transform(const sensor_msgs::PointCloud2& msg_in, sensor_msgs::PointCloud2& msg_out);
    void transform(const sensor_msgs::PointCloud2& msg_in, PointCloudT& pcl_out);
    void transform(const PointCloudT& pcl_in, sensor_msgs::PointCloud2& msg_out);
    void transform(const PointCloudT& pcl_in, PointCloudT& pcl_out);

    void getLastTransform(tf::Transform& t);
    void getLastTransform(tf::StampedTransform& t);

    void getFrameOrigin(std::string frame_id, pcl::PointXYZ& p);
    
    tf::StampedTransform getTransform(const std::string& parent, const std::string& child);
};
#endif // CONVERSIONPCL_H

