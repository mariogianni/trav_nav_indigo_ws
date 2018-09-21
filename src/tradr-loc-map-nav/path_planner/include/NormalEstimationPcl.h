#ifndef NORMALESTIMATIONPCL_H
#define NORMALESTIMATIONPCL_H

#include <ros/ros.h>
#include <ros/console.h>

#include <NormalEstimationPclConfig.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <boost/thread.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <nav_msgs/Path.h>
#include <tf/tf.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <cmath>

#include "KdTreeFLANN.h"


using namespace path_planner;

template<typename PointT>
class NormalEstimationPcl
{

    static const float kLaserZOffset; // [m] how much each laser view point is pushed higher
    static const float kLaserZWrtBody; // [m] actual delta_z between laser view point and base_link
    static const int kMinNumNeighboursForComputingNormal; // minimum number of neighbours for computing the normal 
    
public:     
    
    typedef pcl::PointCloud<PointT> PointCloudT;

    typedef pp::KdTreeFLANN<PointT> KdTreeT;
    
public:
    
    NormalEstimationPcl();
    ~NormalEstimationPcl();
    
    void computeNormals(PointCloudT& pcl, KdTreeT& kdtree, const pcl::PointXYZ& center);

public: ///  < setters     
    
    inline void setConfig(const NormalEstimationPclConfig& new_config)
    {
        config_ = new_config;
    }

    // Set previously built map base_link trajectory 
    void setPrevMapTraj(const nav_msgs::Path& base_link_traj);
    
    // Set laser trajectory 
    void setLaserTraj(const nav_msgs::Path& base_link_traj);
    
public:  /// < getters 
    
    inline NormalEstimationPclConfig& getConfig()
    {
        return config_;
    }
    
    
protected:
    
    // Kernel between two vectors
    inline float kernel(Eigen::Vector3f& v1, Eigen::Vector3f& v2);
    inline float gaussianKernel(Eigen::Vector3f& v1, Eigen::Vector3f& v2);
    inline float cosineKernel(Eigen::Vector3f& v1, Eigen::Vector3f& v2);



    // Compute the covariance matrix between the point center and his neighbors. The covariance matrix is weighted with kernel function
    void computeCovarianceMatrix(const PointCloudT& neighbors, const pcl::PointXYZ& center, Eigen::Matrix3f& covariance_matrix);

    // Compute one normal for a given point i
    bool computeNormal(const size_t i, const size_t begin, const size_t end, PointCloudT& pcl, KdTreeT& kdtree, std::vector<bool>& done, const pcl::PointXYZ& center);

    // Compute normals in a given range
    void computeNormalsInRange(const size_t num_thread, const size_t start, const size_t end, PointCloudT& pcl, KdTreeT& kdtree, std::vector<bool>& done, const pcl::PointXYZ& center);

    // Add a new laser center to the laser trajectory 
    void addPointToLaserTraj(const pcl::PointXYZ& center); 
    
    // Compute closest laser point to each point of the cloud 
    void computeClosestLaserPoints(PointCloudT& pcl); 
        
protected:
    
    NormalEstimationPclConfig config_;
    
    // Minimum value of the weight in the computation of the weighted covariance matrix
    const float threshold_;
    
    boost::recursive_mutex pcl_laser_traj_mutex_;
    PointCloudT pcl_laser_trajectory_;     // laser trajectory
    PointCloudT pcl_prev_map_trajectory_;  // previously built map trajectory
    KdTreeT kdtree_laser_centers_;
    
    std::vector<size_t> closest_laser_point_idx_; 
};

#endif //NORMALESTIMATIONPCL_H
