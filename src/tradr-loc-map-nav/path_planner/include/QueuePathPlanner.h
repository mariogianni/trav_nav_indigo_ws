#ifndef QUEUE_PLANNER_H
#define QUEUE_PLANNER_H

#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <trajectory_control_msgs/PlanningFeedback.h>
#include <trajectory_control_msgs/PlanningTask.h>
#include <trajectory_control_msgs/PlanningGlobalPath.h>
#include <pcl/filters/crop_box.h>

#include "PathPlanner.h"
#include "Transform.h"
#include "PathPlannerManager.h"

#include "KdTreeFLANN.h"

#include <trajectory_control_msgs/message_enums.h>

#define VERBOSE 1


///	\struct TaskSegment
///	\author Alcor
///	\brief A task-segment is composed by a start and a goal (two points). A task is instead a vector of task-segments
///	\note 
/// 	\todo 
///	\date
///	\warning
struct TaskSegment
{
    TaskSegment():b_aborted(false),crop_step(0){}
    
    SegmentStatus status;
    boost::recursive_mutex status_mutex; // status mutex (for synching on the basis of status)
    
    boost::recursive_mutex mutex; // segment mutex 
    boost::thread thread;
    
    nav_msgs::Path path;
    
    trajectory_control_msgs::PlanningTask segment_task;
    
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr p_wall_pcl;
    pp::KdTreeFLANN<pcl::PointXYZRGBNormal> wall_kdtree;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr p_traversability_pcl;
    PathPlanner::KdTreeFLANN traversability_kdtree;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr p_utility_2d_pcl;
    
    boost::shared_ptr<PathPlanner> p_path_planner; // the used path planner instance
    volatile bool b_aborted; // true if we want to abort it 
    
    int crop_step; // crop step applied by the path planner (see QueuePathPlanner::CropBoxMethod)
}; 



///	\class QueuePathPlanner
///	\author Alcor
///	\brief 
///	\note 
/// 	\todo 
///	\date
///	\warning
class QueuePathPlanner
{
    
    static const float kMinXSizeCropBox; // [m] minimum size along X axis of the crop box for planning 
    static const float kMinYSizeCropBox; // [m] minimum size along Y axis of the crop box for planning 
    static const float kMinZSizeCropBox; // [m] minimum size along Z axis of the crop box for planning 
    static const float kGainCropBox; 
    static const float kGainCropBox2;
    
    static const float kGainXCropBoxPathAligned; // crop box extends from (-kGainXCropBoxPathAligned* delta.norm(), -0.5*kSizeYCropBoxPathAligned, -0.5*kSizeZCropBoxPathAligned, 1) to (kGainXCropBoxPathAligned * delta.norm(), 0.5*kSizeYCropBoxPathAligned, 0.5*kSizeZCropBoxPathAligned, 1)
    static const float kSizeYCropBoxPathAligned; // 
    static const float kSizeZCropBoxPathAligned; // 
    
    static const float kTaskCallbackPeriod;  // [s] the duration period of the task callback 
        
public:
    
    typedef PathPlannerManager::CropBoxMethod CropBoxMethod;
    
    // a task-segment is composed by a start and a goal (two points)
    typedef boost::shared_ptr<TaskSegment> TaskSegmentPtr; 
        
    // a task is a vector of task-segments; a task-segment is composed by a start and a goal (two points)
    struct Task: public std::vector<TaskSegmentPtr> 
    {
        boost::recursive_mutex cropbox_pcl_mutex; 
        pcl::PointCloud<pcl::PointXYZI> cropbox_pcl; // the resulting union of cropped pcls
        pcl::PointCloud<pcl::PointXYZI> cropbox_test_pcl; // the currently tested pcl
        
        TaskType type; 
    };
    typedef boost::shared_ptr<Task> TaskPtr; 
    
public:
    ~QueuePathPlanner();
    QueuePathPlanner();
    
protected:
    // path planner library stuff: cloud of segmented walls
    volatile bool wall_flag_;
    pcl::PointCloud<pcl::PointXYZRGBNormal> wall_pcl_;
    boost::recursive_mutex wall_pcl_mutex_; 
    
    // path planner library stuff: traversability cloud
    volatile bool traversability_flag_;
    pcl::PointCloud<pcl::PointXYZI> traversability_pcl_;
    boost::recursive_mutex traversability_pcl_mutex_; 
    
    // path planner library stuff: utility cloud (same points of traversability)
    volatile bool utility_2d_flag_;
    boost::recursive_mutex utility_2d_mutex_;
    pcl::PointCloud<pcl::PointXYZI> utility_2d_pcl_; // [x,y,u(x,y),var(x,y)] we assume that utility_2d_pcl_ contains info about the points in traversability_pcl_
    
    // queue and multi-threading stuff:
    // each queue element or task is a vector of task-segments
    //std::list< std::vector<TaskSegment*> > task_queue_;
    std::list<TaskPtr> task_queue_;
    boost::recursive_mutex task_queue_mutex_; 

    // ros stuff
    ros::NodeHandle node_;
    ros::NodeHandle param_node_;
    std::string reference_frame_;
    std::string robot_frame_;
    Transform transform_;

    // ros stuff: path planner library stuff
    ros::Timer task_timer_;

    volatile bool controller_ready_flag_; // false when controller is busy, true when controller is idle 
    ros::Publisher task_feedback_pub_;
    ros::Subscriber task_feedback_sub_;
    void feedbackCallback(const trajectory_control_msgs::PlanningFeedback& feedback_msg);

    ros::Subscriber wall_sub_;
    void wallCallback(const sensor_msgs::PointCloud2& wall_msg);

    ros::Subscriber traversability_sub_;
    void traversabilityCallback(const sensor_msgs::PointCloud2& traversability_msg);
    
    ros::Subscriber utility_2d_sub_;
    void utility2DCloudCallback(const sensor_msgs::PointCloud2& utility_msg);

    // ros stuff: tasks handling
    ros::Publisher task_path_pub_;
    ros::Publisher task_global_path_pub_;
    boost::recursive_mutex task_path_pub_mutex_; 
        
    ros::Subscriber task_append_sub_; // subscriber for append request
    ros::Subscriber task_remove_sub_; // subscriber for remove request
    ros::Publisher task_remove_pub_; // publisher for remove request (on destruction)
    void appendCallback(const trajectory_control_msgs::PlanningTask& task_msg);
    void removeCallback(const trajectory_control_msgs::PlanningTask& task_msg);
    
    // this callback is called automatically every kTaskCallbackPeriod seconds 
    void taskTimerCallback(const ros::TimerEvent& timer_msg);

    // the main path planning callback: perform the planning on the input segment of the given input task
    void pathPlanningCallback(TaskSegmentPtr segment, TaskPtr task);
    
    void publishCropboxPcl(TaskSegmentPtr segment, TaskPtr task);
    void publishTestCropboxPcl(TaskSegmentPtr segment, TaskPtr task);
    
    // cloud cropping visualization
#ifdef VERBOSE
    ros::Publisher cropbox_pub_;
    boost::recursive_mutex cropbox_pub_mutex_; 
#endif
    
    CropBoxMethod crop_box_method_; // type of crop box which we want to apply:  
    
    
    int cost_function_type_;
    float lambda_trav_;
    float lambda_utility_2d_;
    float tau_exp_decay_;
};

#endif
