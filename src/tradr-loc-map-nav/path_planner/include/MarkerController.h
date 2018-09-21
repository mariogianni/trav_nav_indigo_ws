#ifndef MARKER_CONTROLLER_H_
#define MARKER_CONTROLLER_H_

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>
#include <tf/tf.h>

#include <cstring>
#include <boost/thread/recursive_mutex.hpp>


#include "ColorUtils.h"


class MarkerController
{
    
    static const std::string kDefaultMakerName; 
    static const std::string kDefaultMakerDescription; 
    static const float kInitialVerticalOffset; 
    static const float kTextMessageHeight; 
    static const float kTextMessageVerticalOffset; 
    
public:
    visualization_msgs::InteractiveMarker int_marker_;
    ros::Publisher goal_pub_;
    ros::Publisher goal_abort_pub_;

    boost::recursive_mutex marker_server_mtx; 
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
    interactive_markers::MenuHandler menu_handler_;

    std::string marker_name_; 
    
public: 
    MarkerController(const tf::Vector3& p0 = tf::Vector3(), 
                     const std::string& goal_topic_name = "/goal_topic", 
                     const std::string& goal_abort_topic_name = "/goal_abort_topic", 
                     const std::string& int_marker_server_name = "marker_controller",
                     const std::string& marker_name = kDefaultMakerName); 
    ~MarkerController();
    
    visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker&, const Color& color = Colors::LightGrey()  );
    visualization_msgs::InteractiveMarkerControl& makeBoxControl(visualization_msgs::InteractiveMarker&);
    void makeViewFacingMarker();
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
    void reset();
    
    void setMarkerPosition(const geometry_msgs::Pose &pose); 
    void setMarkerColor(const Color& color, const std::string& text = std::string()); 
    
};


#endif //MARKER_CONTROLLER
