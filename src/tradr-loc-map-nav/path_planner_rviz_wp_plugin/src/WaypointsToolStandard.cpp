/* Tell pluginlib about the class.  It is important
 * to do this in global scope, outside our package's namespace. */


#define WAYPOINT_REDEFINED_TOPICS

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(path_planner_rviz_wp_plugin::WaypointsTool, rviz::Tool)

static const std::string pcl_input_topic_name = "/dynjoinpcl_nn";
static const std::string planner_task_feedback_topic_name = "/planner/tasks/feedback";
static const std::string planner_task_append_topic_name = "/planner/tasks/append";
static const std::string planner_task_remove_topic_name = "/planner/tasks/remove";
static const std::string planner_waypoints_server_name = "/planner/waypoints/server";


#include "WaypointsTool.cpp"