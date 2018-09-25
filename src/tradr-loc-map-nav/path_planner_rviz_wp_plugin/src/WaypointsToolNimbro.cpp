#include "WaypointsToolNimbro.h"


#include "WaypointsTool.cpp" /// <  include the source code for the base class

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(path_planner_rviz_wp_plugin::WaypointsToolNimbro, rviz::Tool)

namespace path_planner_rviz_wp_plugin
{

WaypointsToolNimbro::WaypointsToolNimbro(
    const std::string& pcl_input_topic_name,
    const std::string& planner_task_feedback_topic_name,   
    const std::string& planner_task_append_topic_name,
    const std::string& planner_task_remove_topic_name,
    const std::string& planner_waypoints_server_name,
    const std::string& patrolling_task_append_topic_name,
    const std::string& patrolling_task_pause_topic_name,
    const std::string& robot_frame_name,
    const std::string& world_frame_name, 
    const std::string& robot_name
):WaypointsTool(pcl_input_topic_name, planner_task_feedback_topic_name, planner_task_append_topic_name, 
                planner_task_remove_topic_name, planner_waypoints_server_name,
                patrolling_task_append_topic_name,patrolling_task_pause_topic_name,
                robot_frame_name, world_frame_name, robot_name)
{
    shortcut_key_ = 'n';
}

}
