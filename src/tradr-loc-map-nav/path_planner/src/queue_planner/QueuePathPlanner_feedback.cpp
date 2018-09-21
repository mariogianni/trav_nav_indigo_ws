#include "QueuePathPlanner.h"

//#define CANCEL_EXECUTED_PATH 1

void QueuePathPlanner::feedbackCallback(const trajectory_control_msgs::PlanningFeedback& feedback_msg)
{
    
    std::cout << "QueuePathPlanner::feedbackCallback()" << std::endl;
        
    // if this message have been published by the controller node, then
    if( !feedback_msg.node.compare("control") )
    {
	// controller busy? 
	if( !feedback_msg.task.compare("start") )
	{
	    controller_ready_flag_=false;

	    // stop timer for tasks processing
	    /// < task_timer_.stop();
	}
	else // we are receiving a stop or abort
        //if( !feedback_msg.task.compare("done") )
	{
	    controller_ready_flag_=true;

	    // (re)start timer for task's processing
	    /// < task_timer_.start();
            
#ifdef CANCEL_EXECUTED_PATH
            // init empty path message
            nav_msgs::Path path_msg;
            path_msg.header.stamp=ros::Time::now();
            path_msg.header.frame_id=reference_frame_;   
            // publish the empty path
            boost::recursive_mutex::scoped_lock locker(task_path_pub_mutex_); 
	    task_path_pub_.publish(path_msg);
#endif
        
	}
    }
}
