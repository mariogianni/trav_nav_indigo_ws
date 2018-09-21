#include "QueuePathPlanner.h"
#include <signal.h>

void mySigintHandler(int signum)
{
    std::cout << "mySigintHandler()" << std::endl;
    //boost::recursive_mutex::scoped_lock destroy_locker(destroy_mutex); 

    //p_planner.reset();
    //p_marker_controller.reset(); 

    ros::shutdown();
    exit(signum);
}

int main(int argc, char *argv[])
{
    // ros initialization
    ros::init(argc,argv,"queue_path_planner");
    
    //override default sigint handler 
    signal(SIGINT, mySigintHandler);
    
    // queue planner
    QueuePathPlanner planner;

    // ros loop
    //ros::spin();
    
    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown

    return 0;
}
