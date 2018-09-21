
#include <math.h> 
#include <TrajectoryControlActionServer.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, ros::this_node::getName());

	TrajectoryControlActionServer action_server(ros::this_node::getName());

	ros::spin();

	return 0;
}
