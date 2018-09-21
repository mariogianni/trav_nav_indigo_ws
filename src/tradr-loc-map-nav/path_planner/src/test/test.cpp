#include <ros/ros.h>
#include <DynamicJoinPcl.h>

ros::Publisher *pclpub_ptr;
pcl::PointCloud<pcl::PointXYZ> cloud;

double x, y, z;

void make_pcl(pcl::PointCloud<pcl::PointXYZ>& cloud, double alpha_min, double alpha_max, int alpha_points, double beta_min, double beta_max, int beta_points, double alpha1, double beta1, double radius, double distance_near, double distance_far) {
    cloud.clear();
    double alpha_step = (alpha_max - alpha_min) / double(alpha_points);
    double beta_step = (beta_max - beta_min) / double(beta_points);
    for(double alpha = alpha_min; alpha <= alpha_max; alpha += alpha_step) {
        for(double beta = beta_min; beta <= beta_max; beta += beta_step) {
            pcl::PointXYZ point;
            if(sqrt(pow(tan(alpha) - tan(alpha1), 2) + pow(tan(beta) - tan(beta1), 2)) <= radius)
            {
            	// inside the disc
                point.x = x + distance_near;
                point.y = y + distance_near * tan(alpha);
                point.z = z + distance_near * tan(beta);
            }
            else {
				double distance = distance_far + 0.8 * sin(2 * alpha + 3 * beta);
				point.x = x + distance;
				point.y = y + distance * tan(alpha);
				point.z = z + distance * tan(beta);
            }
            cloud.push_back(point);
        }
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test");

    x = 1.2;
    y = 0.6;
    z = 3.2;

	ros::NodeHandle n("~");

    std::string laser_frame, global_frame, odom_frame;
    n.param<std::string>("laser_frame", laser_frame, "/laser");
    n.param<std::string>("global_frame", global_frame, "/map");
    n.param<std::string>("odom_frame", odom_frame, "/base_link" );

	ros::Publisher pclpub = n.advertise<sensor_msgs::PointCloud2>("/dynamic_point_cloud", 1, true);

	pclpub_ptr = &pclpub;

    int den = 6;
    double alpha_min = -M_PI / 2.0;
    double alpha_max = M_PI / 2.0;
    int alpha_points = den * 40;
    double beta_min = -M_PI / 2.0;
    double beta_max = M_PI / 2.0;
    int beta_points = den * 30;
    double alpha1 = 0;
    double beta1 = 0;
    double radius = M_PI * 0.1;
    double distance_near = 2.0;
    double distance_far = 3.0;
    double t = 0;

    ros::Rate r(10);
	while(ros::ok()) {
        t += 0.05;
        alpha1 = cos(t);
        beta1 = sin(t);
        //distance_near = 0.3 * sin(t * 0.25) + 1.4;
        make_pcl(cloud, alpha_min, alpha_max, alpha_points, beta_min, beta_max, beta_points, alpha1, beta1, radius, distance_near, distance_far);
        cloud.header.frame_id = laser_frame;
        cloud.header.stamp = ros::Time::now().toNSec() * 1e-3;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        pclpub.publish(cloud_msg);
        ros::spinOnce();
        r.sleep();
    }
	return 0;
}

