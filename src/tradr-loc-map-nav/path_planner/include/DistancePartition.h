#ifndef PCL_FILTERS_DISTANCEPARTITION_H_
#define PCL_FILTERS_DISTANCEPARTITION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template<typename PointA, typename PointB>
inline double dist(const PointA& a, const PointB& b)
{
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

template<typename PointT>
void distancePartition(const pcl::PointCloud<PointT>& input, pcl::PointCloud<PointT>& output_in, pcl::PointCloud<PointT>& output_out, const pcl::PointXYZ& center, double distance)
{
	for(size_t i = 0; i < input.size(); i++)
	{
		if(dist(center, input.points[i]) <= distance)
		{
			output_in.push_back(input.points[i]);
		}
		else
		{
			output_out.push_back(input.points[i]);
		}
	}
}

#endif // PCL_FILTERS_DISTANCEPARTITION_H_
