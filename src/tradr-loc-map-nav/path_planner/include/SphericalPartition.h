#ifndef PCL_FILTERS_SPHERICALPARTITION_H_
#define PCL_FILTERS_SPHERICALPARTITION_H_

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/make_shared.hpp>

#ifndef M_2PI
#define M_2PI (2 * M_PI)
#endif

inline size_t flatBucket(const size_t bucket1, const size_t bucket2, const size_t subdivisions)
{
	return bucket1 * subdivisions + bucket2;
}

inline void angleToBucket(double theta, double phi, size_t& bucket1, size_t& bucket2, const size_t subdivisions)
{
	// normalize angles between 0,2PI
	while(phi < 0) phi += 2 * M_PI;
	while(theta < 0) theta += 2 * M_PI;

	bucket1 = (size_t)(phi * subdivisions / M_2PI);
	bucket2 = (size_t)(theta * subdivisions / M_PI);
}

inline void bucketToAngle(const size_t bucket1, const size_t bucket2, double& theta, double& phi, const size_t subdivisions)
{
	phi = bucket1 * M_2PI / (double)subdivisions;
	theta = bucket2 * M_PI / (double)subdivisions;
}

template<typename T>
inline void coordsSphericalToEuclidean(double r, double theta, double phi, T& x, T& y, T& z, double offsetX = 0, double offsetY = 0, double offsetZ = 0)
{
	x = offsetX + r * sin(theta) * cos(phi);
	y = offsetY + r * sin(theta) * sin(phi);
	z = offsetZ + r * cos(theta);
}

inline void coordsEuclideanToSpherical(double x, double y, double z, double& r, double& theta, double& phi, double offsetX = 0, double offsetY = 0, double offsetZ = 0)
{
	// radius [0..infty):
	r = sqrt(pow(x - offsetX, 2) + pow(y - offsetY, 2) + pow(z - offsetZ, 2));
	// inclination [0..PI]:
	theta = acos((z - offsetZ) / r);
	//  azimuth [0..2PI):
	phi = atan2(y - offsetY, x - offsetX);
}

template<typename PointT>
void sphericalPartition(const pcl::PointCloud<PointT>& input, pcl::PointCloud<PointT> *output, const pcl::PointXYZ& center, unsigned int subdivisions)
{
	// partition according to phi,theta:
	for(size_t i = 0; i < input.size(); i++)
	{
		const PointT& p = input.points[i];
		double r, theta, phi;
		coordsEuclideanToSpherical(p.x, p.y, p.z, r, theta, phi, center.x, center.y, center.z);
		size_t bucket1 = 0, bucket2 = 0;
		angleToBucket(theta, phi, bucket1, bucket2, subdivisions);
		output[flatBucket(bucket1, bucket2, subdivisions)].push_back(p);
	}
}

#endif // PCL_FILTERS_SPHERICALPARTITION_H_
