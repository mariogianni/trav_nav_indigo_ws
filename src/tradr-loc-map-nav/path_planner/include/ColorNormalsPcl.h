#ifndef COLOR_NORMALS_PCL_H_
#define COLOR_NORMALS_PCL_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void colorNormalsPCL(pcl::PointCloud<pcl::PointXYZRGBNormal>& output)
{
	for(size_t i = 0; i < output.size(); i++)
	{
#if 1
		// r should be 1, but anyways...
		double r = sqrt(pow(output[i].normal_x, 2) + pow(output[i].normal_y, 2) + pow(output[i].normal_z, 2));
		double phi = atan2(output[i].normal_y, output[i].normal_x);
		double theta = acos(output[i].normal_z / r);

		// normalize angles between 0,2PI
		while(phi < 0) phi += 2 * M_PI;
		while(theta < 0) theta += 2 * M_PI;

		if(r > 1e-6)
		{
			output[i].r = (uint8_t)((0.5 + 0.5 * sin(phi)) * 255.0);
			output[i].g = 200;
			output[i].b = (uint8_t)(theta * 255.0 / M_PI);
		}
		else
		{
			output[i].r = 255;
			output[i].g = 0;
			output[i].b = 255;
		}
#else
		double k = 1.0;
		if(output[i].normal_z < 0) k = -1.0;
		double phi = atan2(k * output[i].normal_y, k * output[i].normal_x);
		output[i].r = (uint8_t)((0.5 + 0.5 * sin(M_PI * k * output[i].normal_z)) * 255.0);
		output[i].g = (uint8_t)((0.5 + 0.5 * sin(2 * phi)) * 255.0);
		output[i].b = 255;
#endif
	}
}

#endif // COLOR_NORMALS_PCL_H_
