#ifndef COPY_POINT_H
#define COPY_POINT_H

#include <pcl/point_types.h>

#define COPY_XY(p,q)    {q.x = p.x; q.y = p.y;}
#define COPY_XYZ(p,q)    {q.x = p.x; q.y = p.y; q.z = p.z;}
#define COPY_HSV(p,q)    {q.h = p.h; q.s = p.s; q.v = p.v;}
#define COPY_I(p,q)      {p.intensity = p.intensity;}
#define COPY_A(p,q)      {p.a = p.a;}
#define COPY_L(p,q)      {p.label = p.label;}
#define COPY_RGB(p,q)    {q.r = p.r; q.g = p.g; q.b = p.b;}
#define COPY_Normal(p,q) {q.normal_x = p.normal_x; q.normal_y = p.normal_y; q.normal_z = p.normal_z;}

template<typename PointT>
void copyPoint(const pcl::PointXY& p, PointT& q)
{
	COPY_XY(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZ& p, PointT& q)
{
	COPY_XYZ(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZHSV& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_HSV(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZI& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_I(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZINormal& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_I(p, q);
	COPY_Normal(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZL& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_L(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZRGBA& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_RGB(p, q);
	COPY_A(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZRGBL& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_RGB(p, q);
	COPY_L(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZRGB& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_RGB(p, q);
}

template<typename PointT>
void copyPoint(const pcl::PointXYZRGBNormal& p, PointT& q)
{
	COPY_XYZ(p, q);
	COPY_RGB(p, q);
	COPY_Normal(p, q);
}

#endif // COPY_POINT_H
