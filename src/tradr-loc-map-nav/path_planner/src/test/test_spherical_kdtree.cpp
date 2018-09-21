#include <DynamicJoinPcl.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>

template<typename T>
struct Accumulator { typedef T Type; };
template<>
struct Accumulator<unsigned char>  { typedef float Type; };
template<>
struct Accumulator<unsigned short> { typedef float Type; };
template<>
struct Accumulator<unsigned int> { typedef float Type; };
template<>
struct Accumulator<char>   { typedef float Type; };
template<>
struct Accumulator<short>  { typedef float Type; };
template<>
struct Accumulator<int> { typedef float Type; };

template<class T>
class AngularDistance {
    typedef T ElementType;
    typedef typename Accumulator<T>::Type ResultType;

    template <typename Iterator1, typename Iterator2>
    ResultType operator()(Iterator1 a, Iterator2 b, size_t size, ResultType worst_dist = -1) const
    {
        ResultType result = ResultType();
        ResultType diff;
        for(size_t i = 0; i < size; ++i ) {
            diff = fmod(*a++ - *b++ + 2*M_PI, 2*M_PI);
            result += diff*diff;
        }
        return result;
    }
    
    template <typename U, typename V>
    inline ResultType accum_dist(const U& a, const V& b, int dim) const
    {
        return fmod(*a++ - *b++ + 2*M_PI, 2*M_PI);
    }
};

int main(int argc, char **argv) {
/*
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for(double a = 0; a <= 2*M_PI; a += 0.01) {
        pcl::PointXYZ p;
        p.x = 0;
        p.y = 0;
        p.z = a;
        cloud.push_back(p);
    }
    
    pcl::KdTreeFLANN<pcl::PointXYZ, AngularDistance<float> > kdtree;
    kdtree.setInputCloud(cloud.makeShared());

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    pcl::PointXYZ center;
    center.x = 0;
    center.y = 0;
    center.z = 0;

    kdtree.radiusSearch(center, 0.2, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    std::cout << "Points:" << std::endl;
    for(std::vector<int>::iterator it = pointIdxRadiusSearch.begin(); it != pointIdxRadiusSearch.end(); it++) {
        std::cout << cloud.points[*it].z << std::endl;
    }
*/
	return 0;
}
