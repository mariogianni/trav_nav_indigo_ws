

#ifndef SIGNAL_UTILS_H
#define SIGNAL_UTILS_H

#include <math.h>
#include <stdlib.h>


///\def M_2PI
#ifndef M_2PI
#define M_2PI 6.28318530717959  // 2*M_PI
#endif


#define FEQUAL(a,b,tol) ( ( fabs(a-b) < (tol) ) ? (1) : (0) ) 

template<typename T>
inline T sat(T val, T min, T max)
{
    return std::min(std::max(val, min), max);
}

template<typename T>
inline int sign(T a)
{
    return ( (a >= 0) ? 1 : -1);
}

//returns the difference between ang1 [rad] and ang2 [rad] in the manifold S1 (unit circle)
//result is the representation of the angle with the smallest absolute value
inline double diffS1(double ang2, double ang1)
{
    double diff = fmod(ang2 - ang1, M_2PI); // now diff is in (-2*M_PI,2*M_PI)

    if (fabs(diff) <= M_PI)
        return diff;
    else
        return ( diff - sign(diff) * M_2PI);
}

//returns the positive distance between ang1 [rad] and ang2 [rad] in the manifold S1 (unit circle)
//result is smallest positive angle between ang1 and ang2
inline double distS1(double ang2, double ang1)
{
    double fabsdiff = fabs(fmod(ang2 - ang1, M_2PI)); // now fabsdiff is in (0,2*M_PI)
    return std::min(fabsdiff, M_2PI - fabsdiff);
}

inline double sinc(double x)
{
    if( FEQUAL(x,0,1e-4) == 1 )
        return 1;
    else
        return sin(x)/x;
}

// positive modulus: similar to matlab's mod(), result is always positive. not similar to fmod()
// es: posMod(-3,4)= 1   fmod(-3,4)= -3
//     posMod(-5,4)= 3   fmod(-5,4)= -1
inline double posMod(double x, double y)
{
	if (y == 0)	return x;
	
	return x - y * floor(x/y);
}

// wraps an angle [rad] so that it is contained in [-M_PI,M_PI)
inline double wrap(double ang)
{
	/*ang = fmod( ang + M_PI, M_2PI ) - M_PI;  // now ang is in (-3*M_PI,M_PI)
	return ( (ang >= -M_PI) ? ang : (ang + M_2PI)  );*/
	return posMod(ang + M_PI, M_2PI ) - M_PI;
}

#endif
