#ifndef _CIRCLE_FIT_BY_TAUBIN
#define _CIRCLE_FIT_BY_TAUBIN
//#include "mystuff.h"
#include "data.h"
#include "circle.h"
#include <stdio.h>      /* printf */
#include <math.h>
//#include "Utilities.cpp"
typedef double reals;       //  defines reals as double (standard for scientific calculations)
//typedef long double reals;  //  defines reals as long double 

//   Note: long double is an 80-bit format (more accurate, but more memory demanding and slower)

typedef long long integers;

//   next define some frequently used constants:

const reals One=1.0,Two=2.0,Three=3.0,Four=4.0,Five=5.0,Six=6.0,Ten=10.0;
reals Sigma (Data& data, Circle& circle);
Circle CircleFitByTaubin (Data& data);
//const reals One=1.0L,Two=2.0L,Three=3.0L,Four=4.0L,Five=5.0L,Six=6.0L,Ten=10.0L;
//const reals Pi=3.141592653589793238462643383L;

#endif
