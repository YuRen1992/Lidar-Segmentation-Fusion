#include "circle.h"

Circle::Circle()
{
	a=0.; b=0.; r=1.; s=0.; i=0; j=0;
}

// Constructor with assignment of the circle parameters only

Circle::Circle(double aa, double bb, double rr)
{
	a=aa; b=bb; r=rr;
}