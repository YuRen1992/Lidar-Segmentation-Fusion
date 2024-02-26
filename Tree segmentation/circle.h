//
//						 circle.h
//
/************************************************************************
			DECLARATION OF THE CLASS CIRCLE
************************************************************************/
// Class for Circle
// A circle has 7 fields: 
//     a, b, r (of type reals), the circle parameters
//     s (of type reals), the estimate of sigma (standard deviation)
//     g (of type reals), the norm of the gradient of the objective function
//     i and j (of type int), the iteration counters (outer and inner, respectively)
#ifndef _CIRCLE_H
#define _CIRCLE_H
class Circle
{
public:

	// The fields of a Circle
	double a, b, r, s, g, Gx, Gy;
	int i, j;

	// constructors
	Circle();
	Circle(double aa, double bb, double rr);

	// routines
	void print(void);

	// no destructor we didn't allocate memory by hand.
};


/************************************************************************
			BODY OF THE MEMBER ROUTINES
************************************************************************/
// Default constructor


// Printing routine

//void Circle::print(void)
//{
//	cout << endl;
//	cout << setprecision(10) << "center (" <<a <<","<< b <<")  radius "
//		 << r << "  sigma " << s << "  gradient " << g << "  iter "<< i << "  inner " << j << endl;
//}

#endif
