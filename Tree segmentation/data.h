#ifndef _DATA_H
#define _DATA_H
//
//						 data.h
//

/************************************************************************
			DECLARATION OF THE CLASS DATA
************************************************************************/
// Class for Data
// A data has 5 fields: 
//       n (of type int), the number of data points 
//       X and Y (arrays of type double), arrays of x- and y-coordinates
//       meanX and meanY (of type double), coordinates of the centroid (x and y sample means)
#include <stdio.h>
#include <vector>
class Data
{
public:

	int n;
	double *X;		//space is allocated in the constructors
	double *Y;		//space is allocated in the constructors
	double meanX, meanY;

	// constructors
	Data()
	{
		n=0;
		X = new double[n];
		Y = new double[n];
		for (int i=0; i<n; i++)
		{
			X[i]=0.;
			Y[i]=0.;
		}
	}
	Data(int N)
	{
		n=N;
		X = new double[n];
		Y = new double[n];

		for (int i=0; i<n; i++)
		{
			X[i]=0.;
			Y[i]=0.;
		}
	}
	Data(std::vector<double>&XX,std::vector<double>&YY)
	{
		n=XX.size();
		X = new double[n];
		Y = new double[n];

		for (int i=0; i<n; i++)
		{
			X[i]=XX[i];
			Y[i]=YY[i];
		}
	}

	// routines
	//void means(void);
	//void center(void);
	//void scale(void);
	//void print(void);

	//// destructors
	//~Data();

	void means(void)
	{
		meanX=0.; meanY=0.;
	
		for (int i=0; i<n; i++)
		{
			meanX += X[i];
			meanY += Y[i];
		}
		meanX /= n;
		meanY /= n;
	}

	// Routine that centers the data set (shifts the coordinates to the centeroid)

	void center(void)
	{
		double sX=0.,sY=0.;  
		int i;
	
		for (i=0; i<n; i++)
		{
			sX += X[i];
			sY += Y[i];
		}
		sX /= n;
		sY /= n;
	
		for (i=0; i<n; i++)
		{
			X[i] -= sX;
			Y[i] -= sY;
		}
		meanX = 0.;
		meanY = 0.;
	}

	// Routine that scales the coordinates (makes them of order one)

	void scale(void)
	{
		double sXX=0.,sYY=0.,scaling;  
		int i;
	
		for (i=0; i<n; i++)
		{
			sXX += X[i]*X[i];
			sYY += Y[i]*Y[i];
		}
		scaling = sqrt((sXX+sYY)/n/2.0);
	
		for (i=0; i<n; i++)
		{
			X[i] /= scaling;
			Y[i] /= scaling;
		}
	}

	// Printing routine

	//void print(void)
	//{
	//	cout << endl << "The data set has " << n << " points with coordinates :"<< endl;
	//
	//	for (int i=0; i<n-1; i++) cout << setprecision(7) << "(" << X[i] << ","<< Y[i] << "), ";
	//
	//	cout << "(" << X[n-1] << ","<< Y[n-1] << ")\n";
	//}

	// Destructor
	~Data()
	{
		delete[] X;
		delete[] Y;
	}
};


/************************************************************************
			BODY OF THE MEMBER ROUTINES
************************************************************************/


// Routine that computes the x- and y- sample means (the coordinates of the centeroid)

#endif


