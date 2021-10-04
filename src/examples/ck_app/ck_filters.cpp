#include <vector>
#include <math.h>
#include "ck_filters.h"

using std::vector;

double CK_Fun::lowPassFilter(double dt, double tconst, double uk, double ykm1) {
// First order low pass filter
// Rule of thumb: tconst >= 5*dt
// large time constant = "fast reponse"
// uk is input at time k
// ykm1 is y_(k-1), the value at time k-1

double a = dt/(tconst + dt);
return (1 - a)*ykm1 + a*uk;

}

vector<double> CK_Fun::unitCrossProduct(vector<double> x, vector<double> y) {
	// x and y are vectors in R^3
	// returns a unit vector
	double s1 = x[1]*y[2] - x[2]*y[1];
	double s2 = x[2]*y[0] - x[0]*y[2];
	double s3 = x[0]*y[1] - x[1]*y[0];

	double norm = sqrt(s1*s1 + s2*s2 + s3*s3);

	return {s1 / norm, s2 / norm, s3 / norm};

}
