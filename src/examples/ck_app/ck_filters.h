#ifndef CK_FILTERS_H
#define CK_FILTERS_H

#include <vector>
using std::vector;

namespace CK_Fun {
double lowPassFilter(double dt, double tconst, double uk, double ykm1);
vector<double> CK_Fun::unitCrossProduct(vector<double> x, vector<double> y);
}

#endif
