#ifndef POLYNOMIAL_INTERPOLATOR_H
#define POLYNOMIAL_INTERPOLATOR_H

#include <vector>

std::vector<double> interpolation_values(int order, int derive);

std::vector<double> define_coefficients(int order);

double interpolator (std::vector<double> coefficients, double temps, int derive);

#endif // POLYNOMIAL_INTERPOLATOR_H
