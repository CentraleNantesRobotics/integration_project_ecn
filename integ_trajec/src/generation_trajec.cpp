#include <vector>
#include <cmath>

std::vector<double> interpolation_values(int order, int derive)
{
    std::vector<double> coefficients = define_coefficients(order);

    std::vector<double> interpolation_values;

    int normilized_temps = 0;

    while (normilized_temps =< 1)
    {
        r = interpolator(coefficients,normilized_temps,derive);
        interpolation_values.push_back(r);
        normilized_temps = normilized_temps + 0.01;
    }
}

std::vector<double> define_coefficients(int order)
{
    std::vector<double> coefficients(6,0);
    switch (order)
    {
    case 1:
        coefficients[1] = 1;

    case 3:
        coefficients[2] = 3;
        coefficients[3] = -2;

    case 5:
        coefficients[3] = 10;
        coefficients[4] = -15;
        coefficients[5] = 6;
    }
    return (coefficients);
}

double interpolator (std::vector<double> coefficients, double temps, int derive)
{
    double r = 0;

    switch (derive)
    {
    case 0:
        for (int i = 0; i < coefficients.size(); i++)
        {
            r += coefficients[i]*std::pow(temps,i);
        }

    case 1:
        for (int i = 1; i < coefficients.size(); i++)
        {
            r += i*coefficients[i]*std::pow(temps,i-1);
        }

    case 2:
        for (int i = 2; i < coefficients.size(); i++)
        {
            r += i*(i-1)*coefficients[i]*std::pow(temps,i-2);
        }
    }
    return(r);
}
