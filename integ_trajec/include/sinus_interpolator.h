#include <cmath>
#include <vector>

std::vector<double> sinus_interpolator (int harmonic, double x_init, double x_final, double work_time, int derive)
{
    std::vector<double> sinus_waypoints(100,0);
    double t = 0;

    while (t <= work_time)
    {
        if (derive == 0)
        {
           sinus_waypoints.push_back(-x_final*std::cos(harmonic*t/work_time*2*M_PI) + x_init);

        }
        if (derive == 1)
        {
            sinus_waypoints.push_back(2*M_PI*x_final*harmonic*std::sin(harmonic*t/work_time*2*M_PI)/work_time);

        }
        if (derive == 2)
        {
            sinus_waypoints.push_back(4*M_PI*M_PI*x_final*harmonic*harmonic*std::cos(harmonic*t/work_time*2*M_PI)/work_time/work_time);

        }
        t = t + (work_time/100);

    }
    return (sinus_waypoints);
}

