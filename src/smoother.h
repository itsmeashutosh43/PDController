#include <math.h>
#include<ros/ros.h>

/*
Class to smooth the input x velocity. The velocity 
decays as a function of angle error with the target goal
*/


class _Smoother{
    public:

    _Smoother(){

    }
    double smooth_velocity(double maximum_vel , double error_tolerance, double k,double error)
    {
        error = (180/3.1415)*error;
        if ((abs(error) >= error_tolerance))
        {
            return 0;
        }
        return maximum_vel * std::exp(k * (-error + error_tolerance))/(1 + std::exp(k * (-error + error_tolerance)));
    }

    double smoother_funct(double maximum_vel , double error_tolerance, double k,double error)
    {
        double x =  maximum_vel * std::exp(k * (-error + error_tolerance))/(1 + std::exp(k * (-error + error_tolerance)));

        if (x < 0.4)
        {
            return 0.4;
        }
        return x;
        
    }

    private:
    double maximum_vel; 
    double error_tolerance;
    double k;
};