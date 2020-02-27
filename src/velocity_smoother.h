#include <math.h>
#include<ros/ros.h>

/*
Class to smooth the input x velocity. The velocity 
decays as a function of angle error with the target goal
*/


class VelocitySmoother{
    public:

    VelocitySmoother(double maximum_vel , double error_tolerance, double k){

        this->maximum_vel = maximum_vel;
        this->error_tolerance = error_tolerance;
        this->k = k;
    }
    double smooth_velocity(double error)
    {
        error = (180/3.1415)*error;
        if ((abs(error) >= 30))
        {
            return 0;
        }
        return (maximum_vel * std::exp(k * (-error + error_tolerance))/(1 + std::exp(k * (-error + error_tolerance)))); 
    }


    private:
    double maximum_vel; 
    double error_tolerance;
    double k;
};