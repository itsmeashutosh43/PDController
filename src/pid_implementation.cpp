/*
Implementation of PID Controller.
*/


#include <iostream>
#include <cmath>
#include "pid_implementation.h"

using namespace std;



PD::PD( double dt, double Kp, double Kd, double Ki )
{
    _dt = dt;
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
}


double PD::calculate( double error, double max_ )
{

    
    // Proportional portion
    double Pout = _Kp * error;

    /*
    // Integral portion
    _integral += error * _dt;
    double Iout = _Ki * _integral;
    */

    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;
    // Total output
    double output = Pout + Dout;


    if ((output > max_))
    {
        output = max_;
    }

    else if((output < 0) && (output < -max_))
    {
        output = -max_;
    }

    // Save error to previous error
    _pre_error = error;

    return output;
}
