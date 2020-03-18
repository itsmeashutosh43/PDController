
class PD
{
public:
    PD( double dt,double Kp, double Kd, double Ki );
    
    double calculate( double error, double vel);

private:
    double _dt;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;
};
