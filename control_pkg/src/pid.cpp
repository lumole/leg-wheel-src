#include "pid.h"
void PID::update(double feedback)
{
    fdb=feedback;
    err[2] = err[1];
    err[1] = err[0];
    err[0] = exp-fdb;
    // I += err[0];
    // if( I > I_limit || I < -I_limit )//dh_test
    // {
    //      output += kp * (err[0] - err[1]) + kd * (err[0] - 2 * err[1] + err[2]);
    // }
    // else
    // {
    //     I += err[0];
    //     output += kp * (err[0] - err[1]) + ki * err[0] + kd * (err[0] - 2 * err[1] + err[2]);
    // }
    output += kp * (err[0] - err[1]) + ki * err[0] + kd * (err[0] - 2 * err[1] + err[2]);
}
message::pid_state PID::print()
{
    std::cout<<"[PID测试:]"<<endl;
    std::cout<<"kp= "<<kp<<" ,ki= "<<ki<<" ,kd= "<<kd<<endl;
    std::cout<<"exp= "<<exp<<",fdb= "<<fdb<<",err="<<err[0]<<endl;
    std::cout<<"output= "<<output<<endl;
    pid_msg.output=output;
    pid_msg.fdb=fdb;
    pid_msg.err=err[0];
    pid_msg.exp=exp;
    pid_msg.i=I;
    pid_msg.header.stamp = ros::Time::now();
    return pid_msg;
}
void PID::index_adjust(double _kp,double _ki,double _kd)
{
    kp+=_kp;
    ki+=_ki;
    kd+=_kd;
}
void PID::exp_set(double _exp)
{
    exp = _exp;
}

double PID::control(void)
{
    double control = output;
    if (output > output_limit)
    {
        control = output_limit;
    }
    if (output < -output_limit)
    {
        control = -output_limit;
    }
    return control;
}
