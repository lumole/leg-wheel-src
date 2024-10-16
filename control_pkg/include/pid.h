#ifndef PID_H
#define PID_H
#include <stdio.h>
#include <iostream>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include "message/pid_state.h"
using namespace std;
using std::cin;
using std::cout;
using std::endl;
using std::string;
class PID
{
public:
    PID(double _timeStep, double index[3], double _I_limit, double _output_limit, double _exp)
    {
        kp = index[0];
        ki = index[1];
        kd = index[2];
        I_limit = _I_limit;
        output_limit = _output_limit;
        exp = _exp;
    }
    void update(double _feedback);
    void index_adjust(double _kp,double _ki,double _kd);
    void exp_set(double _exp);
    double control(void);
    message::pid_state print(void);
    message::pid_state pid_msg;
private:
    double timeStep;
    double output = 0;
    double err[3] = {0};
    double kp, ki, kd;
    double I = 0;
    double I_limit;
    double output_limit;
    double exp;
    double fdb;
};
#endif