#ifndef TIMEVAR_H
#define TIMEVAR_H
#include <stdio.h>
#include <iostream>
using namespace std;
using std::cin;
using std::cout;
using std::endl;
using std::string;
class TimeVar
{
private:
    double Dt;//时间间隔
    double DTS[3];//时间序列
public:
    TimeVar(double timeStep);
    void update(double in);
    void clear_i(void);//积分清除
    double now;//变量的当前值
    double d;//变量导数
    double last_d;//变量的上一个导数
    double dd;//变量的二阶导数
    double i;//变量的积分
};
#endif
