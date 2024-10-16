#include "controller.h"

double LIMIT(double var, double max) // 限幅函数
{
    double output = 0;
    if (var > max)
    {
        output = max;
    }
    if (var < -max)
    {
        output = -max;
    }
    if (var >= -max && var <= max)
    {
        output = var;
    }
    return output;
}

int16_t T2iqControl_MF7015(double T) // 7015 力矩➡电流
{
    if (T > 0)
    {
        return -30.591 * T * T * T * T * T * T + 236.37 * T * T * T * T * T - 700.31 * T * T * T * T + 1014.5 * T * T * T - 712.83 * T * T + 835.07 * T + 1.2362;
    }
    else
    {
        T *= (-1);
        return -(-30.591 * T * T * T * T * T * T + 236.37 * T * T * T * T * T - 700.31 * T * T * T * T + 1014.5 * T * T * T - 712.83 * T * T + 835.07 * T + 1.2362);
    }
}

int16_t T2iqControl_MG6012(double T) // 髋 力矩➡电流
{
    if (T > 0)
    {
        return 0.0011 * T * T * T * T * T * T - 0.0469 * T * T * T * T * T + 0.7035 * T * T * T * T - 4.4394 * T * T * T + 10.647 * T * T + 49.56 * T + 4.06;
    }
    else
    {
        T *= (-1);
        return -(0.0011 * T * T * T * T * T * T - 0.0469 * T * T * T * T * T + 0.7035 * T * T * T * T - 4.4394 * T * T * T + 10.647 * T * T + 49.56 * T + 4.06);
    }
}

int16_t T2iqControl_MF9015(double T) // 9015 力矩➡电流
{
    if (T > 0)
    {
        return 2.0415 * T * T * T * T * T * T - 17.93 * T * T * T * T * T + 73.153 * T * T * T * T - 175.09 * T * T * T + 209.86 * T * T + 212.18 * T + 4.1652;
    }
    else
    {
        T *= (-1);
        return -(2.0415 * T * T * T * T * T * T - 17.93 * T * T * T * T * T + 73.153 * T * T * T * T - 175.09 * T * T * T + 209.86 * T * T + 212.18 * T + 4.1652);
    }
}

void swap(double *a, double *b)
{
    double temp = *a;
    *a = *b;
    *b = temp;
}

// 实现中值滤波
double mid_filt(double data[], int size)
{
    double temp[size];

    // 复制数组
    for (int i = 0; i < size; i++)
        temp[i] = data[i];

    // 冒泡排序
    for (int i = 0; i < size - 1; i++)
        for (int j = 0; j < size - 1 - i; j++)
            if (temp[j] > temp[j + 1])
                swap(&temp[j], &temp[j + 1]);

    // 基数返回中位数
    if (size % 2 == 1)
        return temp[size / 2];
    // 偶数返回中间两个中的平均值
    else
        return (temp[size / 2] + temp[(size / 2) - 1]) / 2;
}

filter::filter()
{
    series[0]=0;
    series[1]=0;
    series[2]=0;
    series[3]=0;
    series[4]=0;
}
double filter::update(double _update)
{
    series[4]=series[3];
    series[3]=series[2];
    series[2]=series[1];
    series[1]=series[0];
    series[0]=_update;
    return mid_filt(series,5);
}

void VMC::L0_pid_correct(Leg_Model *leg)
{
    double P = 0;
    double cos_theta = 0;
    double sin_theta = 0;
    double F = 0;
    double Tp = 0;
    double L0 = 0;
    double dL0 = 0;
    double ddL0 = 0;
    double theta = 0;
    double dtheta = 0;
    double ddtheta = 0;
    double ddzw = 0;
    double ddzm = 0;
    double r, p;
    r = roll * pi / 180;
    p = pitch * pi / 180;
    theta = leg->theta;
    dtheta = leg->dTheta->now;
    ddtheta = leg->dTheta->d;
    F = leg->F;
    Tp = leg->Tp;
    L0 = leg->L0;
    dL0 = leg->dL0->now;
    ddL0 = leg->dL0->d;
    cos_theta = std::cos(theta);
    sin_theta = std::sin(theta);
    ddzm = -ddx * sin(p) + ddy * cos(p) * sin(r) + ddz * cos(r) * cos(p) - g;
    ddzw = ddzm - ddL0 * cos_theta + 2 * dL0 * dtheta * sin_theta + L0 * ddtheta * sin_theta + L0 * dtheta * dtheta * cos_theta;
    leg->F = (leg->F-m_wheel * g - m_wheel * ddzw-Tp * sin_theta / L0)/cos_theta;

}

double VMC::adjust_dx_exp(double phi, double dx_exp) {
    // 定义参数
    static const double phi_threshold = 0.2;
    static const double factor_at_threshold = 0.4;

    // 计算系数 a, 使得 |phi| = 0.25 时，f(phi) = 0.4
    static const double a = (factor_at_threshold - 1) / (phi_threshold * phi_threshold);
    static double phi0 = -0.1;
    // 计算调整因子 f(phi)
    double adjustment_factor = a * pow((phi-phi0), 2) + 1;

    // 计算新的 dx_exp
    return dx_exp * adjustment_factor;
}
