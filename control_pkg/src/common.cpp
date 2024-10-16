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
