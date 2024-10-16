
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

double filter_update(double serie[],double _new)
{
    serie[2]=serie[1];
    serie[1]=serie[0];
    serie[0]=_new;
    return mid_filt(serie, 3);
}