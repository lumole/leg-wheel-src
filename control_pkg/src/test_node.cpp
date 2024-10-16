#include "ros/ros.h"
using namespace std;
constexpr int Control_Rate = 250;
constexpr double Normal_loop_time = 1 / (double)Control_Rate * 1000;
int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");              // 调整编码，可以显示中文
    ros::init(argc, argv, "test_node"); // 初始化
    ros::NodeHandle nh;
    double loop_time = 4;
    double err_time = 0;
    double ERR_time = 0;
    double control_time = 0;

    int loop_count = 0;
    int err_count = 0;
    int ERR_count = 0;

    double startTime_control;
    double endTime_control;
    double endTime_T;
    bool shut=false;
    ros::Rate rate(Control_Rate);
    while (ros::ok())
    {
        startTime_control = ros::Time::now().toNSec(); // 控制周期开始
        loop_count++;
        if (loop_time > Normal_loop_time * 1.2)
        {
            err_count++;
            err_time = loop_time;
        }
        if (loop_time > Normal_loop_time * 10||loop_time < Normal_loop_time * 0.1)
        {
            ERR_count++;
            ERR_time = loop_time;
            shut=true;
        }
        cout << "循环次数：" << loop_count << endl;
        cout << "错误次数：" << err_count << endl;
        cout << "上次死机时间:" << err_time << "ms" << endl;
        cout << "危险错误：" << ERR_count << "时长：" << ERR_time << endl;
        endTime_control = ros::Time::now().toNSec();
        control_time = (endTime_control - startTime_control) / 1e6;
        std::cout << "控制计算用时:" << control_time << "ms" << std::endl; // 控制计算结束

        rate.sleep(); // 周期等待
        endTime_T = ros::Time::now().toNSec();
        loop_time = (endTime_T - startTime_control) / 1e6;
        std::cout << "控制周期:" << loop_time << "ms" << std::endl;
        std::cout << endl;
        if (shut)
        {
            ros::shutdown();
        }
        
    }
    return 0;
}
