#include "interface.h"
#include <iostream>
using namespace std;

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");              // 调整编码，可以显示中文
    ros::init(argc, argv, "interface"); // 初始化
    ros::NodeHandle n;
    Interface *can= new Interface();

    bool isInit = false;
    n.getParam("transmit_receive_node/isInit", isInit);
    if (isInit)
    {
        /* code */
        can->Robot_Init(0);
    }
    ros::Rate motor_rate(800);
    ros::spin();
    return 0;
}