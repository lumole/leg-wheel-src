#include "controller.h"

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");// 调整编码，可以显示中文
    ros::init(argc, argv, "LQR_VMC_Controller"); // 初始化
    VMC *cf=new VMC();
    cf->run();
    return 0;
}
