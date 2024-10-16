#include "keyboard.h"
using namespace std;

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");                              // 调整编码，可以显示中文
    ros::init(argc, argv, "key_parameter_tuning_node"); // 初始化
    ros::NodeHandle n;
    keyboard ky;
    ky.InitializePublishers();
    while (ros::ok())
    {
        ky.switchkeyboard();
        
    }
    return 0;
}