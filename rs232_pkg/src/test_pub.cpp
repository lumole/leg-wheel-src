#include <ros/ros.h>
#include <message/rs232_elec_mag_ctrl.h>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_publisher_node");
    ros::NodeHandle nh;

    // 创建一个Publisher，发布到rs232_elec_mag_ctrl话题
    ros::Publisher pub = nh.advertise<message::rs232_elec_mag_ctrl>("rs232_elec_mag_ctrl", 10);

    // 声明一个消息对象
    message::rs232_elec_mag_ctrl msg;

    // 设置循环的频率
    ros::Rate rate(10);  // 发布频率为10Hz

    while (ros::ok())
    {
        int input_number;
        std::cout << "Enter a number between 0 and 15: ";
        std::cin >> input_number;

        // 确保输入数字在合理范围内
        if (input_number < 0 || input_number > 15)
        {
            std::cout << "Invalid input. Please enter a number between 0 and 15." << std::endl;
            continue;
        }

        // 计算四个控制信号
        msg.header.stamp = ros::Time::now();
        msg.ctrl1 = (input_number & 0x1) != 0;      // 取最低位
        msg.ctrl2 = (input_number & 0x2) != 0;      // 取次低位
        msg.ctrl3 = (input_number & 0x4) != 0;      // 取第三位
        msg.ctrl4 = (input_number & 0x8) != 0;      // 取最高位

        // 发布消息
        pub.publish(msg);

        // 输出发布的消息内容
        ROS_INFO("Published: ctrl1=%d, ctrl2=%d, ctrl3=%d, ctrl4=%d", 
                 msg.ctrl1, msg.ctrl2, msg.ctrl3, msg.ctrl4);

        // 延时以保持发布频率
        rate.sleep();
    }

    return 0;
}

