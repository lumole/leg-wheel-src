//this is written by qp

#include "manipulator.h"

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"cmd2arm_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("CMD2ARM",10);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        std::string input;
        std::cout <<"请输入:" << std::endl;
        std::getline(std::cin,input);

        if(!input.empty())
        {
            std_msgs::String msg;
            msg.data = input;
            pub.publish(msg);
        }
        loop_rate.sleep();
    }
    return 0;
}