#include <ros/ros.h>
#include <serial/serial.h>
#include <jsoncpp/json/json.h>
#include <sstream>

std::string build_json_string()
{
    std::ostringstream oss;
    oss << "{";
    oss << "\"T\":102,";
    oss << "\"base\":0.5,";
    oss << "\"shoulder\":0,";
    oss << "\"elbow\":1,";
    oss << "\"hand\":3.1415926,";
    oss << "\"spd\":0,";
    oss << "\"acc\":10";
    oss << "}\n";

    return oss.str();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle n;

    serial::Serial sp;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort("/dev/ttyUSB2");
    sp.setBaudrate(115200);
    sp.setTimeout(to);

    try
    {
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("Unable to open port.");
        return -1;
    }

    if (sp.isOpen())
    {
        ROS_INFO("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(0.5);
    std::string output = build_json_string();

    while (ros::ok())
    {
        sp.write(output);
        ROS_INFO("%s", output.c_str());
        loop_rate.sleep();
    }

    sp.close();
    return 0;
}
