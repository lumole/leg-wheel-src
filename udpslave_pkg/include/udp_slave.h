#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>


///a////////////////////////////////

#include "message/imu_controller.h"
#include "message/rs232_elec_mag_state.h"
#include "message/rs232_elec_mag_ctrl.h"
#include "message/cmb_interface.h"

class UDPSlave {
public:
    UDPSlave();

    ~UDPSlave();

    //�����callback���յ�UDP����Ϣ
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void catCmdCallback(const std_msgs::Bool::ConstPtr& msg);


    //�ڵ��ڲ��յ�IMU����
    void Imu_Callback(const message::imu_controller::ConstPtr& msg);//�ڵ��ڲ��յ�IMU����

    void Mag_Callback(const message::rs232_elec_mag_state::ConstPtr& msg);

    //UDP��������
    void transferData();

private:
    int recv_sock, send_sock;
    struct sockaddr_in local_addr, remote_addr;
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subs;
    std::vector<ros::Publisher> pubs;

    //��Ϊ�ӽڵ㣬��Ҫ���������ڵ㷢����Ӧ����Ϣ
    nav_msgs::Odometry odom_buf;    //Odometry��Ϣ������Э�����λ�úͷ�����Ϣ
    message::imu_controller imu_buf; // �Զ���IMU��Ϣ
    message::rs232_elec_mag_state mag_buf;
    std_msgs::Bool bool_buf;
    std::mutex _mtx;
    
};