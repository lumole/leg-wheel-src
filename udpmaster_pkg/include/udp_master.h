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
#include <mutex>
#include <dynamic_reconfigure/server.h>
#include "udpmaster_pkg/initialPoseConfig.h"


#include "message/rs232_elec_mag_ctrl.h"
#include "message/cmb_slave.h"
#include "message/rs232_elec_mag_state.h"
#include "message/imu_controller.h"

class UDPMaster {
public:
    UDPMaster();

    UDPMaster(int _i);

    ~UDPMaster();

    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void catCmdCallback(const std_msgs::Bool::ConstPtr& msg);

    void slave_motor_control_Callback(const message::cmb_slave::ConstPtr& msg);  //?????????????????????slave
    void slave_mag_Control_Callback(const message::rs232_elec_mag_ctrl::ConstPtr& msg);//??????????????????????slave

    void transferData();

    int self_id, remote_id;

private:
    void sendData();
    void recvData();
private:
    int recv_sock, send_sock;
    struct sockaddr_in local_addr, remote_addr;
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> subs;
    std::vector<ros::Publisher> pubs;
    //????????????????????????
    geometry_msgs::Twist twist_buf;   //Twist???  ?????????????????????
    std_msgs::Bool bool_buf;

    message::rs232_elec_mag_ctrl mag_buf;     //???????????????????mag???
                                      //????????????????t
    message::cmb_slave motor_buf;    //???????????????????????
    
    std::mutex _mtx;
};

void reconfigureCallback(udpmaster_pkg::initialPoseConfig& config, uint32_t level);
