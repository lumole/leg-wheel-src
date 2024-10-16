// class interface(can)
// #ifndef BASE_DRIVER_H_
// #define BASE_DRIVER_H_

#include <stdlib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <ctime>
#include <cstdlib>

#include <math.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <pthread.h>
#include <thread>
#include <unistd.h>
#include <signal.h>
#include <iostream>
#include "sstream"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "message/interface_controller.h"
#include "message/pid_interface.h"
#include "message/vmc_interface.h"
#include "message/kinematics_interface.h"
#include "message/imu_controller.h"
#include "message/manipulator.h"
#include "interface_pkg/controlcan.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include "std_msgs/String.h"
#include <cmath>
#include <unordered_map>



#include "message/cmb_interface.h"
#define pi 3.1415926535
using namespace std;

enum Motor_Type_E
{
    M7015,
    M9015,
    M6012
};

class Motor
{
public:
    Motor(int ID, int ind, Motor_Type_E type, int vel_limit);
    int SetTorque(double t);
    int SetPosition(double p);
    int SetIncrementalPosition(double ip);
    int SetZero(void);
    int Close(void);
    int angle2encoder(double);
    double encoder2angle(double);

private:
    int ID;
    int ind;
    Motor_Type_E type;
    int vel_limit;
    VCI_CAN_OBJ sent_msg[1];
};


class Interface
{
public:
    int sign(int x); // 符号函数
    Interface();
    void ExitCan(int flag);                                                   // can error
    void CloseCan();                                                          // 关闭CAN口
    void Motor_Feedback();                                                    // 读电机反馈，并传数给发布消息
    void M_Motor_Feedback();                                                  // 读电机反馈，并传数给发布消息
    void VMC_Control(const message::vmc_interface::ConstPtr &msg);            // VMC控制
    void Motor_Check(int ID);                                                 // 电机状态读取检测
    void Motor_F_Transmit(int ID, int output);                                // 电机闭环力矩控制
    void Motor_Pos_Transmit(int ID, int output, int hip_motor_vel_limit);     // 电机位置控制
    void M_Motor_Pos_Transmit(int ID, int output, int hip_motor_vel_limit);   // 电机位置控制
    void M_Motor_Pos_I_Transmit(int ID, int output, int hip_motor_vel_limit); // 电机位置控制
    void M_Control(const message::manipulator::ConstPtr &msg);
    void Motor_Speed_Transmit(int ID, int speed); // 电机速度控制
    void Motor_Set_Zero(int ID);                  // 写入零点到电机ROM
    void M_Motor_Set_Zero(int ID);
    void Motor_Close(int ID);   // 关闭电机，清除之前的命令
    void Robot_Init(int angle); // 机器人初始化
    void Motor_State();


    void Cyber_Gear_Motor_Torque(int ID, double output);  //小米电机轮毂扭矩命令
    double linearTransform(int input, double a); 
    void Cyber_Gear_Motor_Select(int ID, int mode);  //小米电机轮毂扭矩命令
    void Cyber_Gear_Motor_Enable(int ID);  //小米电机轮毂扭矩命令
    void Cyber_Gear_Motor_Set_Zero(int ID);//小米电机设置当前位置为零点

    void Cyber_Gear_Motor_Speed_Limit(int ID, float limit);
    void Cyber_Gear_Motor_Speed(int ID, float output);
    
    void CMB_Control(const message::cmb_interface::ConstPtr& msg);
    double Cyber_Gear_Motor_GetAndFix_angle_from_PosEncoder(int ID, double & angle_loss_sum_due_to_zeroing, double angle_from_encoder);
    void Robot_State_Change(const std_msgs::String::ConstPtr &msg);
    void Cyber_Gear_Motor_Loc(int ID, float output);

private:
    bool isMinit;
    bool temprature_err_flag;
    VCI_CAN_OBJ sent_msg[1];
    VCI_CAN_OBJ recv_msg[10];
    VCI_CAN_OBJ m_sent_msg[1];
    VCI_CAN_OBJ m_recv_msg[10];

    // 0523
    VCI_BOARD_INFO pInfo; // 用来获取设备信息。
    int count = 0;        // 数据列表中，用来存储列表序号。
    VCI_BOARD_INFO pInfo1[50];
    int num = 0;
    //

    ros::Publisher motor_pub;
    ros::Publisher m_motor_pub;
    // 控制的回调函数
    ros::Subscriber m_output_sub;
    ros::Subscriber vmc_output_sub;
    message::interface_controller motor_feedback_msg;
    message::interface_controller m_motor_feedback_msg;
    ros::NodeHandle n;


    ros::Subscriber cmb_output_sub;
    ros::Subscriber test_robotstate_sub;

    double lwheelangle_loss_sum_due_to_zeroing;
    double rwheelangle_loss_sum_due_to_zeroing;
};