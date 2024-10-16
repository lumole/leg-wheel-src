#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h" //传递的消息类型
#include "sstream"
#include "eigen3/Eigen/Dense"
#include <iostream>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include "message/imu_controller.h"       //pitch pitch_vel
#include "message/interface_controller.h" //线速度，髋关节角度
#include "message/vmc_interface.h"
#include "message/lqr_state.h"
#include "message/pid_state.h"
#include "pid.h"
#include "index.h"
#include "timevar.h"


#include "message/rs232_elec_mag_state.h"
#include "message/rs232_elec_mag_ctrl.h"
#include "message/cmb_interface.h"

#include "message/test_msg.h"
#include <unordered_map>
#include <kalman_filter/kf.hpp>



using namespace std;
using namespace std::chrono;
using namespace geometry_msgs;
using std::cin;
using std::cout;
using std::endl;
using std::string;

class LowPassFilter {
private:
    float alpha=0.8;   // 滤波系数，决定滤波的平滑程度，范围为 (0, 1)
    float prevValue; // 上一次滤波后的输出值

public:
    // 构造函数，设置滤波系数，alpha 越小，滤波越平滑

    // 重置滤波器，将上一次输出值设为0或初始值
    void reset(float initialValue = 0.0f) {
        prevValue = initialValue;
    }

    // 滤波函数，输入新的采样值，输出滤波后的值
    float filter(float inputValue) {
        prevValue = alpha * inputValue + (1.0f - alpha) * prevValue;
        return prevValue;
    }
};
class filter
{
public:
    filter();
    double series[5];
    double update(double _update);
};
struct Leg_Model
{
    bool isRobotOffGround;
    int counter_OffGround;
    double phi1;  //前髋关节电机的角度
    double phi4; // 后髋关节电机的角度
    double dphi1, dphi4;
    double L0, L0_exp; // VMC腿长和腿长的期望值
    double phi0;       // VMC髋关节角
    TimeVar *dL0;
    double dphi0;
    double F;                    // 腿部支持力
    double F_gravity, F_inertia; // 腿部前馈控制补偿
    double Tp, T, last_T;        // VMC腿部
    double T1, T2;               // T1前，T2后
    double theta, dtheta, x, dx, phi, dphi;
    double dyaw = 0;
    TimeVar *dTheta;
    double theta_exp, dtheta_exp, x_exp, dx_exp, phi_exp, dphi_exp;
    double x_err;
    double K[2][6];
    double speed_now, speed_last;
    double FN;
    PID *L0_pid;
    filter *theta_filter;
    double nl; // 腿部质心位置系数
    int err_counter;
};

void leg_init(Leg_Model *leg);
double filter_update(double data[], double update, int size);
double mid_filt(double data[], int size);
void swap(double *a, double *b);
double LIMIT(double var, double max); // 限幅
int16_t T2iqControl_MF7015(double T); // mf7015扭矩转电流
int16_t T2iqControl_MG6012(double T); // 髋关节扭矩转电流
int16_t T2iqControl_MF9015(double T); // mf9015扭矩转电流

enum RobotState
{
    INIT,
    COMB,
    PRONE,
    NORM,
    NORM_TO_INIT,
    PROTECT
};

enum LegPIDState
{
    PID_NORM,
    PID_OFFROAD
};

class VMC
{
public:
    VMC();
    void run(void);

private:
    enum RobotState robotstate;
    enum LegPIDState legPidState;
    Leg_Model *lleg, *rleg;
    bool isTestMode;
    bool isPlan;
    bool isRobotSafe;
    bool isRobotJumping;
    bool isYawInit;

    double plan_v, plan_w;
    double yaw_exp, roll_exp;
    double dyaw_exp;
    double roll, pitch, yaw;
    double droll, dpitch, dyaw;
    double ddx, ddy, ddz;
    filter *yawfilter;
    int counter_P;
    int counter_Jump;
    int counter_plan;
    int counter_Motor = 0, error_Motor = 0, last_counter_Motor = 0;
    int counter_IMU = 0, error_IMU = 0, last_counter_IMU = 0;

    //////////////新增加的变量/////////////////////////////////////

    //电磁铁状态
    bool mag_leftfront;
    bool mag_rightfront;
    bool mag_leftback;
    bool mag_rightback;

    bool mag_slave_left_back;
    bool mag_slave_right_back;



    double pitch_fuse;     //pitch融合结果
    double roll_fuse;      //roll融合结果


    double slave_l_phi1;    //从机左前调整角度
    double slave_l_phi4;    //从机左后调整角度
    double slave_r_phi1;    //从机右前调整角度
    double slave_r_phi4;    //从机右后调整角度
    double slave_T1;
    double slave_T2;
    //////////////////////////////////////////////
    // PID运算
    PID *test;
    PID *theta_pid;
    PID *turn_pid;
    PID *roll_pid;
    void leg_pos(Leg_Model *leg);  // 计算腿部姿态，得到腿长L0和足端角度phi0，输入为国际标准单位
    void leg_spd(Leg_Model *leg);  // 计算腿部运动速度dL0，dphi0，输入为国际标准单位
    void FN_solve(Leg_Model *leg); // 求解支持力
    void LQR_K(Leg_Model *leg);    // 由腿长得到对应的K矩阵
    void T_Calc(Leg_Model *leg);   // 算轮毂输出扭矩
    void Tp_Calc(Leg_Model *leg);  // 算髋关节输出扭矩
    void leg_conv(Leg_Model *leg); // 求VMC转换矩阵
    void Plan_exp_set(void);
    void VMC_output();              // 系统保护&VMC控制信息发送
    void turn_control();            // 转向控制
    void leg_length_control();      // 腿长控制 包含横滚
    double nl_calc(Leg_Model *leg); // 腿部质心常数计算
    void state_print();             // 各种状态打印输出
    void VMC_controller();          // VMC控制器的全部解算
    // CAN通信函数
    void Motor_Input(const message::interface_controller::ConstPtr &msg); // 读取电机反馈
    void IMU_Input(const message::imu_controller::ConstPtr &msg);         // 读IMU数据
    void Keyboard_Input(const std_msgs::UInt16 &msg);                     // 键盘输入
    void Joy(const sensor_msgs::Joy::ConstPtr &msg);                      // 手柄回调函数
    void Plan(const geometry_msgs::Twist msg);                            // 手柄回调函数
    // void Plan(const geometry_msgs::TwistStamped msg);                            // 手柄回调函数


    
    ////////////////////额外的回调函数///////////
   

    void Serial_feedback(const message::rs232_elec_mag_state::ConstPtr &msg);            //本机232串口包回调函数
    void Slave_Serial_feedback(const message::rs232_elec_mag_state::ConstPtr &msg);            //从机232串口包回调函数


    void Slave_IMU_Input(const message::imu_controller::ConstPtr &msg);            //从机232串口包回调函数

    void Motor_adjust_Input(const message::cmb_interface::ConstPtr &msg);            //从机232串口包回调函数
    ////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////拼接后函数//////////////////////////

    void CMB_controller();   //接收信息，产生调整
    void CMB_output();       //输出命令给到interface
    double calculateAngularVelocityChange(const message::imu_controller&imu_m, const message::imu_controller& imu_s);
    void fuseIMUData();    //混合滤波，获得混合后的pitch和roll信息


    void Prone_VMC_controller();//伏地模式下的VMC控制器
    void Prone_VMC_output();//伏地模式下的VMC输出
    message::test_msg TEST_setLegModelDataFromStruct(const Leg_Model& leg_model);
    void Leg_PID_Clear();
    void L0_pid_correct(Leg_Model *leg);
    bool PROTECT_Program();
    double adjust_dx_exp(double phi, double dx_exp) ;
    double detectAndEstimate(int motor_id, double new_x, double new_v);
    

    ros::Publisher test_rleg_pub;
    ros::Publisher test_lleg_pub;
    ///////////////////////////////////////////////////////
    ros::Publisher pid_state_pub;
    ros::Publisher vmc_output_pub;
    ros::Publisher lqr_state_pub;
    ros::Subscriber imu_sub;
    ros::Subscriber plan_sub;
    ros::Subscriber motor_input_sub;
    ros::Subscriber key_sub;
    ros::Subscriber joy_sub;
    ros::NodeHandle n;

    ros::Publisher cmb_output_pub;    //CMB模式下的发布者
    ros::Subscriber serial_sub;    //订阅本机串口的电磁铁反馈信息

    ros::Subscriber motor_adjust_sub;    //订阅从udp_pkg中的从机修改信息

    ros::Subscriber slave_imu_sub;     //订阅从机的imu反馈信息
    message::imu_controller imu_m;
    message::imu_controller imu_m_pre;   //记录之前的信息
    message::imu_controller imu_s;
    message::imu_controller imu_s_pre;   //记录之前的信息
    ros::Publisher cmb_slave_output_pub;   //CMB模式下的从机位置发布者
    ros::Publisher test_robotstate_pub;
    LowPassFilter lsfilter, rsfilter;
};

