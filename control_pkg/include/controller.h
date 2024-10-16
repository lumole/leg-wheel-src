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
#include <math.h>
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

#include "message/cmb_interface.h"
#include "message/cmb_slave.h"
#include "message/rs232_elec_mag_ctrl.h"
#include "message/rs232_elec_mag_state.h"

using namespace std;
using namespace std::chrono;
using namespace geometry_msgs;
using std::cin;
using std::cout;
using std::endl;
using std::string;

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
    double phi1; // 前髋关节电机的角度
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
    double theta_exp, dtheta_exp, x_exp, dx_exp, phi_exp, dphi_exp; // 单体 6个变量
    double COMB_z_exp, COMB_x_exp, COMB_z_exp0, COMB_x_exp0;        // 拼接后腿部变量
    double COMB_x, COMB_z;                                          // 拼接后腿部坐标系
    double Fx, Fz;                                                  // 沿x/z的PID输出
    double x_err;
    double K[2][6];
    double speed_now, speed_last;
    double FN;
    PID *L0_pid;
    PID *COMB_z_pid;
    PID *COMB_x_pid;
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
    NORM
};

class VMC
{
public:
    VMC();
    void run(void);

private:
    enum RobotState robotstate;
    Leg_Model *lleg, *rleg;
    bool isTestMode;
    bool isPlan;
    bool isRobotSafe;
    bool isRobotJumping;
    bool isYawInit;

    double plan_v, plan_w;
    double yaw_exp, roll_exp;
    double roll, pitch, yaw;
    double droll, dpitch, dyaw;
    double ddx, ddy, ddz;
    filter *yawfilter;
    int counter_P;
    int counter_Jump;
    int counter_plan;
    int counter_Motor = 0, error_Motor = 0, last_counter_Motor = 0;
    int counter_IMU = 0, error_IMU = 0, last_counter_IMU = 0;
    double yaw_m, yaw_s, yaw_exp_m, yaw_exp_s;
    double fai_b, fai_f;
    double theta_turn_exp = 0;
    double theta_turn;
    double v_vertical;
    double a, b, c, d;
    int counter_test = 0;

    //////////////新增加的变量/////////////////////////////////////
    // 电磁铁状态
    bool mag_leftfront;
    bool mag_rightfront;
    bool mag_leftback;
    bool mag_rightback;

    bool mag_slave_left_back;
    bool mag_slave_right_back;

    double pitch_fuse; // pitch融合结果
    double roll_fuse;  // roll融合结果

    double slave_l_phi1; // 从机左前调整角度
    double slave_l_phi4; // 从机左后调整角度
    double slave_r_phi1; // 从机右前调整角度
    double slave_r_phi4; // 从机右后调整角度

    double master_l_phi1; // 从机左前调整角度
    double master_l_phi4; // 从机左后调整角度
    double master_r_phi1; // 从机右前调整角度
    double master_r_phi4; // 从机右后调整角度
    double slave_T1;
    double slave_T2;

    double master_L0_exp, slave_L0_exp, master_l_xx_exp, slave_l_xx_exp, master_r_xx_exp, slave_r_xx_exp;
    double slave_l_L0_exp, slave_r_L0_exp;
    double dx_run0, dx_rund;
    double V = 0.0;
    double master_dx_l, master_dx_r, slave_dx_l, slave_dx_r;
    double master_dx_l_turn, master_dx_r_turn, slave_dx_l_turn, slave_dx_r_turn;
    double master_dx_l_run, master_dx_r_run, slave_dx_l_run, slave_dx_r_run;
    double master_dx_l_run0, master_dx_r_run0, slave_dx_l_run0, slave_dx_r_run0;
    double master_dx_l_rund, master_dx_r_rund, slave_dx_l_rund, slave_dx_r_rund;
    double master_l_x_exp, master_l_z_exp, slave_l_x_exp, slave_l_z_exp; // left
    double master_r_x_exp, master_r_z_exp, slave_r_x_exp, slave_r_z_exp; // right
    double delta_master_l_z_exp, delta_master_r_z_exp, delta_slave_l_z_exp, delta_slave_r_z_exp;
    double delta_master_l_x_exp, delta_master_r_x_exp, delta_slave_l_x_exp, delta_slave_r_x_exp;
    double delta_master_l_xx_exp, delta_master_r_xx_exp, delta_slave_l_xx_exp, delta_slave_r_xx_exp;
    double delta_master_l_z_exp1, delta_master_r_z_exp1, delta_slave_l_z_exp1, delta_slave_r_z_exp1;
    double ma_T1;
    double ma_T2;
    double COMB_roll_exp, COMB_pitch_exp;
    double F_roll, F_pitch;
    double COMB_L0_exp;
    double t; // trot步态计时
    double sigma;
    double zep, xep_b, xep_z;
    double zs;
    bool trot_flag = 0;

    // 20240912
    int walk_flag, count_walk;
    double walk_m_l, walk_m_r, walk_s_l, walk_s_r;

    double delta_x, R, w;

    double llegphi1;
    double llegphi4;
    double rlegphi1;
    double rlegphi4;
    int roll_flag, pitch_flag;
    double car_break;
    int slave_flag = 0;

    //////////////////////////////////////////////
    // PID运算
    PID *test;
    PID *theta_pid;
    PID *turn_pid;
    PID *roll_pid;
    PID *COMB_roll_pid;
    PID *COMB_pitch_pid;
    PID *COMB_turn_pid_m;
    PID *COMB_turn_pid_s;
    PID *COMB_recover_pid_m;
    PID *COMB_recover_pid_s;
    void leg_pos(Leg_Model *leg);       // 计算腿部姿态，得到腿长L0和足端角度phi0，输入为国际标准单位
    void leg_spd(Leg_Model *leg);       // 计算腿部运动速度dL0，dphi0，输入为国际标准单位
    void FN_solve(Leg_Model *leg);      // 求解支持力
    void LQR_K(Leg_Model *leg);         // 由腿长得到对应的K矩阵
    void T_Calc(Leg_Model *leg);        // 算轮毂输出扭矩
    void Tp_Calc(Leg_Model *leg);       // 算髋关节输出扭矩
    void leg_conv(Leg_Model *leg);      // 求VMC转换矩阵
    void leg_conv_comb(Leg_Model *leg); // 拼接模式下的VMC计算
    void Plan_exp_set(void);
    void VMC_output();              // 系统保护&VMC控制信息发送
    void turn_control();            // 转向控制
    void leg_length_control();      // 腿长控制 包含横滚
    void COMB_leg_length_control(); // pinjie
    double nl_calc(Leg_Model *leg); // 腿部质心常数计算
    void state_print();             // 各种状态打印输出
    void VMC_controller();          // VMC控制器的全部解算
    double Inverse_Kinematics(double L0_exp, double theta, int flag);
    double Inverse_Kinematics_xyz(double x, double z, int flag);
    double Forward_Kinematics_xyz(Leg_Model *leg, int flag);
    double leg_length_limit(Leg_Model *leg);
    double vel_limit(double low_lim, double high_lim, double vel);
    double COMB_turn_l(double R, double w, double a, double c);
    double COMB_turn_r(double R, double w, double a, double c);
    int sign(double x); // 符号函数
    // CAN通信函数
    void Motor_Input(const message::interface_controller::ConstPtr &msg); // 读取电机反馈
    void IMU_Input(const message::imu_controller::ConstPtr &msg);         // 读IMU数据
    void Keyboard_Input(const std_msgs::UInt16 &msg);                     // 键盘输入
    void Joy(const sensor_msgs::Joy::ConstPtr &msg);                      // 手柄回调函数
    void Plan(const geometry_msgs::Twist msg);                            // 手柄回调函数
    // void Plan(const geometry_msgs::TwistStamped msg);                            // 手柄回调函数

    ////////////////////额外的回调函数///////////

    void Serial_feedback(const message::rs232_elec_mag_state::ConstPtr &msg);       // 本机232串口包回调函数
    void Slave_Serial_feedback(const message::rs232_elec_mag_state::ConstPtr &msg); // 从机232串口包回调函数

    void Slave_IMU_Input(const message::imu_controller::ConstPtr &msg); // 从机232串口包回调函数
    ////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////拼接后函数//////////////////////////

    void COMB_controller_Pos(int Kpitch, int Kroll, int Dpitch, int Droll); // 接收信息，产生调整
    void COMB_output_Pos();                                                 // 输出命令给到interface
    void COMB_controller_F();
    void COMB_output_F_single(); // 对输出进行限幅或衰减、判断机器人状态是否安全 发送安全的控制命令到接口 interface
    void COMB_output_F_multi();
    double calculateAngularVelocityChange(const message::imu_controller &imu_m, const message::imu_controller &imu_s);
    void fuseIMUData(); // 混合滤波，获得混合后的pitch和roll信息

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

    ros::Publisher cmb_output_pub;    // CMB模式下的发布者
    ros::Subscriber serial_sub;       // 订阅本机串口的电磁铁反馈信息
    ros::Subscriber slave_serial_sub; // 订阅从机串口的电磁铁反馈信息
    ros::Subscriber slave_imu_sub;    // 订阅从机的imu反馈信息
    message::imu_controller imu_m;
    message::imu_controller imu_m_pre; // 记录之前的信息
    message::imu_controller imu_s;
    message::imu_controller imu_s_pre;   // 记录之前的信息
    ros::Publisher cmb_slave_output_pub; // CMB模式下的从机位置发布者
};