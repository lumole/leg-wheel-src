#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h" //传递的消息类型
#include "std_msgs/String.h"
#include "sstream"
#include "eigen3/Eigen/Dense"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <list>
#include <cmath>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include "controlcan.h"
#include "message/manipulator.h"
#include "message/interface_controller.h"
#include "message/imu_controller.h"
#include <serial/serial.h>
#include <string>
#include <sstream>
#include <jsoncpp/json/json.h>
#include <kalman_filter/kf.hpp>
#include <kalman_filter/ukf.hpp>
#include "message/pose.h"

using namespace std;
using std::cin;
using std::cout;
using std::endl;
using std::string;
// 常数
constexpr double pi = 3.1415926535;
constexpr double g = 9.815;

// 控制系统
constexpr double CONTROL_RATE = 50;
constexpr double TIME_STEP = 1.0 / CONTROL_RATE;

// 机械臂参数
constexpr double M_L0 = 0.096;
constexpr double M_L1 = 0.15;
constexpr double M_L2 = 0.069;

double init[3] = {-0.0248387, 0, 0.245414};
double down_loose[3] = {-0.183839, 0, 0.196414};
double down_seize[3] = {-0.183839, 0.024, 0.196414};
double up_loose[3] = {-0.0918387, 0, 0.226414};
double up_seize[3] = {-0.0918387, 0.024, 0.226414};
double wave1[3] = {0.153839, -0.024, 0.236414};
double wave2[3] = {0.153839, 0.024, 0.236414};
double wave0[3] = {0.153839, 0, 0.236414};
constexpr bool right_side = true;
constexpr bool left_side = false;

double startTime_control, endTime_control, endTime_T;
double anglenormalize(double);

struct frame
{
    double x, y, z, time;
};
class playset
{
private:
    list<frame> key_frame;
    double update_timestep;
    int counter;
    list<frame>::iterator it;

public:
    bool state;
    playset(double _timestep)
    {
        update_timestep = _timestep;
        key_frame.clear();
    };
    void setFrame(double _time, double *_xyz);
    frame update(void);
};

struct device
{
    bool side;
    double exp_pos[3] = {0};   // DH定义
    double tou[3] = {0};       // DH定义
    double pos[3] = {0};       // DH定义
    double exp_angle[3] = {0}; // DH定义
    double angle[3];           // DH定义
    playset *p;
};
class M_controller
{
public:
    M_controller();
    void run();
    int pump_flag = 6;    // 气泵
    int mani_joint_l = 5; // 左臂关节号
    int mani_joint_r = 5; // 右臂关节号
    double x = 235;
    double y = 50;
    double z = 234;
    double t = 3.14;
    int axis_flag = 5;
    double angle_base_l = 0;
    double angle_shoulder_l = 0;
    double angle_elbow_l = 90;
    double angle_eoat_l = 180;
    double angle_base_r = 0;
    double angle_shoulder_r = 0;
    double angle_elbow_r = 90;
    double angle_eoat_r = 180;
    double wave_time = 0.5;
    serial::Serial sp_left, sp_right;
    std::string output_l, output_r;
    void initializePorts();

    // serial::Serial ser;

private:

    device *M_L, *M_R;
    void inverse_kine(device *M);
    void Pos_Control();
    void play(device *M);
    void Print(device *M);
    void Feedback(const message::interface_controller &msg);
    void Keyboard_Input(const std_msgs::UInt16 &msg); // 键盘输入;
    void cmd(const std_msgs::String &msg);
    ros::NodeHandle nh;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;
    ros::Subscriber key_sub;
    ros::Subscriber cmd_sub;
    message::manipulator manipulator_msg;
};

class RobotDynamicsUKF : public kalman_filter::ukf_t
{
public:
    RobotDynamicsUKF();

    // Public member functions
    void IMU_Input(const message::imu_controller::ConstPtr &msg);
    void Motor_Input(const message::interface_controller::ConstPtr &msg);
    void update();

    // OPTIONAL: Stores the current control inputs (v_L, v_R).
    double_t v_L;
    double_t v_R;

private:
    // Implement/override the UKF's state transition model.
    void state_transition(const Eigen::VectorXd& xp, Eigen::VectorXd& x) const override;

    // Implement/override the UKF's observation model.
    void observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const override;

    // ROS components
    ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Subscriber imu_sub;
    ros::Subscriber Motor_sub;

    // Messages
    message::imu_controller imu_msg;
    message::interface_controller motor_msg;

    // Constants
    double delta_t = 1.0 / 250; // Time step
    double b = 0.4716;          // Distance between wheels
    const double radius = 0.152 / 2;
};

class WorldFrameArmController {
public:
    WorldFrameArmController();
    std::vector<double> getBaseAbsolutePosition();
    std::vector<double> getArmAbsolutePosition();
    void moveArmToAbsolutePosition(const std::vector<double>& target_position);
    void run();

private:
    std::vector<double> arm_offset; // 机械臂基座相对底盘的偏移
    std::vector<double> arm_joint_positions; // 机械臂的关节位置
    RobotDynamicsUKF *ukf_base;
    std::vector<double> base_world_position;
    std::vector<double> arm_world_position;
    std::vector<double> arm_base_position;
};