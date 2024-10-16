#include "controller.h"
void VMC::Motor_Input(const message::interface_controller::ConstPtr &msg) // 轮毂输入并处理
{
    counter_Motor++; // 计数电机回调函数执行次数
    //// x'（dps）右
    //rleg->speed_now = msg->speed1;
    //rleg->dx = rleg->speed_now * pi * radius / 180; // 轮子线速度 dx(右)
    //rleg->x += (rleg->speed_now + rleg->speed_last) * pi * radius / 180 / 2 / CONTROL_RATE;
    //rleg->speed_last = rleg->speed_now;

    //// x'（dps）左
    //lleg->speed_now = msg->speed2;
    //lleg->dx = lleg->speed_now * pi * radius / 180; // 轮子线速度 dx(左)
    //lleg->x += (lleg->speed_now + lleg->speed_last) * pi * radius / 180 / 2 / CONTROL_RATE;
    //lleg->speed_last = lleg->speed_now;


    // x'（dps）右
    rleg->speed_now = rsfilter.filter(msg->speed1);
    rleg->dx = rleg->speed_now  * radius; // 轮子线速度 dx(右)由于本身是弧度制单位，因此不需要额外转化
    // rleg->x += (rleg->speed_now + rleg->speed_last) * radius / 2 / CONTROL_RATE;
    // rleg->x = msg->angle1 * radius ;
    rleg->x = detectAndEstimate('r', msg->angle1, rleg->dx)*radius;
    rleg->speed_last = rleg->speed_now;

    // x'（dps）左
    lleg->speed_now = lsfilter.filter(msg->speed2);
    lleg->dx = lleg->speed_now  * radius ; // 轮子线速度 dx(左)
    // lleg->x += (lleg->speed_now + lleg->speed_last) * radius / 2 / CONTROL_RATE;
    // lleg->x = msg->angle2 * radius ;
    lleg->x = detectAndEstimate('l', msg->angle2, lleg->dx)*radius;
    lleg->speed_last = lleg->speed_now;

    // phi1 phi4（rad）
    phi_145 = msg->encoder5 * 2 * pi / 65535;      // 右前
    phi_143 = pi + msg->encoder3 * 2 * pi / 65535; // 右后
    phi_146 = msg->encoder6 * 2 * pi / 65535;      // 左前
    phi_144 = pi + msg->encoder4 * 2 * pi / 65535; // 左后
    // dphi1 dphi4（rad/s）
    dphi_145 = msg->speed5 * pi / 180 / 8; // 右前
    dphi_143 = msg->speed3 * pi / 180 / 8; // 右后
    dphi_146 = msg->speed6 * pi / 180 / 8; // 左前
    dphi_144 = msg->speed4 * pi / 180 / 8; // 左后
}

void VMC::IMU_Input(const message::imu_controller::ConstPtr &msg) // IMU输入
{
    imu_m=*msg;
    counter_IMU++; // 计数IMU回调函数执行次数
    // auto startTime_IMU = ros::Time::now().toNSec(); // IMU回调函数执行一次计时起点

    lleg->phi = -msg->angle_pitch * (pi) / 180; // 俯仰角 phi = pitch
    rleg->phi = -msg->angle_pitch * (pi) / 180; // 俯仰角 phi = pitch

    lleg->dphi = -msg->angle_vel_pitch; // 俯仰角速度 dphi
    rleg->dphi = -msg->angle_vel_pitch; // 俯仰角速度 dphi

    dyaw = msg->angle_vel_yaw;

    yaw = msg->angle_yaw;     // 航向角
    pitch = msg->angle_pitch; // 俯仰角
    roll = msg->angle_roll;   // 横滚角
    ddx = msg->acc_x;
    ddy = msg->acc_y;
    ddz = msg->acc_z;
    if (!isYawInit && counter_P > init_time / TIME_STEP - 10) // 取启动瞬间的yaw为yaw_exp
    {
        yaw_exp = yaw;
        isYawInit = true;
    }
    if (isYawInit)
    {
        yaw=yawfilter->update(yaw);
        cout<<"yaw_filted"<<yaw<<endl;
    }
    imu_m_pre = imu_m;
    
}
void VMC::Plan(const geometry_msgs::Twist msg)
{
    counter_plan = 0;
    isPlan = true;
    plan_v = msg.linear.x;
    plan_w = msg.angular.z;
}
// void VMC::Plan(const geometry_msgs::TwistStamped msg)
// {
//     counter_plan = 0;
//     isPlan = true;
//     plan_v = msg.twist.linear.x;
//     plan_w = msg.twist.angular.z;
// }
void VMC::Keyboard_Input(const std_msgs::UInt16 &msg) // 键盘回调函数
{
    double angle_step = 3;
    switch (msg.data)
    {
    /* W/S：前进后退，A/D：左右转 */
    case 'W': // W
        lleg->dx_exp += 0.1;
        rleg->dx_exp += 0.1;
        break;
    case 'S': // S
        lleg->dx_exp -= 0.1;
        rleg->dx_exp -= 0.1;
        break;
    case 'A': // A
        yaw_exp += angle_step;
        lleg->x_exp -= angle_step * pi / 180 * half_wheel_d;
        rleg->x_exp += angle_step * pi / 180 * half_wheel_d;
        ROS_WARN("A,yaw_exp=%f\n", yaw_exp);
        break;
    case 'D': // D
        yaw_exp -= angle_step;
        lleg->x_exp += angle_step * pi / 180 * half_wheel_d;
        rleg->x_exp -= angle_step * pi / 180 * half_wheel_d;
        ROS_WARN("D,yaw_exp=%f\n", yaw_exp);
        break;
    /* Q/E:高度调整 */
    case 'Q': // Q变高
        if (robotstate == NORM) {
            lleg->L0_exp += 0.01;
            rleg->L0_exp += 0.01;
            ROS_WARN("Q,L0_exp=%f\n", lleg->L0_exp * 0.5 + rleg->L0_exp * 0.5);
        }
        else if (robotstate == COMB) {       //0.5度一调整
            lleg->phi1-=400;
            lleg->phi4+=400;
            rleg->phi1+=400;
            rleg->phi4-=400;
        }
        break;
    case 'E': // E变矮
        if (robotstate == NORM) {
            // if (lleg->L0_exp > 0.156719 && rleg->L0_exp > 0.156719)
            if (lleg->L0_exp > 0.09 && rleg->L0_exp > 0.09)
            {
                lleg->L0_exp -= 0.01;
                rleg->L0_exp -= 0.01;
            }
            ROS_WARN("E,L0_exp=%f\n", lleg->L0_exp * 0.5 + rleg->L0_exp * 0.5);
        }
        else if (robotstate == COMB) {        //0.5度一调整
            lleg->phi1+=400;
            lleg->phi4-=400;
            rleg->phi1-=400;
            rleg->phi4+=400;
        }
        break;
    case 'C':
        isRobotSafe = false; // 这个会C掉所有节点，在keyboard函数里也有处理
        break;
    case 'R':
        if (robotstate == INIT)
        {
            robotstate = NORM;
        }
        break;
    case 'P':
        if (robotstate == INIT)
        {
            robotstate = PRONE;
        }
        break;
    case 'Z': // Z
        // if (!isRobotJumping)
        // {
        //     lleg->L0_exp += jump_length;
        //     rleg->L0_exp += jump_length;
        //     isRobotJumping = true;
        //     counter_Jump = 0;
        // }
        break;
        // UJIKOLP被机械臂节点占用
    }
}

void VMC::Joy(const sensor_msgs::Joy::ConstPtr &Joy) // 手柄回调函数
{
    // cout << "axes(" << Joy->axes.size() << "): [";
    // for (int i = 0; i < Joy->axes.size(); i++)
    // {
    //     cout << Joy->axes.at(i) << " ,";
    // }
    // cout << "]" << endl
    //      << "buttons(" << Joy->buttons.size() << "): [";
    // for (int i = 0; i < Joy->buttons.size(); i++)
    // {
    //     cout << Joy->buttons.at(i) << " ,";
    // }
    // cout << "]" << endl;

    double angle_step = 90;
    // //[左区按键]上/下箭头，左正右负
    // lleg->dx_exp += 0.2 * Joy->axes[7];
    // rleg->dx_exp += 0.2 * Joy->axes[7];
    // //[左区按键]左/右箭头，左正右负
    // yaw_exp += angle_step * Joy->axes[6];
    // lleg->x_exp -= angle_step * Joy->axes[6] * pi / 180 * half_wheel_d;
    // rleg->x_exp += angle_step * Joy->axes[6] * pi / 180 * half_wheel_d;
    // //[左区按键]L1升高，R1降低
    // // lleg->L0_exp += 0.01 * (Joy->buttons[6] - Joy->buttons[7]);
    // // rleg->L0_exp += 0.01 * (Joy->buttons[6] - Joy->buttons[7]);
    // //[左区按键]L2急刹
    // if (Joy->buttons[8] == 1)
    // {
    //     lleg->dx_exp = 0;
    //     rleg->dx_exp = 0;
    // }

    if(robotstate == NORM)
    {
        double wheel_speed=1;
        //[左遥感]
        lleg->dx_exp = wheel_speed * Joy->axes[1];
        rleg->dx_exp = wheel_speed * Joy->axes[1];
        dyaw_exp = angle_step * Joy->axes[0];
        lleg->dx_exp -= angle_step * Joy->axes[0] * pi / 180 * half_wheel_d;
        rleg->dx_exp += angle_step * Joy->axes[0] * pi / 180 * half_wheel_d;

        if(Joy->buttons[4] == 1)
        {
            lleg->L0_exp -= 0.01;
            rleg->L0_exp -= 0.01;
        }
        if(Joy->buttons[5] == 1)
        {
            lleg->L0_exp += 0.01;
            rleg->L0_exp += 0.01;
        }
    }
    else if(robotstate == PRONE)
    {
        lleg->dx_exp = 6 * Joy->axes[1];
        rleg->dx_exp = 6 * Joy->axes[1];
        lleg->dx_exp -= 6*Joy->axes[0];
        rleg->dx_exp += 6*Joy->axes[0];
    }

    if(Joy->buttons[0] == 1)
    {
        if (robotstate == INIT)
        {
            robotstate = NORM;

            std_msgs::String normmsg;
            normmsg.data = "NORM";
            test_robotstate_pub.publish(normmsg);
        }
    }
    else if(Joy->buttons[1] == 1)
    {
        if (robotstate == INIT)
        {
            robotstate = PRONE;
            std_msgs::String msg;
            msg.data = "PRONE";
            test_robotstate_pub.publish(msg);
            // lleg->L0_exp = prone_L0_exp;
            // rleg->L0_exp = prone_L0_exp;
        }
    }
    else if(Joy->buttons[2] == 1)
    {
        if(robotstate == NORM)
            robotstate = NORM_TO_INIT;
        else if(robotstate == PRONE)
            robotstate = INIT;
    }
    else if(Joy->buttons[3] == 1)
    {
        robotstate = NORM_TO_INIT;
        std_msgs::String msg;
        msg.data = "STOP";
        test_robotstate_pub.publish(msg);
    }
    else if(Joy->axes[5]==-1)
    {
        if(legPidState==PID_NORM)
            legPidState=PID_OFFROAD;
        else if(legPidState==PID_OFFROAD)
            legPidState=PID_NORM;
    }


}


void VMC::Serial_feedback(const message::rs232_elec_mag_state::ConstPtr &msg) {
    mag_leftfront = msg->state1;
    mag_rightfront = msg->state2;
    mag_leftback = msg->state3;
    mag_rightback = msg->state4;

}

void VMC::Slave_Serial_feedback(const message::rs232_elec_mag_state::ConstPtr &msg) {
    mag_slave_left_back = msg->state3;
    mag_slave_right_back = msg->state4;

}

void VMC::Slave_IMU_Input(const message::imu_controller::ConstPtr &msg) {
    imu_s = *msg;
    imu_s_pre = imu_s;
}

void VMC::Motor_adjust_Input(const message::cmb_interface::ConstPtr &msg) {
    slave_l_phi1 = msg->P1_l;
    slave_l_phi4 = msg->P2_l;
    slave_r_phi1 = msg->P1_r;
    slave_r_phi4 = msg->P2_r;
    slave_T1 = msg->T_l;
    slave_T2 = msg->T_r;
}

double VMC::detectAndEstimate(int motor_id, double new_x, double new_v) {

    static ros::Publisher test_dead = n.advertise<std_msgs::UInt16>( "/is_motor_dead", 10);
    // 定义电机状态的结构体
    struct MotorState {
        double last_v = 0.0;
        double last_x = 0.0;
        double current_x = 0.0;
        bool deadlock_detected = false;
        double dead_x = 0.0;
        std::chrono::high_resolution_clock::time_point last_time;
    };

    // 静态unordered_map，用于存储所有电机的状态
    static std::unordered_map<int, MotorState> motorStates;

    // 获取或初始化指定电机的状态
    MotorState &state = motorStates[motor_id];

    // 获取当前时间
    auto current_time = std::chrono::high_resolution_clock::now();
    double delta_t = 0.0;

    // 如果不是第一次运行，则计算时间差
    if (state.last_time.time_since_epoch().count() > 0) {
        delta_t = std::chrono::duration<double>(current_time - state.last_time).count();
    }

    // 更新最后一次调用的时间
    state.last_time = current_time;

    // 如果已经检测到死机
    if (state.deadlock_detected) {
        // 使用最后的速度来估算当前位置
        state.current_x = state.last_x + state.last_v * delta_t;
        state.last_x = state.current_x; // 更新估算的最后位置

        // 检测传感器数据是否恢复
        if ( new_x != state.dead_x) {
            // 数据更新了，认为电机恢复
            state.deadlock_detected = false;
            state.last_x = new_x;
            state.last_v = new_v;
        }
    } else {
        // 如果速度和位置与上次相同，则认为死机
        if ( new_x == state.last_x) {
            state.deadlock_detected = true;
            state.dead_x = new_x;
        } else {
            // 正常更新数据，记录最新的速度和位置
            state.last_x = new_x;
            state.last_v = new_v;
            state.current_x = new_x;
        }
    }

    if(motor_id == 'r')
    {
        std_msgs::UInt16 msg;
        msg.data = state.deadlock_detected;
        test_dead.publish(msg);
    }
    

    return state.current_x;
}