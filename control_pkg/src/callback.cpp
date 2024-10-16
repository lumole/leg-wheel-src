#include "controller.h"
void VMC::Motor_Input(const message::interface_controller::ConstPtr &msg) // 轮毂输入并处理
{
    counter_Motor++; // 计数电机回调函数执行次数
    //// x'（dps）右
    // rleg->speed_now = msg->speed1;
    // rleg->dx = rleg->speed_now * pi * radius / 180; // 轮子线速度 dx(右)
    // rleg->x += (rleg->speed_now + rleg->speed_last) * pi * radius / 180 / 2 / CONTROL_RATE;
    // rleg->speed_last = rleg->speed_now;

    //// x'（dps）左
    // lleg->speed_now = msg->speed2;
    // lleg->dx = lleg->speed_now * pi * radius / 180; // 轮子线速度 dx(左)
    // lleg->x += (lleg->speed_now + lleg->speed_last) * pi * radius / 180 / 2 / CONTROL_RATE;
    // lleg->speed_last = lleg->speed_now;

    // x'（dps）右
    rleg->speed_now = msg->speed1;
    rleg->dx = rleg->speed_now * radius; // 轮子线速度 dx(右)由于本身是弧度制单位，因此不需要额外转化
    rleg->x += (rleg->speed_now + rleg->speed_last) * radius / 2 / CONTROL_RATE;
    rleg->speed_last = rleg->speed_now;

    // ROS_INFO("右轮%.2f", rleg->dx);

    // x'（dps）左
    lleg->speed_now = msg->speed2;
    lleg->dx = lleg->speed_now * radius; // 轮子线速度 dx(左)
    lleg->x += (lleg->speed_now + lleg->speed_last) * radius / 2 / CONTROL_RATE;
    lleg->speed_last = lleg->speed_now;

    // ROS_INFO("左轮%.2f", lleg->dx);

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
    imu_m = *msg;
    counter_IMU++; // 计数IMU回调函数执行次数
    // auto startTime_IMU = ros::Time::now().toNSec(); // IMU回调函数执行一次计时起点

    lleg->phi = -msg->angle_pitch * (pi) / 180; // 俯仰角 phi = pitch
    rleg->phi = -msg->angle_pitch * (pi) / 180; // 俯仰角 phi = pitch

    lleg->dphi = -msg->angle_vel_pitch; // 俯仰角速度 dphi
    rleg->dphi = -msg->angle_vel_pitch; // 俯仰角速度 dphi

    dyaw = msg->angle_vel_yaw;

    yaw = msg->angle_yaw;     // 航向角 of NORM
    yaw_m = msg->angle_yaw;   // 航向角 of COMB
    pitch = msg->angle_pitch; // 俯仰角
    roll = msg->angle_roll;   // 横滚角
    ddx = msg->acc_x;
    ddy = msg->acc_y;
    ddz = msg->acc_z;
    if (!isYawInit && counter_P > init_time / TIME_STEP - 10) // 取启动瞬间的yaw为yaw_exp
    {
        yaw_exp = yaw;
        yaw_exp_m = yaw;
        yaw_exp_s = yaw_s;
        theta_turn_exp = 0;
        isYawInit = true;
    }
    if (isYawInit)
    {
        yaw = yawfilter->update(yaw);
        // cout << "yaw_filted" << yaw << endl;
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

double VMC::COMB_turn_l(double R, double w, double a, double c)
{
    double i, j, V;
    V = abs(R * w);
    i = a * w;
    j = (c * c * w * w) / (V - a * w);
    return V - i + j;
}

double VMC::COMB_turn_r(double R, double w, double a, double c)
{
    double i, j, V;
    V = abs(R * w);
    i = a * w;
    j = (c * c * w * w) / (V + a * w);
    return V + i + j;
}

void VMC::Keyboard_Input(const std_msgs::UInt16 &msg) // 键盘回调函数
{
    double angle_step = 3;
    // if(car_break){
    //     msg.data='Z';
    // }
    switch (msg.data)
    {
    /* W/S：前进后退，A/D：左右转 */
    case 'W': // W
        if (robotstate == NORM)
        {
            lleg->dx_exp += 0.3;
            rleg->dx_exp += 0.3;
        }
        else if (robotstate == COMB)
        { // forward
            master_dx_l -= 0.05 / radius;
            master_dx_r -= 0.05 / radius;
            slave_dx_l += 0.05 / radius;
            slave_dx_r += 0.05 / radius;
            // ma_T1 += 0.1;
            // ma_T2 += 0.1;
        }
        cout << master_dx_l << endl;
        cout << master_dx_r << endl;
        break;
    case 'S': // S
        if (robotstate == NORM)
        {
            lleg->dx_exp -= 0.3;
            rleg->dx_exp -= 0.3;
        }
        else if (robotstate == COMB)
        {
            master_dx_l += 0.05 / radius;
            master_dx_r += 0.05 / radius;
            slave_dx_l -= 0.05 / radius;
            slave_dx_r -= 0.05 / radius;
            // ma_T1 -= 0.1; // back
            // ma_T2 -= 0.1;
        }
        cout << master_dx_l << endl;
        cout << master_dx_r << endl;
        break;
    case 'A': // A yuandi
        if (robotstate == NORM)
        {
            yaw_exp += angle_step;
            lleg->x_exp -= angle_step * pi / 180 * half_wheel_d;
            rleg->x_exp += angle_step * pi / 180 * half_wheel_d;
            ROS_WARN("A,yaw_exp=%f\n", yaw_exp);
        }
        else if (robotstate == COMB)
        {
            master_dx_l = 1;
            master_dx_r = -1;
            slave_dx_l = -1;
            slave_dx_r = 1;
            // master_L0_exp = 0.22291;
            // slave_L0_exp = 0.22291;
            // master_theta_exp = pi / 2 - 30 * pi / 180;
            // slave_theta_exp = pi / 2 + 30 * pi / 180;
            master_l_x_exp = L5 / 2 + 0.1;
            slave_l_x_exp = L5 / 2 - 0.1;
            master_l_z_exp = 0.15;
            slave_l_z_exp = 0.15;
            master_r_x_exp = L5 / 2 + 0.1;
            slave_r_x_exp = L5 / 2 - 0.1;
            master_r_z_exp = 0.15;
            slave_r_z_exp = 0.15;
            // ma_T1 += 0.4;
            // ma_T2 -= 0.4;
        }
        // cout << ma_T1 << endl;
        // cout << ma_T2 << endl;
        break;

    case 'D': // D
        if (robotstate == NORM)
        {
            yaw_exp -= angle_step;
            lleg->x_exp += angle_step * pi / 180 * half_wheel_d;
            rleg->x_exp -= angle_step * pi / 180 * half_wheel_d;
            ROS_WARN("D,yaw_exp=%f\n", yaw_exp);
        }
        else if (robotstate == COMB)
        {
            master_dx_l = -1;
            master_dx_r = 1;
            slave_dx_l = 1;
            slave_dx_r = -1;
            // master_L0_exp = 0.22291;
            // slave_L0_exp = 0.22291;
            // master_theta_exp = pi / 2 - 30 * pi / 180;
            // slave_theta_exp = pi / 2 + 30 * pi / 180;
            master_l_x_exp = L5 / 2 + 0.1;
            slave_l_x_exp = L5 / 2 - 0.1;
            master_l_z_exp = 0.15;
            slave_l_z_exp = 0.15;
            master_r_x_exp = L5 / 2 + 0.1;
            slave_r_x_exp = L5 / 2 - 0.1;
            master_r_z_exp = 0.15;
            slave_r_z_exp = 0.15;
            // ma_T1 -= 0.2;
            // ma_T2 += 0.2;
        }
        cout << ma_T1 << endl;
        cout << ma_T2 << endl;
        break;
    /* Q/E:高度调整 */
    case 'Q': // Q变高
        if (robotstate == NORM)
        {
            lleg->L0_exp += 0.01;
            rleg->L0_exp += 0.01;
            ROS_WARN("Q,L0_exp=%f\n", lleg->L0_exp * 0.5 + rleg->L0_exp * 0.5);
        }
        else if (robotstate == COMB)
        { // 0.5度一调整
            // lleg->phi1-=400;
            // lleg->phi4+=400;
            // rleg->phi1+=400;
            // rleg->phi4-=400;
            master_L0_exp += 0.02;
            slave_L0_exp += 0.02;
            master_L0_exp += 0.02;
            slave_L0_exp += 0.02;
            if (master_L0_exp > 0.38)
            {
                master_L0_exp = 0.38;
            }
            if (slave_L0_exp > 0.38)
            {
                slave_L0_exp = 0.38;
            }
            // master_l_z_exp += 0.02;
            // slave_l_z_exp += 0.02;
            // if (master_l_z_exp > 0.38)
            // {
            //     master_l_z_exp = 0.38;
            // }
            // if (slave_l_z_exp > 0.38)
            // {
            //     slave_l_z_exp = 0.38;
            // }
            // master_r_z_exp += 0.02;
            // slave_r_z_exp += 0.02;
            // if (master_r_z_exp > 0.38)
            // {
            //     master_r_z_exp = 0.38;
            // }
            // if (slave_r_z_exp > 0.38)
            // {
            //     slave_r_z_exp = 0.38;
            // }
        }
        break;
    case 'E': // E变矮
        if (robotstate == NORM)
        {
            if (lleg->L0_exp > 0.09 && rleg->L0_exp > 0.09)
            {
                lleg->L0_exp -= 0.01;
                rleg->L0_exp -= 0.01;
            }
            ROS_WARN("E,L0_exp=%f\n", lleg->L0_exp * 0.5 + rleg->L0_exp * 0.5);
        }
        else if (robotstate == COMB)
        { // 0.5度一调整
            // llegphi1 += 400;
            // llegphi4 -= 400;
            // rlegphi1 -= 400;
            // rlegphi4 += 400;
            master_L0_exp -= 0.02;
            slave_L0_exp -= 0.02;
            if (master_L0_exp < 0.09)
            {
                master_L0_exp = 0.09;
            }
            if (slave_L0_exp < 0.09)
            {
                slave_L0_exp = 0.09;
            }
            // master_l_z_exp -= 0.02;
            // slave_l_z_exp -= 0.02;
            // if (master_l_z_exp < 0.1)
            // {
            //     master_l_z_exp = 0.1;
            // }
            // if (slave_l_z_exp < 0.1)
            // {
            //     slave_l_z_exp = 0.1;
            // }
            // master_r_z_exp -= 0.02;
            // slave_r_z_exp -= 0.02;
            // if (master_r_z_exp < 0.1)
            // {
            //     master_r_z_exp = 0.1;
            // }
            // if (slave_r_z_exp < 0.1)
            // {
            //     slave_r_z_exp = 0.1;
            // }
        }
        break;
    case '1': // turn left in R
        R = 0.8;
        w = 0.2;
        delta_x = a * c_orignal / R;

        slave_dx_l = COMB_turn_l(R, w, a, c_orignal - delta_x) / radius;
        master_dx_l = (-1) * COMB_turn_l(R, w, a, c_orignal - delta_x) / radius;
        slave_l_x_exp = L5 / 2 - delta_x;
        master_l_x_exp = L5 / 2 + delta_x;

        slave_dx_r = COMB_turn_r(R, w, a, c_orignal + delta_x) / radius;
        master_dx_r = (-1) * COMB_turn_r(R, w, a, c_orignal + delta_x) / radius;
        slave_r_x_exp = L5 / 2 + delta_x;
        master_r_x_exp = L5 / 2 - delta_x;

        COMB_roll_exp = roll - 7;
        break;
    case '2': // turn right in R
        R = 0.8;
        w = -0.2;
        delta_x = a * c_orignal / R;

        slave_dx_l = COMB_turn_l(R, w, a, c_orignal + delta_x) / radius;
        master_dx_l = (-1) * COMB_turn_l(R, w, a, c_orignal + delta_x) / radius;
        slave_l_x_exp = L5 / 2 + delta_x;
        master_l_x_exp = L5 / 2 - delta_x;

        slave_dx_r = COMB_turn_r(R, w, a, c_orignal - delta_x) / radius;
        master_dx_r = (-1) * COMB_turn_r(R, w, a, c_orignal - delta_x) / radius;
        slave_r_x_exp = L5 / 2 - delta_x;
        master_r_x_exp = L5 / 2 + delta_x;

        COMB_roll_exp = 7;
        break;
    case '3':
        // master_theta_exp -= 2 * pi / 180;
        // if (master_theta_exp < 0)
        // {
        //     master_theta_exp = 0;
        // }
        COMB_roll_exp += 2;
        // master_l_x_exp += 0.02;
        // if (master_l_x_exp > 0.25)
        // {
        //     master_l_x_exp = 0.25;
        // }
        break;
    case '4':
        // master_theta_exp += 2 * pi / 180;
        // if (master_theta_exp > pi)
        // {
        //     master_theta_exp = pi;
        // }
        COMB_roll_exp -= 2;
        // master_l_x_exp -= 0.02;
        // if (master_l_x_exp < -0.13)
        // {
        //     master_l_x_exp = -0.13;
        // }
        break;
    case '5':
        // master_theta_exp -= 2 * pi / 180;
        // if (master_theta_exp < 0)
        // {
        //     master_theta_exp = 0;
        // }
        COMB_pitch_exp += 2;
        // master_r_x_exp += 0.02;
        // if (master_r_x_exp > 0.25)
        // {
        //     master_r_x_exp = 0.25;
        // }
        break;
    case '6':
        // master_theta_exp += 2 * pi / 180;
        // if (master_theta_exp > pi)
        // {
        //     master_theta_exp = pi;
        // }
        COMB_pitch_exp -= 2;
        // master_r_x_exp -= 0.02;
        // if (master_r_x_exp < -0.13)
        // {
        //     master_r_x_exp = -0.13;
        // }
        break;
    case '7':
        // master_theta_exp -= 2 * pi / 180;
        // if (master_theta_exp < 0)
        // {
        //     master_theta_exp = 0;
        // }
        R = 0.5;
        w = 0.2;
        delta_x = a * c_orignal / R;

        slave_dx_l = (-1) * COMB_turn_l(R, w, a, c_orignal - delta_x) / radius;
        master_dx_l = COMB_turn_l(R, w, a, c_orignal - delta_x) / radius;
        slave_l_x_exp = L5 / 2 - delta_x;
        master_l_x_exp = L5 / 2 + delta_x;

        slave_dx_r = (-1) * COMB_turn_r(R, w, a, c_orignal + delta_x) / radius;
        master_dx_r = COMB_turn_r(R, w, a, c_orignal + delta_x) / radius;
        slave_r_x_exp = L5 / 2 + delta_x;
        master_r_x_exp = L5 / 2 - delta_x;

        COMB_roll_exp = -10;
        // slave_l_x_exp += 0.02;
        // if (slave_l_x_exp > 0.25)
        // {
        //     slave_l_x_exp = 0.25;
        // }
        break;
    case '8':
        // master_theta_exp += 2 * pi / 180;
        // if (master_theta_exp > pi)
        // {
        //     master_theta_exp = pi;
        // }
        R = 0.5;
        w = -0.2;
        delta_x = a * c_orignal / R;

        slave_dx_l = (-1) * COMB_turn_l(R, w, a, c_orignal + delta_x) / radius;
        master_dx_l = COMB_turn_l(R, w, a, c_orignal + delta_x) / radius;
        slave_l_x_exp = L5 / 2 + delta_x;
        master_l_x_exp = L5 / 2 - delta_x;

        slave_dx_r = (-1) * COMB_turn_r(R, w, a, c_orignal - delta_x) / radius;
        master_dx_r = COMB_turn_r(R, w, a, c_orignal - delta_x) / radius;
        slave_r_x_exp = L5 / 2 - delta_x;
        master_r_x_exp = L5 / 2 + delta_x;

        COMB_roll_exp = 10;
        // slave_l_x_exp -= 0.02;
        // if (slave_l_x_exp < -0.13)
        // {
        //     slave_l_x_exp = -0.13;
        // }
        break;
    case '9':
        // master_theta_exp -= 2 * pi / 180;
        // if (master_theta_exp < 0)
        // {
        //     master_theta_exp = 0;
        // }
        // slave_r_x_exp += 0.02;
        // if (slave_r_x_exp > 0.25)
        // {
        //     slave_r_x_exp = 0.25;
        // }
        COMB_roll_exp = (-1) * roll;
        break;
    case '0':
        // master_theta_exp += 2 * pi / 180;
        // if (master_theta_exp > pi)
        // {
        //     master_theta_exp = pi;
        // }
        // slave_r_x_exp -= 0.02;
        // if (slave_r_x_exp < -0.13)
        // {
        //     slave_r_x_exp = -0.13;
        // }
        COMB_pitch_exp = (-1) * pitch;
        break;
    case 'C':
        isRobotSafe = false; // 这个会C掉所有节点，在keyboard函数里也有处理
        break;
    case 'R':
        if (robotstate == INIT)
        {
            robotstate = COMB;
            slave_flag = 1;
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
        // car_break=1;
        /* rleg->speed_last = rleg->dx;
         lleg->speed_last = lleg->dx;
         ma_T1 = (0 - rleg->dx) * 0.01+(rleg->dx - rleg->speed_last)*0.3;
         ma_T2 = (0 - lleg->dx) * 0.05+(lleg->dx - lleg->speed_last)*0.3;
         ROS_INFO("zuo轮%.2f", ma_T1);
         ROS_INFO("右轮%.2f", ma_T2);
         if(abs(ma_T1-0)<0.04&&abs(ma_T2-0)<0.04){
             ma_T1=0;
             ma_T2=0;
         };*/
        master_dx_l = 0;
        master_dx_r = 0;
        slave_dx_l = 0;
        slave_dx_r = 0;
        // master_theta_exp = pi / 2;
        // slave_theta_exp = pi / 2;
        // master_L0_exp = 0.15;
        // slave_L0_exp = 0.15;
        master_l_x_exp = L5 / 2;
        // master_L0_exp = 0.15;
        slave_l_x_exp = L5 / 2;
        // slave_L0_exp = 0.15;
        master_r_x_exp = L5 / 2;
        slave_r_x_exp = L5 / 2;
        COMB_roll_exp = 0;
        COMB_pitch_exp = 0;
        // ros::Duration(6.0).sleep();

        // car_break=0;
        // ma_T1=0;
        // ma_T2=0;

        /*   if(ma_T1)
           ma_T1=0;
           ma_T2=0;*/
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

    if (robotstate == NORM)
    {
        //[左区按键]上/下箭头，左正右负
        lleg->dx_exp += 0.3 * Joy->axes[7];
        rleg->dx_exp += 0.3 * Joy->axes[7];
        //[左区按键]左/右箭头，左正右负
        yaw_exp += 3 * Joy->axes[6];
        lleg->x_exp -= 3 * Joy->axes[6] * pi / 180 * half_wheel_d;
        rleg->x_exp += 3 * Joy->axes[6] * pi / 180 * half_wheel_d;
        //[左区按键]L1升高，R1降低
        lleg->L0_exp += 0.01 * (Joy->buttons[6] - Joy->buttons[7]);
        rleg->L0_exp += 0.01 * (Joy->buttons[6] - Joy->buttons[7]);
        lleg->dx_exp += 0.01 * Joy->axes[1];
        rleg->dx_exp += 0.01 * Joy->axes[1];
        yaw_exp += 0.05 * Joy->axes[0];
        lleg->x_exp -= 0.05 * Joy->axes[0] * pi / 180 * half_wheel_d;
        rleg->x_exp += 0.05 * Joy->axes[0] * pi / 180 * half_wheel_d;
    }
    else if (robotstate == COMB)
    { // forward
        //[左区按键]上/下箭头，左正右负
        dx_run0 = 0.3 * Joy->axes[1];   // joy
        dx_rund += 0.05 * Joy->axes[7]; // key

        // Old turning structure before 20241012
        // R = 1.5 + 1 * (1 - abs(Joy->axes[0]));
        // V = dx_run0 + dx_rund + 0.00000001; // speed of center of the co-robot
        // w = V * sign(Joy->axes[0]) / R;     // turning speed
        // delta_x = a * c_orignal * sign(Joy->axes[0]) / R;
        // master_dx_l_run = (V - a * w + ((c_orignal - delta_x) * (c_orignal - delta_x) * w * w) / (V - a * w)) / radius;
        // master_dx_r_run = (V + a * w + ((c_orignal + delta_x) * (c_orignal + delta_x) * w * w) / (V + a * w)) / radius;
        // slave_dx_l_run = (V - a * w + ((c_orignal - delta_x) * (c_orignal - delta_x) * w * w) / (V - a * w)) / radius;
        // slave_dx_r_run = (V + a * w + ((c_orignal + delta_x) * (c_orignal + delta_x) * w * w) / (V + a * w)) / radius;

        // New turning structure on 20241012
        R_b = 1.65 + 1 * (1 - abs(Joy->axes[0]));
        R_f = sqrt(turn_L2 * turn_L2 + R_b * R_b - turn_L1 * turn_L1);
        V = dx_run0 + dx_rund;
        w = V / R_b;
        double vel_b, vel_f;
        a = 1 - abs(sign(Joy->axes[0]));
        b = abs(sign(Joy->axes[0]));
        c = sign(Joy->axes[0]);

        d = 1 - abs(Joy->axes[6]);

        // master_dx_l_run = (R_b - sign(Joy->axes[0]) * half_wheel_d) * w / radius;
        // master_dx_r_run = (R_b + sign(Joy->axes[0]) * half_wheel_d) * w / radius;
        // slave_dx_l_run = (R_f - sign(Joy->axes[0]) * half_wheel_d) * w / radius;
        // slave_dx_r_run = (R_f + sign(Joy->axes[0]) * half_wheel_d) * w / radius;

        // // master_dx_l_run -= 0.05 * Joy->axes[7] / radius + 0.05 * Joy->axes[1] / radius;
        //// master_dx_r_run -= 0.05 * Joy->axes[7] / radius + 0.05 * Joy->axes[1] / radius;
        //// slave_dx_l_run += 0.05 * Joy->axes[7] / radius + 0.05 * Joy->axes[1] / radius;
        //// slave_dx_r_run += 0.05 * Joy->axes[7] / radius + 0.05 * Joy->axes[1] / radius;

        // master_dx_l_run *= 1 - abs(Joy->axes[6]);
        // master_dx_r_run *= 1 - abs(Joy->axes[6]);
        // slave_dx_l_run *= 1 - abs(Joy->axes[6]);
        // slave_dx_r_run *= 1 - abs(Joy->axes[6]);

        // master_dx_l_run *= -1;
        // master_dx_r_run *= -1;
        //[左区按键]左/右箭头，左正右负
        master_dx_l_turn += 0.1 * Joy->axes[6];
        master_dx_r_turn -= 0.1 * Joy->axes[6];
        slave_dx_l_turn -= 0.1 * Joy->axes[6];
        slave_dx_r_turn += 0.1 * Joy->axes[6];

        master_dx_l_turn *= 1 - abs(Joy->axes[7]);
        master_dx_r_turn *= 1 - abs(Joy->axes[7]);
        slave_dx_l_turn *= 1 - abs(Joy->axes[7]);
        slave_dx_r_turn *= 1 - abs(Joy->axes[7]);
        // master_L0_exp = 0.22291;
        // slave_L0_exp = 0.22291;
        // master_theta_exp = pi / 2 - 30 * pi / 180;
        // slave_theta_exp = pi / 2 + 30 * pi / 180;
        delta_master_l_xx_exp = abs(master_dx_l_turn) / 10 + delta_x;
        delta_slave_l_xx_exp = -abs(master_dx_l_turn) / 10 - delta_x;
        delta_master_r_xx_exp = abs(master_dx_l_turn) / 10 - delta_x;
        delta_slave_r_xx_exp = -abs(master_dx_l_turn) / 10 + delta_x;

        // delta_master_l_xx_exp = 0 + delta_x;
        // delta_slave_l_xx_exp = 0 - delta_x;
        // delta_master_r_xx_exp = 0 - delta_x;
        // delta_slave_r_xx_exp = 0 + delta_x;
        //[左区按键]L1升高，R1降低
        master_L0_exp += 0.02 * (Joy->buttons[4] - Joy->buttons[5]);
        slave_L0_exp += 0.02 * (Joy->buttons[4] - Joy->buttons[5]);
        if (master_L0_exp > 0.32)
        {
            master_L0_exp = 0.32;
        }
        if (slave_L0_exp > 0.32)
        {
            slave_L0_exp = 0.32;
        }
        if (master_L0_exp < 0.09)
        {
            master_L0_exp = 0.09;
        }
        if (slave_L0_exp < 0.09)
        {
            slave_L0_exp = 0.09;
        }

        // COMB_roll_exp = 1 * asin(master_L0_exp * (-1) * sign(Joy->axes[0]) / R) * 180 / pi - 6 * Joy->axes[3];
        COMB_roll_exp = -6 * Joy->axes[3];
        COMB_pitch_exp = 6 * Joy->axes[4];
    }

    //[左区按键]L2急刹
    if (Joy->axes[2] == -1)
    {
        if (robotstate == NORM)
        {
            lleg->dx_exp = 0;
            rleg->dx_exp = 0;
        }
        else if (robotstate == COMB)
        {
            master_dx_l_run0 = 0;
            master_dx_r_run0 = 0;
            slave_dx_l_run0 = 0;
            slave_dx_r_run0 = 0;
            master_dx_l_rund = 0;
            master_dx_r_rund = 0;
            slave_dx_l_rund = 0;
            slave_dx_r_rund = 0;
            master_dx_l_turn = 0;
            master_dx_r_turn = 0;
            slave_dx_l_turn = 0;
            slave_dx_r_turn = 0;
            dx_run0 = 0;
            dx_rund = 0;
            V = 0;
            // master_theta_exp = pi / 2;
            // slave_theta_exp = pi / 2;
            // master_L0_exp = 0.15;
            // slave_L0_exp = 0.15;

            // master_l_xx_exp = L5 / 2;
            // slave_l_xx_exp = L5 / 2;
            // master_r_xx_exp = L5 / 2;
            // slave_r_xx_exp = L5 / 2;

            delta_master_l_xx_exp = 0;
            delta_slave_l_xx_exp = 0;
            delta_master_r_xx_exp = 0;
            delta_slave_r_xx_exp = 0;

            COMB_roll_exp = 0;
            COMB_pitch_exp = 0;
        }
    }
    // A
    if (Joy->buttons[0] == 1)
    {
        if (robotstate == NORM)
        {
        }
        else if (robotstate == COMB)
        {
            COMB_mode = 1;
            master_dx_l_run0 = 0;
            master_dx_r_run0 = 0;
            slave_dx_l_run0 = 0;
            slave_dx_r_run0 = 0;
            master_dx_l_rund = 0;
            master_dx_r_rund = 0;
            slave_dx_l_rund = 0;
            slave_dx_r_rund = 0;
            master_dx_l_turn = 0;
            master_dx_r_turn = 0;
            slave_dx_l_turn = 0;
            slave_dx_r_turn = 0;
            dx_run0 = 0;
            dx_rund = 0;
            V = 0;
            delta_master_l_xx_exp = 0;
            delta_slave_l_xx_exp = 0;
            delta_master_r_xx_exp = 0;
            delta_slave_r_xx_exp = 0;
            delta_master_l_xx_exp = 0;
            delta_slave_l_xx_exp = 0;
            delta_master_r_xx_exp = 0;
            delta_slave_r_xx_exp = 0;
            COMB_roll_exp = 0;
            COMB_pitch_exp = 0;

            // isRobotSafe = false;
        }
    }
    // B
    if (Joy->buttons[1] == 1)
    {
        if (robotstate == NORM)
        {
        }
        else if (robotstate == COMB)
        {
            theta_turn_exp = 0;
        }
    }
    // X
    if (Joy->buttons[2] == 1)
    {
        if (robotstate == NORM)
        {
        }
        else if (robotstate == COMB)
        {
            trot_flag = 0;
        }
    }
    // Y
    if (Joy->buttons[3] == 1)
    {
        if (robotstate == NORM)
        {
            // 蹲下
        }
        else if (robotstate == COMB)
        {
            COMB_mode = 0;
            master_dx_l_run0 = 0;
            master_dx_r_run0 = 0;
            slave_dx_l_run0 = 0;
            slave_dx_r_run0 = 0;
            master_dx_l_rund = 0;
            master_dx_r_rund = 0;
            slave_dx_l_rund = 0;
            slave_dx_r_rund = 0;
            master_dx_l_turn = 0;
            master_dx_r_turn = 0;
            slave_dx_l_turn = 0;
            slave_dx_r_turn = 0;
            dx_run0 = 0;
            dx_rund = 0;
            V = 0;
            // master_l_xx_exp = L5 / 2;
            master_L0_exp = 0.09;
            // slave_l_xx_exp = L5 / 2;
            slave_L0_exp = 0.09;
            // master_r_xx_exp = L5 / 2;
            // slave_r_xx_exp = L5 / 2;
            delta_master_l_xx_exp = 0;
            delta_slave_l_xx_exp = 0;
            delta_master_r_xx_exp = 0;
            delta_slave_r_xx_exp = 0;
            COMB_roll_exp = 0;
            COMB_pitch_exp = 0;

            // isRobotSafe = false;
        }
    }
    // R2
    if (Joy->axes[5] == -1)
    {
        if (robotstate == NORM)
        {
            lleg->dx_exp = 0;
            rleg->dx_exp = 0;
        }
        else if (robotstate == COMB)
        {
        }
    }
    // SELECT
    if (Joy->buttons[6] == 1)
    {
        if (robotstate == INIT)
        {
            robotstate = COMB;
            slave_flag = 1;
        }
    }
    // MODE
    if (Joy->buttons[8] == 1)
    {
        if (robotstate == INIT)
        {
            robotstate = NORM;
        }
    }
}

int VMC::sign(double x) // 符号函数
{
    int s;
    if (x > 0)
    {
        s = 1;
    }
    if (x < 0)
    {
        s = -1;
    }
    if (x == 0)
    {
        s = 0;
    }
    return s;
}

void VMC::Serial_feedback(const message::rs232_elec_mag_state::ConstPtr &msg)
{
    mag_leftfront = msg->state1;
    mag_rightfront = msg->state2;
    mag_leftback = msg->state3;
    mag_rightback = msg->state4;

    mag_leftfront = 1;  // 临时使用固定数值用于拼接控制
    mag_rightfront = 1; // 2024/8/3临时使用的

    // ROS_WARN("mag_leftfront: %d",mag_leftfront);
    // ROS_WARN("mag_rightfront: %d",mag_rightfront);
    // ROS_WARN("mag_leftback: %d",mag_leftback);
    // ROS_WARN("mag_rightback: %d",mag_rightback);
    // mag_leftback = msg->state3;
    // mag_rightback = msg->state4;
}

void VMC::Slave_Serial_feedback(const message::rs232_elec_mag_state::ConstPtr &msg)
{
    mag_slave_left_back = msg->state3;
    mag_slave_right_back = msg->state4;

    mag_slave_left_back = 1;  // 临时使用固定数值用于拼接控制
    mag_slave_right_back = 1; // 2024/8/3临时使用的

    // ROS_WARN("mag_slave_left_front: %d",msg->state1);
    // ROS_WARN("mag_slave_right_front: %d",msg->state2);
    // ROS_WARN("mag_slave_left_back: %d",mag_slave_left_back);
    // ROS_WARN("mag_slave_right_back: %d",mag_slave_right_back);
}

void VMC::Slave_IMU_Input(const message::imu_controller::ConstPtr &msg)
{
    imu_s = *msg;
    imu_s_pre = imu_s;
    yaw_s = imu_s.angle_yaw;
    counter_test++;
}