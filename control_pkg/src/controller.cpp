#include "controller.h"

VMC::VMC() // 初始化发布者、订阅者、pid等
{
    robotstate = INIT;
    // 定义话题订阅者 NORM
    motor_input_sub = n.subscribe("/motor_feedback", 1, &VMC::Motor_Input, this); // 接收电机反馈 只接受最新的
    imu_sub = n.subscribe("/imu_topic", 1, &VMC::IMU_Input, this);                // 接受IMU信息
    key_sub = n.subscribe("/key_command", 10, &VMC::Keyboard_Input, this);        // 接受键盘输入
    joy_sub = n.subscribe("/joy", 10, &VMC::Joy, this);                           // 接受手柄输入
    // plan_sub = n.subscribe("cmd_vel", 10, &VMC::Plan, this);                     // 接受规划输入
    plan_sub = n.subscribe("/cmd_vel", 10, &VMC::Plan, this); // 接受规划输入
    // 此处需要订阅232串口的信息 COMB
    serial_sub = n.subscribe("/mag_state", 10, &VMC::Serial_feedback, this);
    slave_serial_sub = n.subscribe("/robot2/mag_state", 10, &VMC::Slave_Serial_feedback, this);
    slave_imu_sub = n.subscribe("/robot2/imu_topic", 10, &VMC::Slave_IMU_Input, this);
    // 定义话题发布者
    // NORM
    vmc_output_pub = n.advertise<message::vmc_interface>("/vmc_output", 10); // 发送VMC控制器输出至接口
    lqr_state_pub = n.advertise<message::lqr_state>("/lqr_state", 10);       // 发送VMC控制器输出至接口
    pid_state_pub = n.advertise<message::pid_state>("/pid_state", 10);       // 发送VMC控制器输出至接口
    // COMB
    cmb_output_pub = n.advertise<message::cmb_interface>("/cmb_master", 10);         // 发送后面主机需要调整的信息,给本机节点
    cmb_slave_output_pub = n.advertise<message::cmb_slave>("/robot2/motor_adj", 10); // 发送前面从机需要调整的信息，给udp_pkg

    // 腿部结构体初始化
    lleg = (struct Leg_Model *)malloc(sizeof(struct Leg_Model)); // 单体 NORM leg
    rleg = (struct Leg_Model *)malloc(sizeof(struct Leg_Model)); // 单体 NORM leg
    leg_init(rleg);                                              // 单体
    leg_init(lleg);                                              // 单体
    lleg->dL0 = new TimeVar(TIME_STEP);                          // 单体
    rleg->dL0 = new TimeVar(TIME_STEP);                          // 单体
    lleg->dTheta = new TimeVar(TIME_STEP);                       // 单体
    rleg->dTheta = new TimeVar(TIME_STEP);                       // 单体
    // PID初始化
    theta_pid = new PID(1 / CONTROL_RATE, theta_pid_index, 0, 10, yaw);                             // NORM
    turn_pid = new PID(1 / CONTROL_RATE, turn_pid_index, 0, 3, 0);                                  // NORM
    roll_pid = new PID(1 / CONTROL_RATE, roll_pid_index, 0, 100, roll_exp);                         // NORM
    COMB_roll_pid = new PID(1 / CONTROL_RATE, COMB_roll_pid_index, 0, 100, COMB_roll_exp);          // COMB
    COMB_pitch_pid = new PID(1 / CONTROL_RATE, COMB_pitch_pid_index, 0, 100, COMB_pitch_exp);       // COMB
    COMB_turn_pid_m = new PID(1 / CONTROL_RATE, COMB_turn_pid_m_index, 0, 5, theta_turn_exp);       // COMB
    COMB_turn_pid_s = new PID(1 / CONTROL_RATE, COMB_turn_pid_s_index, 0, 5, theta_turn_exp);       // COMB
    COMB_recover_pid_m = new PID(1 / CONTROL_RATE, COMB_recover_pid_m_index, 0, 5, theta_turn_exp); // COMB
    COMB_recover_pid_s = new PID(1 / CONTROL_RATE, COMB_recover_pid_s_index, 0, 5, theta_turn_exp); // COMB
    plan_v = 0;                                                                                     // NORM
    plan_w = 0;                                                                                     // NORM
    roll = 0;                                                                                       // NORM
    roll_exp = 0;                                                                                   // NORM
    pitch = 0;                                                                                      // NORM
    yaw = 0;                                                                                        // NORM
    yaw_exp = 0;                                                                                    // NORM
    droll = 0;                                                                                      // NORM
    dpitch = 0;                                                                                     // NORM
    dyaw = 0;                                                                                       // NORM
    counter_plan = 0;                                                                               // NORM
    test = turn_pid;                                                                                // NORM
    isPlan = false;                                                                                 // NORM
    isRobotSafe = true;                                                                             // NORM
    isRobotJumping = false;                                                                         // NORM
    isYawInit = false;                                                                              // NORM
    counter_P = 0;                                                                                  // NORM
    isTestMode = true;                                                                              // NORM
    n.getParam("vmc_node/isTestMode", isTestMode);                                                  // NORM
    yawfilter = new filter();                                                                       // NORM
    fai_b = atan(half_wheel_d / turn_L2);                                                           // COMB
    fai_f = atan(half_wheel_d / turn_L1);                                                           // COMB

    // COMB Electromagnet 电磁铁
    // Master 主机
    mag_leftback = 0;
    mag_leftfront = 0;
    mag_rightback = 0;
    mag_rightfront = 0;
    // Slave 从机
    mag_slave_left_back = 0;
    mag_slave_right_back = 0;
    // IMU data fusion by LIU Hao IMU数据融合
    pitch_fuse = 0;
    roll_fuse = 0;
    // L0+theta to control phi1/4
    // 拼接模式下初始状态
    master_L0_exp = 0.15;     // 主机腿长（轮轴到髋关节中点的距离）
    slave_L0_exp = 0.15;      // 从机腿长（轮轴到髋关节中点的距离）
    master_l_xx_exp = L5 / 2; // 拼接【主机】的【左腿】沿【x】的期望
    slave_l_xx_exp = L5 / 2;  // 拼接【从机】的【左腿】沿【x】的期望
    master_r_xx_exp = L5 / 2; // 拼接【主机】的【右腿】沿【x】的期望
    slave_r_xx_exp = L5 / 2;  // 拼接【从机】的【右腿】沿【x】的期望
    // xyz to control phi1/4
    // roll_flag = 1;
    // pitch_flag = 1;
    t = 0;
    COMB_roll_exp = 0.0;       // 拼接 roll期望
    COMB_pitch_exp = 0.0;      // 拼接 pitch期望
    delta_master_l_z_exp1 = 0; // 拼接【主机】的【左腿】沿【z】的，COMB_pitch_exp变化时导致的腿长变化
    delta_master_r_z_exp1 = 0; // 拼接【主机】的【右腿】沿【z】的，COMB_pitch_exp变化时导致的腿长变化
    delta_slave_l_z_exp1 = 0;  // 拼接【从机】的【左腿】沿【z】的，COMB_pitch_exp变化时导致的腿长变化
    delta_slave_r_z_exp1 = 0;  // 拼接【从机】的【右腿】沿【z】的，COMB_pitch_exp变化时导致的腿长变化
    master_dx_l = 0;           // 拼接【主机】的【左轮】的速度
    master_dx_r = 0;           // 拼接【主机】的【右轮】的速度
    slave_dx_l = 0;            // 拼接【从机】的【左轮】的速度
    slave_dx_r = 0;            // 拼接【从机】的【右轮】的速度
    master_dx_l_turn = 0;      // 拼接【主机】的【左轮】的速度 中的 【原地转弯速度】
    master_dx_r_turn = 0;      // 拼接【主机】的【右轮】的速度 中的 【原地转弯速度】
    slave_dx_l_turn = 0;       // 拼接【从机】的【左轮】的速度 中的 【原地转弯速度】
    slave_dx_r_turn = 0;       // 拼接【从机】的【右轮】的速度 中的 【原地转弯速度】
    master_dx_l_run = 0;       // 拼接【主机】的【左轮】的速度 中的 【直线行进速度】
    master_dx_r_run = 0;       // 拼接【主机】的【右轮】的速度 中的 【直线行进速度】
    slave_dx_l_run = 0;        // 拼接【从机】的【左轮】的速度 中的 【直线行进速度】
    slave_dx_r_run = 0;        // 拼接【从机】的【右轮】的速度 中的 【直线行进速度】
    // dx_run0 = 0;
    // dx_rund = 0;
    delta_x = 0; // 转向时腿沿【x】的变化
    // ma_T1 = 0;
    // ma_T2 = 0;
    R = 1; // 转弯半径
    // slave_flag = 0;
    car_break = 0;
    walk_m_l = 0;   // 拼接【主机】的【左腿】的步态时腿长变化量
    walk_m_r = 0;   // 拼接【主机】的【右腿】的步态时腿长变化量
    walk_s_l = 0;   // 拼接【从机】的【左腿】的步态时腿长变化量
    walk_s_r = 0;   // 拼接【从机】的【右腿】的步态时腿长变化量
    walk_flag = 0;  // 是否步态的标志
    count_walk = 0; // 步态时序
}

void leg_init(Leg_Model *leg) // 初始化腿部结构体
{
    // 所有量默认为0
    memset(leg, 0, sizeof(struct Leg_Model));
    // 类或结构体初始化
    leg->L0_pid = new PID(1 / CONTROL_RATE, l0_pid_index, 0, 100, leg->L0_exp);
    leg->COMB_z_pid = new PID(1 / CONTROL_RATE, COMB_z_pid_index, 0, 100, leg->COMB_z_exp);
    leg->COMB_x_pid = new PID(1 / CONTROL_RATE, COMB_x_pid_index, 0, 100, leg->COMB_x_exp);
    leg->dTheta = new TimeVar(TIME_STEP);
    leg->dL0 = new TimeVar(TIME_STEP);
    // 不为0的量在以下初始化
    leg->isRobotOffGround = false;
    leg->phi0 = pi / 2;
    // leg->phi1 = pi;
    leg->L0_exp = 0.15;
    leg->x_exp = 0;
    leg->COMB_x_exp = L5 / 2;
    leg->COMB_z_exp = 0.15;
    leg->L0 = 0.09;
    leg->F = m_body / 2 * g;
    // leg->theta_filter=new filter();
}

void VMC::state_print()
{
    // pid状态输出
    pid_state_pub.publish(test->print());
    // printf("[LQR输入右]:\ntheta = %.2grad, x = %.2gm, phi = %.2grad,\ndtheta = %.2grad, dx = %.2gm/s, dphi = %.2grad/s\n", rleg->theta, rleg->x, rleg->phi, rleg->dtheta, rleg->dx, rleg->dphi);
    // printf("[LQR输入左]:\ntheta = %.2grad, x = %.2gm, phi = %.2grad,\ndtheta = %.2grad, dx = %.2gm/s, dphi = %.2grad/s\n", lleg->theta, lleg->x, lleg->phi, lleg->dtheta, lleg->dx, lleg->dphi);
    printf("[LQR输出右]:\nT = %.2gN*m, T1 = %.2gN*m, T2 = %.2gN*m\n", rleg->T, rleg->T1, rleg->T2);
    printf("[LQR输出左]:\nT = %.2gN*m, T1 = %.2gN*m, T2 = %.2gN*m\n", lleg->T, lleg->T1, lleg->T2);
    printf("[[[[ F_l = %f, F_r = %f ]]]]\n", lleg->F, rleg->F);
    // printf("[差速转向]:\n左 = %.2gm/s, 右 = %.2gm/s\n", lleg->dx_exp, rleg->dx_exp);
    // printf("*F: 左 = %f, 右 = %f\n", lleg->F, rleg->F);
    // printf("*l_exp = %f, l = %f\n", lleg->L0_exp, lleg->L0);
    // printf("[右]x_err = %f\n", -rleg->x_exp + rleg->x);
    // printf("[左]x_er/r = %f\n", -lleg->x_exp + lleg->x);

    message::lqr_state lqr_msg;
    lqr_msg.theta = rleg->theta;
    lqr_msg.dtheta = rleg->dtheta;
    lqr_msg.x = rleg->x;
    lqr_msg.dx = rleg->dx;
    lqr_msg.x_exp = rleg->x_exp;
    lqr_msg.dx_exp = rleg->dx_exp;
    lqr_msg.phi = rleg->phi;
    lqr_msg.dphi = rleg->dphi;
    lqr_msg.height = rleg->L0;
    lqr_msg.height_exp = rleg->L0_exp;
    lqr_msg.v = (rleg->dx + lleg->dx) / 2;
    lqr_msg.w = dyaw;
    lqr_msg.header.stamp = ros::Time::now();
    lqr_msg.yaw = yaw;
    lqr_msg.yaw_exp = (int)yaw_exp % 360;
    // ROS_WARN("控制周期:%d", counter_P);
    // ROS_WARN("电机回调次数:%d", counter_Motor);
    // ROS_WARN("IMU回调次数:%d", counter_IMU);
    // endTime_control = ros::Time::now().toNSec();

    // std::cout << "控制计算用时:" << (endTime_control - startTime_control) / 1e6 << "ms" << std::endl;

    // std::cout << "已开始控制:" << (all_T - init_T) / 1e9 << "s" << std::endl;
    lqr_state_pub.publish(lqr_msg);
}

void VMC::turn_control() // 要求：yaw、yaw_exp 功能：使exp在360度上取模 并施加转向控制
{
    double T_turn;
    double exp = (int)yaw_exp % 360;
    double fdb = yaw;
    double dis = abs(exp - fdb);
    if (dis <= 180) // exp与fdb未经过正负180度线
    {
        if (exp >= fdb)
        {
            exp = fdb + dis;
        }
        else
        {
            exp = fdb - dis;
        }
    }
    else // exp与fdb经过了正负180度线
    {
        dis = 360 - dis;
        if (exp >= fdb)
        {
            exp = fdb - dis;
        }
        else
        {
            exp = fdb + dis;
        }
    }
    turn_pid->exp_set(exp);
    turn_pid->update(fdb);
    T_turn = turn_pid->control();
    if (!lleg->isRobotOffGround && !rleg->isRobotOffGround)
    {
        rleg->T += T_turn;
        lleg->T -= T_turn;
    }
    printf("[转向环]: \ndis=%f,yaw_exp=%f,yaw=%f,control=%f\n", dis, exp, fdb, T_turn);
    // double T_turn;
    // turn_pid->exp_set(rleg->x_exp - lleg->x_exp);
    // turn_pid->update(rleg->x - lleg->x);
    // T_turn = turn_pid->control();
    // if (!lleg->isRobotOffGround && !rleg->isRobotOffGround)
    // {
    //     rleg->T += T_turn;
    //     lleg->T -= T_turn;
    // }
}

double VMC::nl_calc(Leg_Model *leg) // 腿部质心位置系数计算函数
{
    double l = leg->L0;
    double l2 = pow(l, 2);
    double l3 = pow(l, 3);
    double l4 = pow(l, 4);
    double l5 = pow(l, 5);
    double l6 = pow(l, 6);
    // return 423.6 * l4 - 456.93 * l3 + 183.94 * l2 - 33.22 * l + 2.8118;//old
    return 98143 * l6 - 145520 * l5 + 87428 * l4 - 27207 * l3 + 4626.5 * l2 - 409.49 * l + 15.354; // new al knee
}

double VMC::leg_length_limit(Leg_Model *leg)
{
    if (leg->COMB_z_exp < lowest_leg_length)
    {
        return lowest_leg_length;
    }
    else if (leg->COMB_z_exp > highest_leg_length)
    {
        return highest_leg_length;
    }
    else
    {
        return leg->COMB_z_exp;
    }
}

void VMC::COMB_leg_length_control() // 【拼接】模式下腿长 力位控制
{
    // 参照舵狗王算法：https://zhuanlan.zhihu.com/p/69869440
    COMB_roll_pid->exp_set(COMB_roll_exp);
    COMB_roll_pid->update(roll);
    double pitch_fdb;
    if (climb_flag == 1) // 机身平行斜坡爬坡，相对关系
    {
        pitch_fdb = atan((master_L0_exp - slave_L0_exp) / wheel_d);
    }
    else // 机身平行水平面爬坡，绝对关系
    {
        pitch_fdb = pitch;
    }
    COMB_pitch_pid->exp_set(COMB_pitch_exp);
    COMB_pitch_pid->update(pitch_fdb);
    F_roll = COMB_roll_pid->control();
    F_pitch = COMB_pitch_pid->control();

    // “基础”期望的确定
    if (trot_flag == 0) // 普通站立模式
    {
        lleg->COMB_z_exp0 = master_L0_exp;
        rleg->COMB_z_exp0 = master_L0_exp;
        lleg->COMB_x_exp0 = master_l_xx_exp + delta_master_l_xx_exp;
        rleg->COMB_x_exp0 = master_r_xx_exp + delta_master_r_xx_exp;
        slave_l_L0_exp = slave_L0_exp;
        slave_r_L0_exp = slave_L0_exp;
        // slave_l_xx_exp
        // slave_r_xx_exp
    }
    else // 步态模式
    {
        t += TIME_STEP;
        if (t > Ts)
        {
            t -= Ts;
        }
        if (t <= Ts * lambda)
        {
            sigma = 2 * pi * t / (lambda * Ts);
            zep = h * (1 - cos(sigma)) / 2;
            xep_b = (xf - xs) * (sigma - sin(sigma)) / (2 * pi) + xs;
            xep_z = (xs - xf) * (sigma - sin(sigma)) / (2 * pi) + xf;
            // z
            lleg->COMB_z_exp0 = master_L0_exp - zep; // 左后
            rleg->COMB_z_exp0 = master_L0_exp;       // 右后
            slave_l_L0_exp = slave_L0_exp;           // 左前
            slave_r_L0_exp = slave_L0_exp - zep;     // 右前
            // x
            lleg->COMB_x_exp0 = xep_z;
            rleg->COMB_x_exp0 = xep_b;
            slave_l_xx_exp = xep_b;
            slave_r_xx_exp = xep_z;
        }
        if (t > Ts * lambda && t < Ts)
        {
            sigma = 2 * pi * (t - Ts * lambda) / (lambda * Ts);
            zep = h * (1 - cos(sigma)) / 2;
            xep_b = (xf - xs) * (sigma - sin(sigma)) / (2 * pi) + xs;
            xep_z = (xs - xf) * (sigma - sin(sigma)) / (2 * pi) + xf;
            // z
            lleg->COMB_z_exp0 = master_L0_exp;       // 左后
            rleg->COMB_z_exp0 = master_L0_exp - zep; // 右后
            slave_l_L0_exp = slave_L0_exp - zep;     // 左前
            slave_r_L0_exp = slave_L0_exp;           // 右前
            // x
            lleg->COMB_x_exp0 = xep_b;
            rleg->COMB_x_exp0 = xep_z;
            slave_l_xx_exp = xep_z;
            slave_r_xx_exp = xep_b;
        }
    }
    ROS_ERROR("【rleg->phi1 = %f,rleg->phi4 = %f】\n", rleg->phi1, rleg->phi4);
    // 反馈：实际z
    lleg->COMB_z = Forward_Kinematics_xyz(lleg, 2); //
    rleg->COMB_z = Forward_Kinematics_xyz(rleg, 2); //
    // 反馈：实际x
    lleg->COMB_x = Forward_Kinematics_xyz(lleg, 1); //
    rleg->COMB_x = Forward_Kinematics_xyz(rleg, 1); //
    // 总的腿长期望需要加上roll和pitch的补偿
    lleg->COMB_z_exp = lleg->COMB_z_exp0 + K_roll * F_roll + K_pitch * F_pitch;                          // 作为后腿（主机），俯仰角输出都是+
    rleg->COMB_z_exp = rleg->COMB_z_exp0 - K_roll * F_roll + K_pitch * F_pitch;                          // 作为后腿（主机），俯仰角输出都是+
    lleg->COMB_x_exp = lleg->COMB_x_exp0 + tan(pitch * pi / 180) * lleg->COMB_z + delta_master_l_xx_exp; // 确保重心竖直投影位于足端连线中点
    rleg->COMB_x_exp = rleg->COMB_x_exp0 + tan(pitch * pi / 180) * rleg->COMB_z + delta_master_r_xx_exp; // 确保重心竖直投影位于足端连线中点

    // 限制z的长度(上下限)
    lleg->COMB_z_exp = leg_length_limit(lleg);
    rleg->COMB_z_exp = leg_length_limit(rleg);
    printf("-------------------------------------------------------------------\n");
    ROS_WARN("[MASTER]左轮速度 = %f, 右轮速度 = %f\n", master_dx_l, master_dx_r);
    ROS_WARN("[SLAVE] 左轮速度 = %f, 右轮速度 = %f\n", slave_dx_l, slave_dx_r);
    ROS_WARN("左腿z期望 = %f, x期望 = %f\n", lleg->COMB_z_exp, lleg->COMB_x_exp);
    ROS_WARN("右腿z期望 = %f, x期望 = %f\n", rleg->COMB_z_exp, rleg->COMB_x_exp);
    // ROS_ERROR("----------------yaw_s = %f\n", yaw_exp_s);
    // ROS_ERROR("----------yaw_m = %f\n", yaw_exp_m);
    ROS_ERROR("----theta_turn = %f,theta_turn_exp = %f\n", theta_turn, theta_turn_exp);
    // ROS_ERROR("V * cos(theta_turn) = %f, sign(theta_turn) * v_vertical * sin(fai_b)=%f\n", V * cos(theta_turn), sign(theta_turn) * v_vertical * sin(fai_b));
    // ROS_ERROR("-fai_b = %f, fai_f = %f\n", fai_b, fai_f);
    ROS_WARN("左腿z反馈 = %f, x反馈 = %f\n", lleg->COMB_z, lleg->COMB_x);
    ROS_WARN("右腿z反馈 = %f, x反馈 = %f\n", rleg->COMB_z, rleg->COMB_x);
    ROS_WARN("roll期望 = %f, pitch期望 = %f\n", COMB_roll_exp, COMB_pitch_exp);
    lleg->COMB_z_pid->exp_set(lleg->COMB_z_exp);
    rleg->COMB_z_pid->exp_set(rleg->COMB_z_exp);
    lleg->COMB_z_pid->update(lleg->COMB_z);
    rleg->COMB_z_pid->update(rleg->COMB_z);
    lleg->COMB_x_pid->exp_set(lleg->COMB_x_exp);
    rleg->COMB_x_pid->exp_set(rleg->COMB_x_exp);
    lleg->COMB_x_pid->update(lleg->COMB_x);
    rleg->COMB_x_pid->update(rleg->COMB_x);
    // PID输出
    lleg->Fz = lleg->COMB_z_pid->control();
    rleg->Fz = rleg->COMB_z_pid->control();
    lleg->Fx = lleg->COMB_x_pid->control();
    rleg->Fx = rleg->COMB_x_pid->control();
    ROS_ERROR("左腿Fz = %f, Fx反馈 = %f\n", lleg->Fz, lleg->Fx);
    ROS_ERROR("右腿Fz = %f, Fx反馈 = %f\n", rleg->Fz, rleg->Fx);
}

void VMC::leg_length_control() // 【单体】 腿长控制 包含了横滚控制和跳跃控制
{
    roll_pid->update(roll);
    lleg->L0_pid->exp_set(lleg->L0_exp);
    rleg->L0_pid->exp_set(rleg->L0_exp);
    lleg->L0_pid->update(lleg->L0);
    rleg->L0_pid->update(rleg->L0);
    if (!lleg->isRobotOffGround && !rleg->isRobotOffGround)
    {
        lleg->F = lleg->L0_pid->control() + roll_pid->control();
        rleg->F = rleg->L0_pid->control() - roll_pid->control();
    }
    else
    {
        lleg->F = lleg->L0_pid->control();
        rleg->F = rleg->L0_pid->control();
    }
    // jump
    if (isRobotJumping)
    {
        counter_Jump++;
        if (counter_Jump >= jump_time / TIME_STEP)
        {
            isRobotJumping = false;
            rleg->L0_exp -= 0 * jump_length;
            lleg->L0_exp -= 0 * jump_length;
        }
    }

    //// 前馈 ////
    // roll_pid->update(roll);
    // printf("( roll = %f, output = %f )\n", roll, roll_pid->control());
    // if (lleg->L0_exp > 0.146719)
    // {
    //     lleg->L0_pid->exp_set(lleg->L0_exp);
    // }
    // if (rleg->L0_exp > 0.146719)
    // {
    //     rleg->L0_pid->exp_set(rleg->L0_exp);
    // }
    // lleg->L0_pid->update(lleg->L0);
    // rleg->L0_pid->update(rleg->L0);
    // // 前馈控制量计算 2024.01.30
    // lleg->nl = nl_calc(lleg);
    // lleg->F_gravity = 9.8 * (0.5 * m_body + lleg->nl * m_l);
    // lleg->F_inertia = (0.5 * m_body + lleg->nl * m_l) * lleg->dyaw * lleg->dx * lleg->L0 / (2 * 0.183); // dx其实应该改成机身速度，不会测，摆了
    // rleg->nl = nl_calc(rleg);
    // rleg->F_gravity = 9.8 * (0.5 * m_body + rleg->nl * m_l);
    // rleg->F_inertia = (0.5 * m_body + rleg->nl * m_l) * rleg->dyaw * rleg->dx * rleg->L0 / (2 * 0.183);
    // printf("[ L = %f, L_exp = %f, err = %f ]\n", lleg->L0, lleg->L0_exp, lleg->L0 - lleg->L0_exp);
    // if (!lleg->isRobotOffGround && !rleg->isRobotOffGround) // 在地上
    // {
    //     lleg->F = lleg->L0_pid->control() + roll_pid->control() + (lleg->F_gravity - lleg->F_inertia) * nl_index;
    //     rleg->F = rleg->L0_pid->control() - roll_pid->control() + (rleg->F_gravity + rleg->F_inertia) * nl_index;
    //     printf("[ lleg->F = %f, lleg->F_gravity = %f, lleg->L0_pid = %f ]\n", lleg->F, lleg->F_gravity, lleg->L0_pid->control());
    // }
    // else
    // {
    //     lleg->F = lleg->L0_pid->control() + lleg->F_gravity - lleg->F_inertia;
    //     rleg->F = rleg->L0_pid->control() + lleg->F_gravity - lleg->F_inertia;
    // }
    // if (isRobotJumping)
    // {
    //     counter_Jump++;
    //     if (counter_Jump >= jump_time / TIME_STEP)
    //     {
    //         isRobotJumping = false;
    //         rleg->L0_exp -= 0 * jump_length;
    //         lleg->L0_exp -= 0 * jump_length;
    //     }
    // }
}

void VMC::VMC_controller() // VMC控制器的所有解算
{
    leg_pos(lleg); // 求实际 L0 phi0(theta)
    leg_pos(rleg); // 求实际 L0 phi0(theta)
    leg_spd(lleg); // 求腿部运动速度 dL0 dphi0(dtheta)
    leg_spd(rleg); // 求腿部运动速度 dL0 dphi0(dtheta)

    // lleg->dtheta=lleg->theta_filter->update(lleg->dtheta);
    // rleg->dtheta=rleg->theta_filter->update(rleg->dtheta);

    LQR_K(lleg); // 求对应腿长的K矩阵
    LQR_K(rleg); // 求对应腿长的K矩阵

    FN_solve(lleg); // 求解支持力
    FN_solve(rleg); // 求解支持力

    // lqr反馈量计算
    T_Calc(lleg);
    Tp_Calc(lleg);
    T_Calc(rleg);
    Tp_Calc(rleg);

    // 腿长pid及横滚控制
    leg_length_control();

    // 两腿theta一致补偿
    theta_pid->update(lleg->theta - rleg->theta);
    lleg->Tp -= theta_pid->control();
    rleg->Tp += theta_pid->control();

    // 求前后髋关节电机输出力矩 T1前 T2后
    leg_conv(lleg);
    leg_conv(rleg);

    // 转向控制 输入为yaw & yaw_exp，向T_l & T_r 上叠加修正量
    turn_control();
    // 后面必须是dx_exp 因为如果速度大于速度期望，就发散了
    if (lleg->dx_exp != 0)
    {
        lleg->x_exp += lleg->dx_exp * TIME_STEP;
    }

    if (rleg->dx_exp != 0)
    {
        rleg->x_exp += rleg->dx_exp * TIME_STEP;
    }

    lleg->x_err = lleg->x_exp - lleg->x;
    rleg->x_err = rleg->x_exp - rleg->x;
    if (lleg->x_err > rleg->x_err) // 左err大于右err，左转时发生
    {
        rleg->x_exp += 0.001;
        lleg->x_exp -= 0.001;
    }
    else // 右err大于左err，右转时发生
    {
        rleg->x_exp -= 0.001;
        lleg->x_exp += 0.001;
    }
}

void VMC::Plan_exp_set()
{
    if (isPlan)
    {
        double angle_step = plan_w * 180 / pi / CONTROL_RATE;
        angle_step = LIMIT(angle_step, 0.2);
        double speed = LIMIT(plan_v, 0.2);
        lleg->dx_exp = speed;
        rleg->dx_exp = speed;
        // w为- 时右转
        yaw_exp += angle_step;
        lleg->x_exp -= angle_step * pi / 180 * half_wheel_d;
        rleg->x_exp += angle_step * pi / 180 * half_wheel_d;
    }
    counter_plan++;
    if (counter_plan > CONTROL_RATE / PLAN_RATE * 3 && counter_plan < CONTROL_RATE / PLAN_RATE * 10)
    {
        lleg->dx_exp = 0;
        rleg->dx_exp = 0;
        isPlan = false;
    }
}

void VMC::VMC_output() // 输出限幅 保护等
{
    message::vmc_interface vmc_msg;
    vmc_msg.T_l = 0;
    vmc_msg.T1_l = 0;
    vmc_msg.T2_l = 0;
    vmc_msg.T_r = 0;
    vmc_msg.T1_r = 0;
    vmc_msg.T2_r = 0;
    vmc_msg.flag = 0;
    if (hip_protect != 0 || wheel_protect != 0 || !isTestMode)
    {
        ROS_WARN("！！！！先断电再关控制！！！！");
    }
    // 限制最大扭矩，国际标准单位
    lleg->T = LIMIT(lleg->T, foot_limit);
    lleg->T1 = LIMIT(lleg->T1, hip_limit);
    lleg->T2 = LIMIT(lleg->T2, hip_limit);
    rleg->T = LIMIT(rleg->T, foot_limit);
    rleg->T1 = LIMIT(rleg->T1, hip_limit);
    rleg->T2 = LIMIT(rleg->T2, hip_limit);
    if (counter_P > init_time / TIME_STEP + 1)
    {
        // if (lleg->phi0 < 40 * pi / 180 || lleg->phi0 > 140 * pi / 180 || rleg->phi0 < 40 * pi / 180 || rleg->phi0 > 140 * pi / 180)
        // {
        //     ROS_ERROR("Wrong!,phi0 = %f度/%f度", lleg->phi0 * 180 / pi, rleg->phi0 * 180 / pi);
        //     isRobotSafe = false;
        // }
        // 后电机为phi1取值范围为[1.5，4]   前电机为phi4取值范围为[-1，1.5]
        if (lleg->phi1 < 1.5 || lleg->phi1 > 4.5 || lleg->phi4 < -1.5 || lleg->phi4 > 1.5)
        {
            ROS_ERROR("Wrong!,left_leg phi1 = %f度  phi4=%f度", lleg->phi1 * 180 / pi, lleg->phi4 * 180 / pi);
            lleg->err_counter++;
        }
        else
        {
            rleg->err_counter = 0;
        }

        // 后电机为phi1取值范围为[1.5，4]   前电机为phi4取值范围为[-1，1.5]
        if (rleg->phi1 < 1.5 || rleg->phi1 > 4.5 || rleg->phi4 < -1.5 || rleg->phi4 > 1.5)
        {
            ROS_ERROR("Wrong!,right_leg phi1 = %f度  phi4=%f度", rleg->phi1 * 180 / pi, rleg->phi4 * 180 / pi);
            rleg->err_counter++;
        }
        else
        {
            rleg->err_counter = 0;
        }
        if (rleg->err_counter > 10 || lleg->err_counter > 10)
        {
            isRobotSafe = false;
        }

        if (lleg->dx > 3 || lleg->dx < -3 || rleg->dx > 3 || rleg->dx < -3)
        {
            ROS_ERROR("轮胎速度太快了");
            isRobotSafe = false;
        }
        // 电机通讯判断
        if (last_counter_Motor == counter_Motor)
        {
            error_Motor++;
        }
        else
        {
            error_Motor = 0;
        }
        last_counter_Motor = counter_Motor;
        if (error_Motor > 24 && counter_Motor != 0)
        {
            ROS_ERROR("电机通讯断开");
            isRobotSafe = false;
        }
        // IMU通讯判断
        if (last_counter_IMU == counter_IMU)
        {
            error_IMU++;
        }
        else
        {
            error_IMU = 0;
        }
        last_counter_IMU = counter_IMU;
        if (error_IMU > 30 && counter_IMU != 0)
        {
            ROS_ERROR("IMU通讯断开");
            isRobotSafe = false;
        }
    }
    if (isRobotSafe && !isTestMode) // 安全且没有在测试模式
    {
        // vmc_msg.T_l = T2iqControl_MF9015(lleg->T * wheel_protect);
        vmc_msg.T_l = lleg->T * wheel_protect;
        vmc_msg.T1_l = T2iqControl_MG6012(lleg->T1 * hip_protect);
        vmc_msg.T2_l = T2iqControl_MG6012(lleg->T2 * hip_protect);

        // vmc_msg.T_r = T2iqControl_MF9015(rleg->T * wheel_protect);
        vmc_msg.T_r = rleg->T * wheel_protect;
        vmc_msg.T1_r = T2iqControl_MG6012(rleg->T1 * hip_protect);
        vmc_msg.T2_r = T2iqControl_MG6012(rleg->T2 * hip_protect);
    }
    if (isRobotSafe)
    {
        vmc_msg.flag = 1;
    }
    vmc_output_pub.publish(vmc_msg);
    if (!isRobotSafe)
    {
        ros::shutdown();
    }
}

void VMC::run() // 设定控制周期的主循环，传感器回调，控制器计算，pub控制信息
{
    double total_time = 0;
    std::chrono::duration<double, std::milli> rate(TIME_STEP * 1000);
    while (ros::ok())
    {
        high_resolution_clock::time_point h_start = high_resolution_clock::now();
        // if (mag_leftfront && mag_rightfront && mag_slave_left_back && mag_slave_right_back)
        // {
        //     robotstate = COMB;
        // }
        // cout<<robotstate<<endl;
        switch (robotstate)
        {
        case NORM: // VMC控制
            Plan_exp_set();
            VMC_controller(); // 控制器  //这个函数里面应该包括检测电磁铁通断的内容
            state_print();    // 打印状态
            counter_P++;
            if (counter_P > init_time / TIME_STEP)
            {
                VMC_output(); // 对输出进行限幅或衰减、判断机器人状态是否安全 发送安全的控制命令到接口 interface
            }
            else
            {
                cout << "!!!!!" << init_time - counter_P * TIME_STEP << "s后启动" << endl;
            }
            break;
        case INIT: // 初始化位置控制
            cout << "按 r/select 初始化" << endl;
            printf("state = %d\n", slave_flag);
            break;
        case COMB: // 拼接后控制
            // ROS_ERROR("拼接模式");
            if (COMB_mode == 0) // 位置开环+运动学
            {
                COMB_controller_Pos(2000, 1500, 100, 0); // int Kpitch, int Kroll
                COMB_output_Pos();
            }
            if (COMB_mode == 1)
            {
                COMB_controller_F();
                counter_P++;
                if (counter_P > init_time / TIME_STEP)
                {
                    COMB_output_F_single(); // 对输出进行限幅或衰减、判断机器人状态是否安全 发送安全的控制命令到接口 interface
                    COMB_output_F_multi();
                }
                else
                {
                    cout << "!!!!!" << init_time - counter_P * TIME_STEP << "s后启动" << endl;
                }
            }
            break;
        }
        // 控制结束时间
        high_resolution_clock::time_point h_end = high_resolution_clock::now();
        // 时间间隔计算
        std::chrono::duration<double, std::milli> control_time = duration_cast<duration<double, std::milli>>(h_end - h_start);
        if (rate > control_time)
        {
            std::this_thread::sleep_for(rate - control_time);
            // ROS_INFO("sleep_time:%lf", (rate - control_time).count());
        }
        // 周期结束时间
        high_resolution_clock::time_point h_T_end = high_resolution_clock::now();
        std::chrono::duration<double, std::milli> P_time = duration_cast<duration<double, std::milli>>(h_T_end - h_start);
        double Period_Time_ms = P_time.count();
        total_time += Period_Time_ms;
        // cout << "控制周期:" << Period_Time_ms << "ms" << endl
        //      << endl
        //      << endl;
        if (Period_Time_ms > TIME_STEP * 10 * 1000 || Period_Time_ms < TIME_STEP * 0.1 * 1000)
        {
            if (counter_P > init_time / TIME_STEP + 1)
            {
                ROS_ERROR("死机");
                isRobotSafe = true; // false
            }
        }
        ros::spinOnce();
    }
}

double VMC::Inverse_Kinematics(double L0_exp, double theta, int flag)
{
    double x, z; // foot's xyz
    double a, b, c;
    x = L0_exp * cos(theta) + L5 / 2;
    z = L0_exp * sin(theta);
    a = 2 * x * L1;
    b = 2 * z * L1;
    c = x * x + z * z + L1 * L1 - L2 * L2;
    double alpha11 = 2 * atan((b + sqrt(a * a + b * b - c * c)) / (a + c));
    double alpha12 = 2 * atan((b - sqrt(a * a + b * b - c * c)) / (a + c));

    double d = 2 * L4 * (x - L5);
    double e = 2 * L4 * z;
    double f = (x - L5) * (x - L5) + L4 * L4 + z * z - L3 * L3;
    double alpha21 = 2 * atan((e + sqrt(d * d + e * e - f * f)) / (d + f));
    double alpha22 = 2 * atan((e - sqrt(d * d + e * e - f * f)) / (d + f));
    if (alpha11 < 0)
    {
        alpha11 = alpha11 + 2 * pi;
    }
    if (alpha12 < 0)
    {
        alpha12 = alpha12 + 2 * pi;
    }
    if (alpha21 < 0)
    {
        alpha21 = alpha21 + 2 * pi;
    }
    // if (alpha22 < 0)
    // {
    //     alpha22 = alpha22 + 2 * pi;
    // }

    if (flag == 1)
    {
        return alpha11 * 180 / pi;
    }
    if (flag == 4)
    {
        return alpha22 * 180 / pi;
    }
}

double VMC::Inverse_Kinematics_xyz(double x, double z, int flag)
{
    double a, b, c;
    a = 2 * x * L1;
    b = 2 * z * L1;
    c = x * x + z * z + L1 * L1 - L2 * L2;
    double alpha11 = 2 * atan((b + sqrt(a * a + b * b - c * c)) / (a + c));
    double alpha12 = 2 * atan((b - sqrt(a * a + b * b - c * c)) / (a + c));

    double d = 2 * L4 * (x - L5);
    double e = 2 * L4 * z;
    double f = (x - L5) * (x - L5) + L4 * L4 + z * z - L3 * L3;
    double alpha21 = 2 * atan((e + sqrt(d * d + e * e - f * f)) / (d + f));
    double alpha22 = 2 * atan((e - sqrt(d * d + e * e - f * f)) / (d + f));
    if (alpha11 < 0)
    {
        alpha11 = alpha11 + 2 * pi;
    }
    if (alpha12 < 0)
    {
        alpha12 = alpha12 + 2 * pi;
    }
    if (alpha21 < 0)
    {
        alpha21 = alpha21 + 2 * pi;
    }
    // if (alpha22 < 0)
    // {
    //     alpha22 = alpha22 + 2 * pi;
    // }

    if (flag == 1)
    {
        return alpha11 * 180 / pi;
    }
    if (flag == 4)
    {
        return alpha22 * 180 / pi;
    }
}

double VMC::Forward_Kinematics_xyz(Leg_Model *leg, int flag) // flag: 1->x; 2->z
{
    double alpha1 = leg->phi1;
    double alpha4 = leg->phi4;
    double Xa = L1 * cos(leg->phi1);
    double Ya = L1 * sin(leg->phi1);
    double Xc = L5 + L4 * cos(leg->phi4);
    double Yc = L4 * sin(leg->phi4);
    double lengthAC = sqrt((Xc - Xa) * (Xc - Xa) + (Yc - Ya) * (Yc - Ya));
    double A = 2 * L2 * (Xc - Xa);
    double B = 2 * L2 * (Yc - Ya);
    double C = L2 * L2 + lengthAC * lengthAC - L3 * L3;
    double theta1 = 2 * atan((B + sqrt(A * A + B * B - C * C)) / (A + C));
    double x = Xa + L2 * cos(theta1);
    double y = Ya + L2 * sin(theta1);
    if (flag == 1)
    {
        return x;
    }
    if (flag == 2)
    {
        return y;
    }
}

double VMC::vel_limit(double low_lim, double high_lim, double vel)
{
    if (vel < low_lim)
    {
        return low_lim;
    }
    else if (vel > high_lim)
    {
        return high_lim;
    }
    else
    {
        return vel;
    }
}

// 力位控制
void VMC::COMB_controller_F() // 力位混合控制【拼接】
{
    //// 腿的控制 力位混合控制 ////
    // 求Fx和Fz(包含横滚、俯仰的控制)
    COMB_leg_length_control();
    // 将Fx和Fz通过雅各比矩阵算出髋关节电机扭矩
    leg_conv_comb(lleg);
    leg_conv_comb(rleg);
    //// 轮子的控制 速度闭环控制（电机自带） ////
    // fdb turn_angle
    double dis_s = abs(yaw_exp_s - yaw_s);
    if (dis_s <= 180) // exp与fdb未经过正负180度线
    {
        if (yaw_exp_s >= yaw_s)
        {
            yaw_exp_s = yaw_s + dis_s;
        }
        else
        {
            yaw_exp_s = yaw_s - dis_s;
        }
    }
    else // exp与fdb经过了正负180度线
    {
        dis_s = 360 - dis_s;
        if (yaw_exp_s >= yaw_s)
        {
            yaw_exp_s = yaw_s - dis_s;
        }
        else
        {
            yaw_exp_s = yaw_s + dis_s;
        }
    }
    double dis_m = abs(yaw_exp_m - yaw_m);
    if (dis_m <= 180) // exp与fdb未经过正负180度线
    {
        if (yaw_exp_m >= yaw_m)
        {
            yaw_exp_m = yaw_m + dis_m;
        }
        else
        {
            yaw_exp_m = yaw_m - dis_m;
        }
    }
    else // exp与fdb经过了正负180度线
    {
        dis_m = 360 - dis_m;
        if (yaw_exp_m >= yaw_m)
        {
            yaw_exp_m = yaw_m - dis_m;
        }
        else
        {
            yaw_exp_m = yaw_m + dis_m;
        }
    }

    theta_turn = ((yaw_s - yaw_m) - (yaw_exp_s - yaw_exp_m)) * pi / 180;
    if (c == 0)
    {
        theta_turn_exp = 0;
    }
    else
    {
        theta_turn_exp = (atan(turn_L2 / R_b) + atan(turn_L1 / R_f)) * c;
    }
    if (theta_turn_exp > 19 * pi / 180)
    {
        theta_turn_exp = 19 * pi / 180;
    }
    if (theta_turn_exp < -19 * pi / 180)
    {
        theta_turn_exp = -19 * pi / 180;
    }
    // theta_turn's definition is in callback.cpp

    // Dead area of COMB turn
    // if (theta_turn > -1 * pi / 180 && theta_turn < 1 * pi / 180)
    // {
    //     theta_turn = 0;
    // }

    if (c != 0) // w != 0
    {
        if (V > 0)
        {
            COMB_turn_pid_s->exp_set(theta_turn_exp);
            COMB_turn_pid_s->update(theta_turn);
            master_dx_l_run = (R_b - c * half_wheel_d) * w / radius;
            master_dx_r_run = (R_b + c * half_wheel_d) * w / radius;
            slave_dx_l_run = (R_f - c * half_wheel_d) * w / radius - COMB_turn_pid_s->control();
            slave_dx_r_run = (R_f + c * half_wheel_d) * w / radius + COMB_turn_pid_s->control();
        }
        else
        {
            COMB_turn_pid_m->exp_set(theta_turn_exp);
            COMB_turn_pid_m->update(theta_turn);
            master_dx_l_run = (R_b - c * half_wheel_d) * w / radius + COMB_turn_pid_m->control();
            master_dx_r_run = (R_b + c * half_wheel_d) * w / radius - COMB_turn_pid_m->control();
            slave_dx_l_run = (R_f - c * half_wheel_d) * w / radius;
            slave_dx_r_run = (R_f + c * half_wheel_d) * w / radius;
        }
    }
    else // w == 0
    {
        if (V > 0)
        {
            // v_vertical = V * sin(theta_turn) / cos(fai_b);
            // // master_dx_l_run = a * (V * cos(theta_turn) - v_vertical * sin(fai_b)) / radius + b * ((R_b - c * half_wheel_d) * w / radius);
            // // master_dx_r_run = a * (V * cos(theta_turn) + v_vertical * sin(fai_b)) / radius + b * ((R_b + c * half_wheel_d) * w / radius);
            // // slave_dx_l_run = (R_f - c * half_wheel_d) * w / radius;
            // // slave_dx_r_run = (R_f + c * half_wheel_d) * w / radius;
            // master_dx_l_run = (V * cos(theta_turn) - v_vertical * sin(fai_b)) / radius;
            // master_dx_r_run = (V * cos(theta_turn) + v_vertical * sin(fai_b)) / radius;
            // slave_dx_l_run = V / radius;
            // slave_dx_r_run = V / radius;
            COMB_recover_pid_m->exp_set(theta_turn_exp);
            COMB_recover_pid_m->update(theta_turn);
            master_dx_l_run = V / radius + COMB_recover_pid_m->control();
            master_dx_r_run = V / radius - COMB_recover_pid_m->control();
            slave_dx_l_run = V / radius;
            slave_dx_r_run = V / radius;
        }
        else
        {
            // v_vertical = V * sin(theta_turn) / cos(fai_f);
            // master_dx_l_run = V / radius;
            // master_dx_r_run = V / radius;
            // slave_dx_l_run = (V * cos(theta_turn) - v_vertical * sin(fai_f)) / radius;
            // slave_dx_r_run = (V * cos(theta_turn) + v_vertical * sin(fai_f)) / radius;
            COMB_recover_pid_s->exp_set(theta_turn_exp);
            COMB_recover_pid_s->update(theta_turn);
            master_dx_l_run = V / radius;
            master_dx_r_run = V / radius;
            slave_dx_l_run = V / radius - COMB_recover_pid_s->control();
            slave_dx_r_run = V / radius + COMB_recover_pid_s->control();
        }
        // else
        // {
        //     master_dx_l_run = 0;
        //     master_dx_r_run = 0;
        //     slave_dx_l_run = 0;
        //     slave_dx_r_run = 0;
        // }
    }

    // wheel speed cal end
    master_dx_l_run *= d;
    master_dx_r_run *= d;
    slave_dx_l_run *= d;
    slave_dx_r_run *= d;
    master_dx_l_run *= -1;
    master_dx_r_run *= -1;
    master_dx_l = vel_limit(-10, 10, master_dx_l_turn + master_dx_l_run);
    master_dx_r = vel_limit(-10, 10, master_dx_r_turn + master_dx_r_run);
    slave_dx_l = vel_limit(-10, 10, slave_dx_l_turn + slave_dx_l_run);
    slave_dx_r = vel_limit(-10, 10, slave_dx_r_turn + slave_dx_r_run);
}
void VMC::COMB_output_F_single() // 拼接模式下单机的节点间的控制输出
{
    message::cmb_interface cmb_msg; // 本机【主机】的controller给interface的消息
    // 国际标准单位
    cmb_msg.dx_l = 0;
    cmb_msg.T1_l = 0; // 前面电机
    cmb_msg.T2_l = 0; // 后面电机
    cmb_msg.dx_r = 0;
    cmb_msg.T1_r = 0;
    cmb_msg.T2_r = 0;
    cmb_msg.F_flag = 1;
    // 限制最大扭矩，国际标准单位
    // master_dx_l = master_dx_l_turn + master_dx_l_run;
    // master_dx_r = master_dx_r_turn + master_dx_r_run;
    lleg->T1 = LIMIT(lleg->T1, hip_limit);
    lleg->T2 = LIMIT(lleg->T2, hip_limit);
    rleg->T1 = LIMIT(rleg->T1, hip_limit);
    rleg->T2 = LIMIT(rleg->T2, hip_limit);
    if (counter_P > init_time / TIME_STEP + 1)
    {
        if (lleg->phi1 < pi / 2 || lleg->phi1 > 4.5 || lleg->phi4 < -1.1 || lleg->phi4 > pi / 2)
        {
            ROS_ERROR("Wrong!,left_leg phi1 = %f度  phi4=%f度", lleg->phi1 * 180 / pi, lleg->phi4 * 180 / pi);
            // lleg->err_counter++;
        }
        else
        {
            lleg->err_counter = 0;
        }

        // 后电机为phi1取值范围为[1.5，4]   前电机为phi4取值范围为[-1，1.5]
        if (rleg->phi1 < pi / 2 || rleg->phi1 > 4.5 || rleg->phi4 < -1.1 || rleg->phi4 > pi / 2)
        {
            ROS_ERROR("Wrong!,right_leg phi1 = %f度  phi4=%f度", rleg->phi1 * 180 / pi, rleg->phi4 * 180 / pi);
            // rleg->err_counter++;
        }
        else
        {
            rleg->err_counter = 0;
        }
        if (rleg->err_counter > 10 || lleg->err_counter > 10)
        {
            isRobotSafe = false;
        }
        // 电机通讯判断
        if (last_counter_Motor == counter_Motor)
        {
            error_Motor++;
        }
        else
        {
            error_Motor = 0;
        }
        last_counter_Motor = counter_Motor;
        if (error_Motor > 24 && counter_Motor != 0)
        {
            ROS_ERROR("电机通讯断开");
            isRobotSafe = false;
        }
        // IMU通讯判断
        if (last_counter_IMU == counter_IMU)
        {
            error_IMU++;
        }
        else
        {
            error_IMU = 0;
        }
        last_counter_IMU = counter_IMU;
        if (error_IMU > 30 && counter_IMU != 0)
        {
            ROS_ERROR("IMU通讯断开");
            isRobotSafe = false;
        }
    }
    if (isRobotSafe && !isTestMode) // 安全且没有在测试模式
    {
        // 左腿
        cmb_msg.dx_l = master_dx_l * wheel_protect;
        cmb_msg.T1_l = T2iqControl_MG6012(lleg->T1 * hip_protect);
        cmb_msg.T2_l = T2iqControl_MG6012(lleg->T2 * hip_protect);
        printf("T1_l = %f, T2_l = %f\n", lleg->T1, lleg->T2);
        // 右腿
        cmb_msg.dx_r = master_dx_r * wheel_protect;
        cmb_msg.T1_r = T2iqControl_MG6012(rleg->T1 * hip_protect);
        cmb_msg.T2_r = T2iqControl_MG6012(rleg->T2 * hip_protect);
        printf("T1_r = %f, T2_r = %f\n", rleg->T1, rleg->T2);
    }
    if (isRobotSafe)
    {
        cmb_msg.flag = 1;
    }
    cmb_output_pub.publish(cmb_msg); // 发布本机的两个控制节点
    if (!isRobotSafe)
    {
        ros::shutdown();
    }
}
void VMC::COMB_output_F_multi() // 拼接模式下多机之间的输出
{
    message::cmb_slave slave_cmb_msg; // 【主机】给【从机】的消息
    slave_cmb_msg.flag = 1;           // 启动标志位
    slave_cmb_msg.F_flag = 1;         // 力位控制标准位
    // state_print 想在终端里看什么消息在这打
    // 【从机】腿，给它期望，让从机根据期望自己去控制，而不是位置控制的时候全部由主机计算关节角度
    slave_cmb_msg.slave_l_L0_exp = slave_l_L0_exp;
    slave_cmb_msg.slave_r_L0_exp = slave_r_L0_exp;
    slave_cmb_msg.slave_l_xx_exp = slave_l_xx_exp + delta_slave_l_xx_exp;
    slave_cmb_msg.slave_r_xx_exp = slave_r_xx_exp + delta_slave_r_xx_exp;
    slave_cmb_msg.COMB_roll_exp = COMB_roll_exp;
    slave_cmb_msg.COMB_pitch_exp = COMB_pitch_exp;
    // 【从机】轮
    // slave_dx_l = slave_dx_l_turn + slave_dx_l_run;
    // slave_dx_r = slave_dx_r_turn + slave_dx_r_run;
    slave_cmb_msg.dx_l = slave_dx_l * wheel_protect;
    slave_cmb_msg.dx_r = slave_dx_r * wheel_protect;
    // cout << slave_cmb_msg.T_l << endl;
    // cout << slave_cmb_msg.T_r << endl;
    cmb_slave_output_pub.publish(slave_cmb_msg); // 发送给UDP_pkg
}

// 位置控制
void VMC::COMB_controller_Pos(int Kpitch, int Kroll, int Dpitch, int Droll) // 位置开环控制【拼接】
{
    double err_roll = COMB_roll_exp - 0;
    double err_pitch = COMB_pitch_exp - pitch;
    // roll变化时需要调节的腿长变化量
    delta_master_l_z_exp = half_wheel_d * tan(err_roll * pi / 180);
    delta_slave_l_z_exp = half_wheel_d * tan(err_roll * pi / 180);
    delta_master_r_z_exp = -half_wheel_d * tan(err_roll * pi / 180);
    delta_slave_r_z_exp = -half_wheel_d * tan(err_roll * pi / 180);
    // pitch变化时需要调节的腿长变化量
    delta_master_l_z_exp1 = -master_L0_exp * 2 * sin(COMB_pitch_exp * pi / 360) * sin(COMB_pitch_exp * pi / 360) + c_orignal * sin((COMB_pitch_exp - 0) * pi / 180) * cos((COMB_pitch_exp - 0) * pi / 180);
    delta_master_r_z_exp1 = -master_L0_exp * 2 * sin(COMB_pitch_exp * pi / 360) * sin(COMB_pitch_exp * pi / 360) + c_orignal * sin((COMB_pitch_exp - 0) * pi / 180) * cos((COMB_pitch_exp - 0) * pi / 180);
    delta_slave_l_z_exp1 = -slave_L0_exp * 2 * sin(COMB_pitch_exp * pi / 360) * sin(COMB_pitch_exp * pi / 360) - c_orignal * sin((COMB_pitch_exp - 0) * pi / 180) * cos((COMB_pitch_exp - 0) * pi / 180);
    delta_slave_r_z_exp1 = -slave_L0_exp * 2 * sin(COMB_pitch_exp * pi / 360) * sin(COMB_pitch_exp * pi / 360) - c_orignal * sin((COMB_pitch_exp - 0) * pi / 180) * cos((COMB_pitch_exp - 0) * pi / 180);
    delta_master_l_x_exp = master_L0_exp * sin(COMB_pitch_exp * pi / 180) + c_orignal * sin(COMB_pitch_exp * pi / 180) * sin(COMB_pitch_exp * pi / 180);
    delta_master_r_x_exp = master_L0_exp * sin(COMB_pitch_exp * pi / 180) + c_orignal * sin(COMB_pitch_exp * pi / 180) * sin(COMB_pitch_exp * pi / 180);
    delta_slave_l_x_exp = slave_L0_exp * sin(COMB_pitch_exp * pi / 180) - c_orignal * sin(COMB_pitch_exp * pi / 180) * sin(COMB_pitch_exp * pi / 180);
    delta_slave_r_x_exp = slave_L0_exp * sin(COMB_pitch_exp * pi / 180) - c_orignal * sin(COMB_pitch_exp * pi / 180) * sin(COMB_pitch_exp * pi / 180);
    // “基础”期望的确定
    if (trot_flag == 0) // 普通站立模式
    {
        master_l_z_exp = master_L0_exp + delta_master_l_z_exp + delta_master_l_z_exp1;
        master_r_z_exp = master_L0_exp + delta_master_r_z_exp + delta_master_l_z_exp1;
        slave_l_z_exp = slave_L0_exp + delta_slave_l_z_exp + delta_slave_l_z_exp1;
        slave_r_z_exp = slave_L0_exp + delta_slave_r_z_exp + delta_slave_l_z_exp1;
        master_l_x_exp = master_l_xx_exp + delta_master_l_x_exp + delta_master_l_xx_exp;
        master_r_x_exp = master_r_xx_exp + delta_master_r_x_exp + delta_master_r_xx_exp;
        slave_l_x_exp = slave_l_xx_exp + delta_slave_l_x_exp + delta_slave_l_xx_exp;
        slave_r_x_exp = slave_r_xx_exp + delta_slave_r_x_exp + delta_slave_r_xx_exp;
    }
    else // 步态模式
    {
        t += TIME_STEP;
        if (t > Ts)
        {
            t -= Ts;
        }
        if (t <= Ts * lambda)
        {
            sigma = 2 * pi * t / (lambda * Ts);
            zep = h * (1 - cos(sigma)) / 2;
            xep_b = (xf - xs) * (sigma - sin(sigma)) / (2 * pi) + xs;
            xep_z = (xs - xf) * (sigma - sin(sigma)) / (2 * pi) + xf;
            master_l_z_exp = master_L0_exp + delta_master_l_z_exp + delta_master_l_z_exp1 - zep;
            master_r_z_exp = master_L0_exp + delta_master_r_z_exp + delta_master_l_z_exp1;
            slave_l_z_exp = slave_L0_exp + delta_slave_l_z_exp + delta_slave_l_z_exp1;
            slave_r_z_exp = slave_L0_exp + delta_slave_r_z_exp + delta_slave_l_z_exp1 - zep;
            master_l_x_exp = xep_z + delta_master_l_x_exp + delta_master_l_xx_exp;
            master_r_x_exp = xep_b + delta_master_r_x_exp + delta_master_r_xx_exp;
            slave_l_x_exp = xep_b + delta_slave_l_x_exp + delta_slave_l_xx_exp;
            slave_r_x_exp = xep_z + delta_slave_r_x_exp + delta_slave_r_xx_exp;
        }
        if (t > Ts * lambda && t < Ts)
        {
            sigma = 2 * pi * (t - Ts * lambda) / (lambda * Ts);
            zep = h * (1 - cos(sigma)) / 2;
            xep_b = (xf - xs) * (sigma - sin(sigma)) / (2 * pi) + xs;
            xep_z = (xs - xf) * (sigma - sin(sigma)) / (2 * pi) + xf;
            master_l_z_exp = master_L0_exp + delta_master_l_z_exp + delta_master_l_z_exp1;
            master_r_z_exp = master_L0_exp + delta_master_r_z_exp + delta_master_l_z_exp1 - zep;
            slave_l_z_exp = slave_L0_exp + delta_slave_l_z_exp + delta_slave_l_z_exp1 - zep;
            slave_r_z_exp = slave_L0_exp + delta_slave_r_z_exp + delta_slave_l_z_exp1;
            master_l_x_exp = xep_b + delta_master_l_x_exp + delta_master_l_xx_exp;
            master_r_x_exp = xep_z + delta_master_r_x_exp + delta_master_r_xx_exp;
            slave_l_x_exp = xep_z + delta_slave_l_x_exp + delta_slave_l_xx_exp;
            slave_r_x_exp = xep_b + delta_slave_r_x_exp + delta_slave_r_xx_exp;
        }
    }

    // {
    //     delta_master_l_z_exp = 0.09 - master_l_z_exp;
    //     delta_master_r_z_exp = 0.09 - master_r_z_exp;
    //     delta_slave_l_z_exp += -1 * c_orignal * sin((COMB_pitch_exp - pitch) * pi / 180) + delta_master_l_z_exp;
    //     delta_slave_r_z_exp += -1 * c_orignal * sin((COMB_pitch_exp - pitch) * pi / 180) + delta_master_r_z_exp;
    // }
    // if (delta_slave_l_z_exp < 0.09 - slave_l_z_exp)
    // {
    //     delta_slave_l_z_exp = 0.09 - slave_l_z_exp;
    //     delta_slave_r_z_exp = 0.09 - slave_r_z_exp;
    //     delta_master_l_z_exp += 1 * c_orignal * sin((COMB_pitch_exp - pitch) * pi / 180) + delta_slave_l_z_exp;
    //     delta_master_r_z_exp += 1 * c_orignal * sin((COMB_pitch_exp - pitch) * pi / 180) + delta_slave_r_z_exp;
    // }
    // }
    // limit
    // 步态
    // if (walk_flag)
    // {
    //     if ((count_walk / 60) % 2 == 0)
    //     {
    //         walk_m_l = 0;
    //         walk_m_r = -0.05;
    //         walk_s_l = -0.05;
    //         walk_s_r = 0;
    //     }
    //     if ((count_walk / 60) % 2 == 1)
    //     {
    //         walk_m_l = -0.05;
    //         walk_m_r = 0;
    //         walk_s_l = 0;
    //         walk_s_r = -0.05;
    //     }
    //     count_walk++;
    // }
    // else
    // {
    //     walk_m_l = 0;
    //     walk_m_r = 0;
    //     walk_s_l = 0;
    //     walk_s_r = 0;
    //     count_walk = 0;
    // }
    if (master_l_z_exp < 0.09)
    {
        delta_master_l_z_exp = 0.09 - master_L0_exp;
        delta_slave_l_z_exp = 0.09 - slave_L0_exp;
        delta_master_r_z_exp = -2 * half_wheel_d * tan((COMB_roll_exp - 0) * pi / 180) + delta_master_l_z_exp;
        delta_slave_r_z_exp = -2 * half_wheel_d * tan((COMB_roll_exp - 0) * pi / 180) + delta_master_l_z_exp;
    }
    if (master_r_z_exp < 0.09)
    {
        delta_master_r_z_exp = 0.09 - master_L0_exp;
        delta_slave_r_z_exp = 0.09 - slave_L0_exp;
        delta_master_l_z_exp = 2 * half_wheel_d * tan((COMB_roll_exp - 0) * pi / 180) + delta_master_r_z_exp;
        delta_slave_l_z_exp = 2 * half_wheel_d * tan((COMB_roll_exp - 0) * pi / 180) + delta_master_r_z_exp;
    }
    // 输出量
    master_l_phi1 = Inverse_Kinematics_xyz(master_l_x_exp, master_l_z_exp, 1);
    master_l_phi4 = Inverse_Kinematics_xyz(master_l_x_exp, master_l_z_exp, 4);
    master_r_phi1 = Inverse_Kinematics_xyz(master_r_x_exp, master_r_z_exp, 1);
    master_r_phi4 = Inverse_Kinematics_xyz(master_r_x_exp, master_r_z_exp, 4);

    slave_l_phi1 = Inverse_Kinematics_xyz(slave_l_x_exp, slave_l_z_exp, 1);
    slave_l_phi4 = Inverse_Kinematics_xyz(slave_l_x_exp, slave_l_z_exp, 4);
    slave_r_phi1 = Inverse_Kinematics_xyz(slave_r_x_exp, slave_r_z_exp, 1);
    slave_r_phi4 = Inverse_Kinematics_xyz(slave_r_x_exp, slave_r_z_exp, 4);

    // fuseIMUData(); // 计算pitch_fuse和roll_fuse
    // if (abs(pitch_fuse) < 2.5 && abs(roll_fuse) < 2.5)
    // {
    //     return;
    // }

    // // 根据我现在的角度算出一个对应的高度来
    // if (pitch_fuse < 0 && roll_fuse < 0)
    // {
    //     slave_l_phi1 = (0 - pitch_fuse) * Kpitch + (0 - imu_m.angle_vel_pitch) * Dpitch;         // pitch降低高度
    //     slave_l_phi4 = -(0 - pitch_fuse) * Kpitch + (-1) * (0 - imu_m.angle_vel_pitch) * Dpitch; // pitch降低高度

    //     slave_r_phi1 = -(0 - pitch_fuse) * Kpitch + (-1) * (0 - imu_m.angle_vel_pitch) * Dpitch + (-1) * (0 - roll_fuse) * Kroll; // pitch降低高度的同时，roll降低高度
    //     slave_r_phi4 = (0 - pitch_fuse) * Kpitch + (0 - imu_m.angle_vel_pitch) * Dpitch + (0 - roll_fuse) * Kroll;                // pitch降低高度的同时，roll降低高度

    //     rleg->phi1 += (-1) * (0 - roll_fuse) * Kroll;
    //     rleg->phi4 += (0 - roll_fuse) * Kroll;
    // }

    // else if (pitch_fuse < 0 && roll_fuse > 0)
    // {
    //     slave_l_phi1 = (0 - pitch_fuse) * Kpitch + (0 - imu_m.angle_vel_pitch) * Dpitch + (-1) * (0 - roll_fuse) * Kroll;  // pitch降低高度的同时，roll降低高度
    //     slave_l_phi4 = -(0 - pitch_fuse) * Kpitch + (-1) * (0 - imu_m.angle_vel_pitch) * Dpitch + (0 - roll_fuse) * Kroll; // pitch降低高度的同时，roll降低高度

    //     slave_r_phi1 = -(0 - pitch_fuse) * Kpitch + (-1) * (0 - imu_m.angle_vel_pitch) * Dpitch;
    //     slave_r_phi4 = (0 - pitch_fuse) * Kpitch + (0 - imu_m.angle_vel_pitch) * Dpitch;

    //     lleg->phi1 += (-1) * (0 - roll_fuse) * Kroll;
    //     lleg->phi4 += (0 - roll_fuse) * Kroll;
    // }

    // else if (pitch_fuse > 0 && roll_fuse < 0)
    // {
    //     lleg->phi1 += -(0 - pitch_fuse) * Kpitch + (-1) * (0 - imu_m.angle_vel_pitch) * Dpitch; // 顺时针降低高度
    //     lleg->phi4 += (0 - pitch_fuse) * Kpitch + (0 - imu_m.angle_vel_pitch) * Dpitch;         // 逆时针降低高度

    //     rleg->phi1 += (0 - pitch_fuse) * Kpitch + (0 - imu_m.angle_vel_pitch) * Dpitch + (-1) * (0 - roll_fuse) * Kroll; // 后腿pitch降低高度，同时roll也影响到需要降低高度
    //     rleg->phi4 += -(0 - pitch_fuse) * Kpitch + (-1) * (0 - imu_m.angle_vel_pitch) * Dpitch + (0 - roll_fuse) * Kroll;

    //     slave_r_phi1 = (-1) * (0 - roll_fuse) * Kroll;
    //     slave_r_phi4 = (0 - roll_fuse) * Kroll;
    // }

    // else if (pitch_fuse > 0 && roll_fuse > 0)
    // {
    //     lleg->phi1 += -(0 - pitch_fuse) * Kpitch + (-1) * (0 - imu_m.angle_vel_pitch) * Dpitch + (-1) * (0 - roll_fuse) * Kroll; // 顺时针降低高度
    //     lleg->phi4 += (0 - pitch_fuse) * Kpitch + (0 - imu_m.angle_vel_pitch) * Dpitch + (0 - roll_fuse) * Kroll;                // 逆时针降低高度

    //     rleg->phi1 += (0 - pitch_fuse) * Kpitch + (0 - imu_m.angle_vel_pitch) * Dpitch; // 后腿pitch降低高度，同时roll也影响到需要降低高度
    //     rleg->phi4 += -(0 - pitch_fuse) * Kpitch + (-1) * (0 - imu_m.angle_vel_pitch) * Dpitch;

    //     slave_l_phi1 = (-1) * (0 - roll_fuse) * Kroll;
    //     slave_l_phi4 = (0 - roll_fuse) * Kroll;
    // }

    // 先输出这个master主机的调整量

    // ROS_WARN("左前:%f.3",lleg->phi1);
    // ROS_WARN("左后:%f.3",lleg->phi4);
    // ROS_WARN("右前:%f.3",rleg->phi1);
    // ROS_WARN("右后:%f.3",rleg->phi4);

    // 再输出这个slave从机的调整量

    // ROS_WARN("从机左前:%.3f",slave_l_phi1);
    // ROS_WARN("从机左后:%.3f",slave_l_phi4);
    // ROS_WARN("从机右前:%.3f",slave_r_phi1);
    // ROS_WARN("从机右后:%.3f",slave_r_phi4);
}
void VMC::COMB_output_Pos() // 位置开环控制消息发布=本机发给interface节点的+发送给从机的角度信息
{
    message::cmb_interface cmb_msg;
    message::cmb_slave slave_cmb_msg;
    // cmb_msg.T_l = 0;
    // cmb_msg.P1_l = 0;
    // cmb_msg.P2_l = 0;
    // cmb_msg.T_r = 0;
    // cmb_msg.P1_r = 0;
    // cmb_msg.P2_r = 0;
    // cmb_msg.flag = 0;

    // slave_cmb_msg.T_l = 0;
    // slave_cmb_msg.P1_l = 0;
    // slave_cmb_msg.P2_l = 0;
    // slave_cmb_msg.T_r = 0;
    // slave_cmb_msg.P1_r = 0;
    // slave_cmb_msg.P2_r = 0;
    slave_cmb_msg.flag = 1;   // 启动标志位
    slave_cmb_msg.F_flag = 0; // 力位控制标准位
    printf("------------------------------------------\n");
    printf("mlexp=%f,mrexp=%f,slexp=%f,srexp=%f\n", master_l_z_exp, master_r_z_exp, slave_l_z_exp, slave_r_z_exp);
    // printf("master_L0_exp=%f, delta_master_l_z_exp=%f,delta_master_r_z_exp=%f,delta_slave_l_z_exp=%f,delta_slave_r_z_exp=%f\n", master_L0_exp, delta_master_l_z_exp, delta_master_r_z_exp, delta_slave_l_z_exp, delta_slave_r_z_exp);
    printf("master_dx_l=%f,master_dx_r=%f,slave_dx_l=%f,slave_dx_r=%f\n", master_dx_l, master_dx_r, slave_dx_l, slave_dx_r);
    printf("l1=%f,l4=%f,r1=%f,r4=%f\n\n", master_l_phi1, master_l_phi4, master_r_phi1, master_r_phi4);
    printf("pitch=%f, roll=%f\n", pitch, roll);
    printf("speed_l=%f, speed_r=%f\n", lleg->speed_now, rleg->speed_now);
    ROS_ERROR("delta_master_l_z_exp=%f, delta_master_r_z_exp=%f, delta_slave_l_z_exp=%f, delta_slave_r_z_exp=%f\n", delta_master_l_z_exp, delta_master_r_z_exp, delta_slave_l_z_exp, delta_slave_r_z_exp);
    printf("R=%f;  delta_x=%f\n", R, delta_x);
    printf("COMB_roll_exp=%f\n", COMB_roll_exp);
    ROS_ERROR("count_walk=%d\n", count_walk);
    printf("R=%f;  delta_x=%f\n", R, delta_x);
    printf("------------------------------------------\n");

    double dis_s = abs(yaw_exp_s - yaw_s);
    if (dis_s <= 180) // exp与fdb未经过正负180度线
    {
        if (yaw_exp_s >= yaw_s)
        {
            yaw_exp_s = yaw_s + dis_s;
        }
        else
        {
            yaw_exp_s = yaw_s - dis_s;
        }
    }
    else // exp与fdb经过了正负180度线
    {
        dis_s = 360 - dis_s;
        if (yaw_exp_s >= yaw_s)
        {
            yaw_exp_s = yaw_s - dis_s;
        }
        else
        {
            yaw_exp_s = yaw_s + dis_s;
        }
    }
    double dis_m = abs(yaw_exp_m - yaw_m);
    if (dis_m <= 180) // exp与fdb未经过正负180度线
    {
        if (yaw_exp_m >= yaw_m)
        {
            yaw_exp_m = yaw_m + dis_m;
        }
        else
        {
            yaw_exp_m = yaw_m - dis_m;
        }
    }
    else // exp与fdb经过了正负180度线
    {
        dis_m = 360 - dis_m;
        if (yaw_exp_m >= yaw_m)
        {
            yaw_exp_m = yaw_m - dis_m;
        }
        else
        {
            yaw_exp_m = yaw_m + dis_m;
        }
    }
    theta_turn = ((yaw_s - yaw_m) - (yaw_exp_s - yaw_exp_m)) * pi / 180;
    // Dead area
    // if (theta_turn > -1 * pi / 180 && theta_turn < 1 * pi / 180)
    // {
    //     theta_turn = 0;
    // }

    // master_dx_l_run = (R_b - sign(Joy->axes[0]) * half_wheel_d) * w / radius;
    // master_dx_r_run = (R_b + sign(Joy->axes[0]) * half_wheel_d) * w / radius;
    // slave_dx_l_run = (R_f - sign(Joy->axes[0]) * half_wheel_d) * w / radius;
    // slave_dx_r_run = (R_f + sign(Joy->axes[0]) * half_wheel_d) * w / radius;
    if (V > 0)
    {
        v_vertical = V * sin(theta_turn) / cos(fai_b);
        master_dx_l_run = a * (V * cos(theta_turn) - v_vertical * sin(fai_b)) / radius + b * ((R_b - c * half_wheel_d) * w / radius);
        master_dx_r_run = a * (V * cos(theta_turn) + v_vertical * sin(fai_b)) / radius + b * ((R_b + c * half_wheel_d) * w / radius);
        slave_dx_l_run = (R_f - c * half_wheel_d) * w / radius;
        slave_dx_r_run = (R_f + c * half_wheel_d) * w / radius;
    }
    else if (V < 0)
    {
        v_vertical = V * sin(theta_turn) / cos(fai_f);
        master_dx_l_run = (R_b - c * half_wheel_d) * w / radius;
        master_dx_r_run = (R_b + c * half_wheel_d) * w / radius;
        slave_dx_l_run = a * (V * cos(theta_turn) - v_vertical * sin(fai_f)) / radius + b * ((R_f - c * half_wheel_d) * w / radius);
        slave_dx_r_run = a * (V * cos(theta_turn) + v_vertical * sin(fai_f)) / radius + b * ((R_f + c * half_wheel_d) * w / radius);
    }
    else
    {
        master_dx_l_run = 0.0;
        master_dx_r_run = 0.0;
        slave_dx_l_run = 0.0;
        slave_dx_r_run = 0.0;
    }
    master_dx_l_run *= d;
    master_dx_r_run *= d;
    slave_dx_l_run *= d;
    slave_dx_r_run *= d;
    master_dx_l_run *= -1;
    master_dx_r_run *= -1;
    master_dx_l = vel_limit(-10, 10, master_dx_l_turn + master_dx_l_run);
    master_dx_r = vel_limit(-10, 10, master_dx_r_turn + master_dx_r_run);
    slave_dx_l = vel_limit(-10, 10, slave_dx_l_turn + slave_dx_l_run);
    slave_dx_r = vel_limit(-10, 10, slave_dx_r_turn + slave_dx_r_run);
    cmb_msg.P1_l = static_cast<int16_t>(master_l_phi1); // 髋关节输出位置信息
    cmb_msg.P2_l = static_cast<int16_t>(master_l_phi4);
    cmb_msg.P1_r = static_cast<int16_t>(master_r_phi1);
    cmb_msg.P2_r = static_cast<int16_t>(master_r_phi4);
    if (master_dx_l > 10)
    {
        master_dx_l = 10;
    }
    if (master_dx_l < -10)
    {
        master_dx_l = -10;
    }
    if (master_dx_r > 10)
    {
        master_dx_r = 10;
    }
    if (master_dx_r < -10)
    {
        master_dx_r = -10;
    }
    cmb_msg.T_l = master_dx_l * wheel_protect; // 轮毂依旧输出扭矩信息控制速度
    cmb_msg.T_r = master_dx_r * wheel_protect;
    cmb_output_pub.publish(cmb_msg); // 发布本机的两个控制节点

    slave_cmb_msg.P1_l = static_cast<int16_t>(slave_l_phi1);
    slave_cmb_msg.P2_l = static_cast<int16_t>(slave_l_phi4);
    slave_cmb_msg.P1_r = static_cast<int16_t>(slave_r_phi1);
    slave_cmb_msg.P2_r = static_cast<int16_t>(slave_r_phi4);
    if (slave_dx_l > 10)
    {
        slave_dx_l = 10;
    }
    if (slave_dx_l < -10)
    {
        slave_dx_l = -10;
    }
    if (slave_dx_r > 10)
    {
        slave_dx_r = 10;
    }
    if (slave_dx_r < -10)
    {
        slave_dx_r = -10;
    }
    slave_cmb_msg.T_l = slave_dx_l * wheel_protect;
    slave_cmb_msg.T_r = slave_dx_r * wheel_protect;
    // cout << slave_cmb_msg.T_l << endl;
    // cout << slave_cmb_msg.T_r << endl;
    cmb_slave_output_pub.publish(slave_cmb_msg); // 发送给UDP_pkg
}

// 计算角速度变化率（速度变化的绝对值和）
double VMC::calculateAngularVelocityChange(const message::imu_controller &imu_m, const message::imu_controller &imu_s)
{
    return std::abs(imu_m.angle_vel_pitch - imu_s.angle_vel_pitch) +
           std::abs(imu_m.angle_vel_roll - imu_s.angle_vel_roll) + std::abs(imu_m.angle_vel_yaw - imu_s.angle_vel_yaw);
}

void VMC::fuseIMUData()
{
    double delta1 = calculateAngularVelocityChange(imu_m, imu_m_pre);
    double delta2 = calculateAngularVelocityChange(imu_s, imu_s_pre);

    // 防止除以零的情况
    if (delta1 == 0)
        delta1 = 1e-6;
    if (delta2 == 0)
        delta2 = 1e-6;

    // 计算权重（变化率的倒数）
    double weight1 = 1.0 / delta1;
    double weight2 = 1.0 / delta2;

    // 归一化权重
    double totalWeight = weight1 + weight2;
    weight1 /= totalWeight;
    weight2 /= totalWeight;

    // 根据权重融合数据

    pitch_fuse = weight1 * imu_m.angle_pitch + weight2 * imu_s.angle_pitch;
    roll_fuse = weight1 * imu_m.angle_roll + weight2 * imu_s.angle_roll;

    // ROS_WARN("pitch:%.3f  roll:%.3f",pitch_fuse,roll_fuse);
}
