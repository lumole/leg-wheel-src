#include "controller.h"

VMC::VMC() // 初始化发布者、订阅者、pid等
{
    robotstate = INIT;
    // 定义话题订阅者
    motor_input_sub = n.subscribe("/motor_feedback", 1, &VMC::Motor_Input, this); // 接收电机反馈 只接受最新的
    imu_sub = n.subscribe("/imu_topic", 1, &VMC::IMU_Input, this);                // 接受IMU信息
    key_sub = n.subscribe("/key_command", 10, &VMC::Keyboard_Input, this);        // 接受键盘输入
    joy_sub = n.subscribe("/joy", 10, &VMC::Joy, this);                           // 接受手柄输入
    // plan_sub = n.subscribe("cmd_vel", 10, &VMC::Plan, this);                     // 接受规划输入
    plan_sub = n.subscribe("/cmd_vel", 10, &VMC::Plan, this); // 接受规划输入

    // 此处需要订阅232串口的信息
    serial_sub = n.subscribe("/mag_state", 10, &VMC::Serial_feedback, this);
    // slave_serial_sub=n.subscribe("/mag_cmd", 10, &VMC::Slave_Serial_feedback, this);    //订阅电磁铁控制的命令在232pkg里
    motor_adjust_sub = n.subscribe("/motor_adj", 10, &VMC::Motor_adjust_Input, this);

    // 定义话题发布者
    vmc_output_pub = n.advertise<message::vmc_interface>("/vmc_output", 10); // 发送VMC控制器输出至接口
    lqr_state_pub = n.advertise<message::lqr_state>("/lqr_state", 10);       // 发送VMC控制器输出至接口
    pid_state_pub = n.advertise<message::pid_state>("/pid_state", 10);       // 发送VMC控制器输出至接口


    cmb_output_pub = n.advertise<message::cmb_interface>("/cmb_slave", 10); // 发送后面主机需要调整的信息,给本机节点
    // cmb_slave_output_pub = n.advertise<message::cmb_interface>("/robot2/motor_adj", 10);   //发送前面从机需要调整的信息，给udp_pkg
    test_rleg_pub = n.advertise<message::test_msg>("/test_rleg", 10);
    test_lleg_pub = n.advertise<message::test_msg>("/test_lleg", 10);

    test_robotstate_pub = n.advertise<std_msgs::String>("/robotstate", 10);
    // 腿部结构体初始化
    lleg = (struct Leg_Model *)malloc(sizeof(struct Leg_Model));
    rleg = (struct Leg_Model *)malloc(sizeof(struct Leg_Model));
    leg_init(rleg);
    leg_init(lleg);
    lleg->dL0 = new TimeVar(TIME_STEP);
    rleg->dL0 = new TimeVar(TIME_STEP);
    lleg->dTheta = new TimeVar(TIME_STEP);
    rleg->dTheta = new TimeVar(TIME_STEP);
    // PID初始化
    theta_pid = new PID(1 / CONTROL_RATE, theta_pid_index, 0, 10, yaw);
    turn_pid = new PID(1 / CONTROL_RATE, turn_pid_index, 0, 3, 0);
    roll_pid = new PID(1 / CONTROL_RATE, roll_pid_index, 0, 100, roll_exp);
    yawfilter = new filter();
    plan_v = 0;
    plan_w = 0;
    roll = 0;
    roll_exp = 0;
    pitch = 0;
    yaw = 0;
    yaw_exp = 0;
    droll = 0;
    dpitch = 0;
    dyaw = 0;
    counter_plan = 0;
    test = turn_pid;
    isPlan = false;
    isRobotSafe = true;
    isRobotJumping = false;
    isYawInit = false;
    counter_P = 0;
    isTestMode = false;
    n.getParam("vmc_node/isTestMode", isTestMode);

    mag_leftback = 0;
    mag_leftfront = 0;
    mag_rightback = 0;
    mag_rightfront = 0;

    mag_slave_left_back = 0;
    mag_slave_right_back = 0;

    slave_l_phi1 = 0;
    slave_l_phi4 = 0;
    slave_r_phi1 = 0;
    slave_r_phi4 = 0;
    slave_T1 = 0;
    slave_T2 = 0;
}

void leg_init(Leg_Model *leg) // 初始化腿部结构体
{
    // 所有量默认为0
    memset(leg, 0, sizeof(struct Leg_Model));
    // 类或结构体初始化
    if(leg->L0_pid)
        free(leg->L0_pid);
    leg->L0_pid = new PID(1 / CONTROL_RATE, l0_pid_index, 40, 50, leg->L0_exp);
    leg->dTheta = new TimeVar(TIME_STEP);
    leg->dL0 = new TimeVar(TIME_STEP);
    // 不为0的量在以下初始化
    leg->isRobotOffGround = false;
    leg->phi0 = pi / 2;
    leg->phi1 = pi;
    leg->L0_exp = 0.15;
    leg->x_exp = 0;
    leg->L0 = 0.142;
    leg->F = m_body / 2 * g;
    leg->theta_exp=0.0198435;
    leg->phi_exp=0.00342348;

    // leg->theta_filter=new filter();
}

void VMC::state_print()
{
    // static uint16_t freq_division_counter = 0;
    // if (++freq_division_counter % 125 != 0)
    //     return;
    // pid状态输出
    pid_state_pub.publish(test->print());
    // printf("[LQR输入右]:\ntheta = %.2grad, x = %.2gm, phi = %.2grad,\ndtheta = %.2grad, dx = %.2gm/s, dphi = %.2grad/s\n", rleg->theta, rleg->x, rleg->phi, rleg->dtheta, rleg->dx, rleg->dphi);
    // printf("[LQR输入左]:\ntheta = %.2grad, x = %.2gm, phi = %.2grad,\ndtheta = %.2grad, dx = %.2gm/s, dphi = %.2grad/s\n", lleg->theta, lleg->x, lleg->phi, lleg->dtheta, lleg->dx, lleg->dphi);
    // printf("[LQR输出右]:\nT = %.2gN*m, T1 = %.2gN*m, T2 = %.2gN*m\n", rleg->T, rleg->T1, rleg->T2);
    // printf("[LQR输出左]:\nT = %.2gN*m, T1 = %.2gN*m, T2 = %.2gN*m\n", lleg->T, lleg->T1, lleg->T2);
    printf("[差速转向]:\n左 = %.2gm/s, 右 = %.2gm/s\n", lleg->dx_exp, rleg->dx_exp);
    // printf("*F: 左 = %f, 右 = %f\n", lleg->F, rleg->F);
    printf("*l_exp = %f, l = %f\n", lleg->L0_exp, lleg->L0);
    printf("*yaw_exp = %f, dyaw_exp = %f\n", yaw_exp, dyaw_exp);
    printf("[x_exp]:左 = %.2gm, 右 = %.2gm\n", lleg->x_exp, rleg->x_exp);
    printf("[右]x_err = %f\n", -rleg->x_exp + rleg->x);
    printf("[左]x_err = %f\n", -lleg->x_exp + lleg->x);

    printf("[右]phi1 = %f\n", rleg->phi1);
    printf("[左]phi1 = %f\n", lleg->phi1);
    printf("[右]phi4 = %f\n", rleg->phi4);
    printf("[左]phi4 = %f\n", lleg->phi4);
    printf("[右]F = %f\n", rleg->F);
    printf("[左]F = %f\n", lleg->F);

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
    test_rleg_pub.publish(TEST_setLegModelDataFromStruct(*rleg));
    test_lleg_pub.publish(TEST_setLegModelDataFromStruct(*lleg));
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
    return 423.6 * l4 - 456.93 * l3 + 183.94 * l2 - 33.22 * l + 2.8118;
}

void VMC::leg_length_control() // 腿长控制 包含了横滚控制和跳跃控制
{
    roll_pid->update(roll);
    static ros::Publisher test_pub = n.advertise<std_msgs::Float32>("/leg_exp", 1000);

    if(legPidState==PID_NORM)
    {
        lleg->L0_pid->exp_set(lleg->L0_exp);
        rleg->L0_pid->exp_set(rleg->L0_exp);
        lleg->L0_pid->update(lleg->L0);
        rleg->L0_pid->update(rleg->L0);
        std_msgs::Float32 msg_temp;
        msg_temp.data=abs(-(lleg->L0_exp-0.01)/2*abs(lleg->phi)+lleg->L0_exp);
        test_pub.publish(msg_temp);
    }
    
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

    // L0_pid_correct(lleg);
    // L0_pid_correct(rleg);
    

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

    lleg->Tp = LIMIT(lleg->Tp, 1.5);//test
    rleg->Tp = LIMIT(rleg->Tp, 1.5);//test

    // 腿长pid及横滚控制
    leg_length_control();

    // 两腿theta一致补偿
    theta_pid->update(lleg->theta - rleg->theta);
    lleg->Tp -= theta_pid->control();
    rleg->Tp += theta_pid->control();

    


    // 求前后髋关节电机输出力矩 T1前 T2后
    leg_conv(lleg);
    leg_conv(rleg);

    lleg->T = LIMIT(lleg->T, 1.5);//test
    rleg->T = LIMIT(rleg->T, 1.5);//test

    // 转向控制 输入为yaw & yaw_exp，向T_l & T_r 上叠加修正量
    turn_control();
    // 后面必须是dx_exp 因为如果速度大于速度期望，就发散了
    if(dyaw_exp != 0)
    {
        yaw_exp += dyaw_exp * TIME_STEP;
    }

    double phi_max = 0.15;
    // if (!((abs(rleg->phi) <= phi_max) || 
    // (rleg->phi <= -phi_max && lleg->x_exp <= 0) || 
    // (rleg->phi >= phi_max && lleg->x_exp >= 0)))
    // {
    //     lleg->x_exp = lleg->x;
    //     lleg->dx_exp = 0;
    // }

    if(lleg->dx_exp != 0)
    {
        lleg->x_exp = lleg->x;
    }

    // if (!((abs(rleg->phi) <= phi_max) || 
    // (rleg->phi <= -phi_max && rleg->x_exp <= 0) || 
    // (rleg->phi >= phi_max && rleg->x_exp >= 0)))
    // {
    //     rleg->x_exp = rleg->x;
    //     rleg->dx_exp = 0;
    // }
    
    if(rleg->dx_exp != 0)
    {
        rleg->x_exp = rleg->x;
    }

    float k = 0.01; // 比例系数
    float threshold = 0.001; // 误差阈值

    lleg->x_err = lleg->x_exp - lleg->x;
    rleg->x_err = rleg->x_exp - rleg->x;

    float err_diff = fabs(lleg->x_err - rleg->x_err); // 计算左右误差差值

    if (err_diff > threshold) // 当左右误差差值超过阈值时才调整
    {
        if (lleg->x_err > rleg->x_err) // 左err大于右err，左转时发生
        {
            float adjustment = k * (lleg->x_err - rleg->x_err); // 调整量与误差成比例
            rleg->x_exp += adjustment;
            // lleg->x_exp -= adjustment;
        }
        else // 右err大于左err，右转时发生
        {
            float adjustment = k * (rleg->x_err - lleg->x_err); // 调整量与误差成比例
            rleg->x_exp -= adjustment;
            // lleg->x_exp += adjustment;
        }
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
    vmc_msg.if_speed_mode = false;
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
        if (lleg->phi1 < 1.5 || lleg->phi1 > 4.11 || lleg->phi4 < -0.96 || lleg->phi4 > 1.5)
        {
            ROS_ERROR("Wrong!,left_leg phi1 = %f度  phi4=%f度", lleg->phi1 * 180 / pi, lleg->phi4 * 180 / pi);
            lleg->err_counter++;
        }
        else
        {
            rleg->err_counter = 0;
        }

        // 后电机为phi1取值范围为[1.5，4]   前电机为phi4取值范围为[-1，1.5]
        if (rleg->phi1 < 1.5 || rleg->phi1 > 4.11 || rleg->phi4 < -0.96 || rleg->phi4 > 1.5)
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
        printf("Tl=%f\n",vmc_msg.T_l);
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
        // ros::shutdown();
    }
}

void VMC::run() // 设定控制周期的主循环，传感器回调，控制器计算，pub控制信息
{
    double total_time = 0;
    std::chrono::duration<double, std::milli> rate(TIME_STEP * 1000);
    std_msgs::String msg1;
    while (ros::ok())
    {
        high_resolution_clock::time_point h_start = high_resolution_clock::now();
        switch (robotstate)
        {
        case NORM: // VMC控制
            Plan_exp_set();
            VMC_controller(); // 控制器  //这个函数里面应该包括检测电磁铁通断的内容
            state_print();    // 打印状态
            if(PROTECT_Program())
            {
                break;
            }
            counter_P++;
            if (counter_P > init_time / TIME_STEP)
            {
                VMC_output(); // 对输出进行限幅或衰减、判断机器人状态是否安全 发送安全的控制命令到接口 interface
            }
            else
            {
                cout << "!!!!!" << init_time - counter_P * TIME_STEP << "s后启动" << endl;
                if(counter_P%(int)(CONTROL_RATE/2))
                {
                    msg1.data = "INITING";
                    test_robotstate_pub.publish(msg1);
                }
                
            }
            if (mag_slave_left_back && mag_slave_right_back)
            {
                robotstate = COMB;
            }
            
            break;
        case NORM_TO_INIT:
            if(lleg->L0 <= 0.11 && rleg->L0 <= 0.11)
            {
                robotstate = INIT;
                std_msgs::String msg;
                msg.data = "NORM_TO_INIT";
                test_robotstate_pub.publish(msg);
                
                
                // Leg_PID_Clear();
                leg_init(lleg);
                leg_init(rleg);
                break;
            }
            
            lleg->L0_exp = 0.09;
            rleg->L0_exp = 0.09;
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
            counter_P = 0;
            yaw_exp = yaw;
            lleg->x_exp = 0;
            rleg->x_exp = 0;
            cout << "按r初始化, 按p进入伏地模式" << endl;
            break;
        case PRONE:
            // Prone_VMC_controller(); // 控制器  //这个函数里面应该包括检测电磁铁通断的内容
            // state_print();    // 打印状态
            counter_P++;
            if (counter_P > init_time / TIME_STEP)
            {
                // VMC_output(); // 对输出进行限幅或衰减、判断机器人状态是否安全 发送安全的控制命令到接口 interface
                Prone_VMC_output();
            }
            else
            {
                cout << "!!!!!" << init_time - counter_P * TIME_STEP << "s后启动" << endl;
                
            }
            break;
        case COMB: // 拼接后控制
            CMB_controller();
            CMB_output();
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
        cout << "控制周期:" << Period_Time_ms << "ms" << endl
             << endl
             << endl;
        if (Period_Time_ms > TIME_STEP * 10 * 1000 || Period_Time_ms < TIME_STEP * 0.1 * 1000)
        {
            if (counter_P > init_time / TIME_STEP + 1)
            {
                ROS_ERROR("死机");
                isRobotSafe = false;
            }
        }
        ros::spinOnce();
    }
}

void VMC::CMB_controller()
{
    // 暂时先保持这个状态，然后去对接
    // 去掉两个0，再除以8
    lleg->phi1 = -24000 + slave_l_phi1; // 向下
    lleg->phi4 = 24000 + slave_l_phi4;
    lleg->T = 0 + slave_T1;

    rleg->phi1 = 24000 + slave_r_phi1;
    rleg->phi4 = -24000 + slave_r_phi4;
    rleg->T = 0 + slave_T2;
}

void VMC::CMB_output()
{
    message::cmb_interface cmb_msg;
    message::cmb_interface slave_cmb_msg;
    cmb_msg.T_l = 0;
    cmb_msg.P1_l = 0;
    cmb_msg.P2_l = 0;
    cmb_msg.T_r = 0;
    cmb_msg.P1_r = 0;
    cmb_msg.P2_r = 0;
    cmb_msg.flag = 0;

    slave_cmb_msg.T_l = 0;
    slave_cmb_msg.P1_l = 0;
    slave_cmb_msg.P2_l = 0;
    slave_cmb_msg.T_r = 0;
    slave_cmb_msg.P1_r = 0;
    slave_cmb_msg.P2_r = 0;
    slave_cmb_msg.flag = 0;

    cmb_msg.T_l = lleg->T * wheel_protect;           // 轮毂依旧输出扭矩信息控制速度
    cmb_msg.P1_l = static_cast<int16_t>(lleg->phi1); // 髋关节输出位置信息
    cmb_msg.P2_l = static_cast<int16_t>(lleg->phi4);

    cmb_msg.T_r = rleg->T * wheel_protect;
    cmb_msg.P1_r = static_cast<int16_t>(rleg->phi1);
    cmb_msg.P2_r = static_cast<int16_t>(rleg->phi4);

    cmb_output_pub.publish(cmb_msg); // 发布本机的两个控制节点

    // slave_cmb_msg.P1_l = static_cast<int16_t>(slave_l_phi1);
    // slave_cmb_msg.P2_l = static_cast<int16_t>(slave_l_phi4);
    // slave_cmb_msg.P1_r = static_cast<int16_t>(slave_r_phi1);
    // slave_cmb_msg.P2_r = static_cast<int16_t>(slave_r_phi4);
    // slave_cmb_msg.T_l = slave_T1;
    // slave_cmb_msg.T_r = slave_T2;
    // cmb_slave_output_pub.publish(slave_cmb_msg);//发送给UDP_pkg
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
}

void VMC::Prone_VMC_controller()
{
    leg_pos(lleg); // 求实际 L0 phi0(theta)
    leg_pos(rleg); // 求实际 L0 phi0(theta)
    leg_spd(lleg); // 求腿部运动速度 dL0 dphi0(dtheta)
    leg_spd(rleg); // 求腿部运动速度 dL0 dphi0(dtheta)

    // lleg->dtheta=lleg->theta_filter->update(lleg->dtheta);
    // rleg->dtheta=rleg->theta_filter->update(rleg->dtheta);

    // LQR_K(lleg); // 求对应腿长的K矩阵
    // LQR_K(rleg); // 求对应腿长的K矩阵

    FN_solve(lleg); // 求解支持力
    FN_solve(rleg); // 求解支持力

    // lqr反馈量计算
    // T_Calc(lleg);
    // Tp_Calc(lleg);
    // T_Calc(rleg);
    // Tp_Calc(rleg);

    // 腿长pid及横滚控制
    leg_length_control();
    // lleg->F = 20;
    // rleg->F = 20;

    // 两腿theta一致补偿
    theta_pid->update(lleg->theta - rleg->theta);
    lleg->Tp -= theta_pid->control();
    rleg->Tp += theta_pid->control();

    // 求前后髋关节电机输出力矩 T1前 T2后
    leg_conv(lleg);
    leg_conv(rleg);

    // // 后面必须是dx_exp 因为如果速度大于速度期望，就发散了
    // if (lleg->dx_exp != 0)
    // {
    //     lleg->x_exp += lleg->dx_exp * TIME_STEP;
    // }

    // if (rleg->dx_exp != 0)
    // {
    //     rleg->x_exp += rleg->dx_exp * TIME_STEP;
    // }

    // lleg->x_err = lleg->x_exp - lleg->x;
    // rleg->x_err = rleg->x_exp - rleg->x;
    // if (lleg->x_err > rleg->x_err) // 左err大于右err，左转时发生
    // {
    //     rleg->x_exp += 0.001;
    //     lleg->x_exp -= 0.001;
    // }
    // else // 右err大于左err，右转时发生
    // {
    //     rleg->x_exp -= 0.001;
    //     lleg->x_exp += 0.001;
    // }
}

void VMC::Prone_VMC_output()
{
    message::vmc_interface vmc_msg;
    vmc_msg.T_l = 0;
    vmc_msg.T1_l = 0;
    vmc_msg.T2_l = 0;
    vmc_msg.T_r = 0;
    vmc_msg.T1_r = 0;
    vmc_msg.T2_r = 0;
    vmc_msg.flag = 0;
    vmc_msg.speed_l = 0;
    vmc_msg.speed_r = 0;
    vmc_msg.if_speed_mode = true;
    if (hip_protect != 0 || wheel_protect != 0 || !isTestMode)
    {
        ROS_WARN("！！！！先断电再关控制！！！！");
    }
    // 限制最大扭矩，国际标准单位
    // lleg->T = LIMIT(lleg->T, foot_limit);
    lleg->T1 = LIMIT(lleg->T1, hip_limit);
    lleg->T2 = LIMIT(lleg->T2, hip_limit);
    // rleg->T = LIMIT(rleg->T, foot_limit);
    rleg->T1 = LIMIT(rleg->T1, hip_limit);
    rleg->T2 = LIMIT(rleg->T2, hip_limit);
    rleg->dx_exp = LIMIT(rleg->dx_exp, prone_speed_limit);
    lleg->dx_exp = LIMIT(lleg->dx_exp, prone_speed_limit);
    if (counter_P > init_time / TIME_STEP + 1)
    {
        // if (lleg->phi0 < 40 * pi / 180 || lleg->phi0 > 140 * pi / 180 || rleg->phi0 < 40 * pi / 180 || rleg->phi0 > 140 * pi / 180)
        // {
        //     ROS_ERROR("Wrong!,phi0 = %f度/%f度", lleg->phi0 * 180 / pi, rleg->phi0 * 180 / pi);
        //     isRobotSafe = false;
        // }
        // 后电机为phi1取值范围为[1.5，4]   前电机为phi4取值范围为[-1，1.5]
        if (lleg->phi1 < 1.5 || lleg->phi1 > 4.11 || lleg->phi4 < -0.96 || lleg->phi4 > 1.5)
        {
            ROS_ERROR("Wrong!,left_leg phi1 = %f度  phi4=%f度", lleg->phi1 * 180 / pi, lleg->phi4 * 180 / pi);
            lleg->err_counter++;
        }
        else
        {
            rleg->err_counter = 0;
        }

        // 后电机为phi1取值范围为[1.5，4]   前电机为phi4取值范围为[-1，1.5]
        if (rleg->phi1 < 1.5 || rleg->phi1 > 4.11 || rleg->phi4 < -0.96 || rleg->phi4 > 1.5)
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
        // vmc_msg.T_l = lleg->T * wheel_protect;
        // printf("Tl=%f\n",vmc_msg.T_l);
        vmc_msg.speed_l = lleg->dx_exp;
        vmc_msg.T1_l = T2iqControl_MG6012(lleg->T1 * hip_protect);
        vmc_msg.T2_l = T2iqControl_MG6012(lleg->T2 * hip_protect);

        // vmc_msg.T_r = T2iqControl_MF9015(rleg->T * wheel_protect);
        // vmc_msg.T_r = rleg->T * wheel_protect;
        vmc_msg.speed_r = rleg->dx_exp;
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
        // ros::shutdown();
    }

}

// 将 Leg_Model 结构体转换为 ROS 消息类型
message::test_msg VMC::TEST_setLegModelDataFromStruct(const Leg_Model& leg_model) {
    static message::test_msg leg_msg;
    leg_msg.isRobotOffGround = leg_model.isRobotOffGround;
    leg_msg.counter_OffGround = leg_model.counter_OffGround;
    leg_msg.phi1 = leg_model.phi1;
    leg_msg.phi4 = leg_model.phi4;
    leg_msg.dphi1 = leg_model.dphi1;
    leg_msg.dphi4 = leg_model.dphi4;
    leg_msg.L0 = leg_model.L0;
    leg_msg.L0_exp = leg_model.L0_exp;
    leg_msg.phi0 = leg_model.phi0;
    
    leg_msg.dphi0 = leg_model.dphi0;
    leg_msg.F = leg_model.F;
    leg_msg.F_gravity = leg_model.F_gravity;
    leg_msg.F_inertia = leg_model.F_inertia;
    leg_msg.Tp = leg_model.Tp;
    leg_msg.T = leg_model.T;
    leg_msg.last_T = leg_model.last_T;
    leg_msg.T1 = leg_model.T1;
    leg_msg.T2 = leg_model.T2;
    leg_msg.theta = leg_model.theta;
    leg_msg.dtheta = leg_model.dtheta;
    leg_msg.x = leg_model.x;
    leg_msg.dx = leg_model.dx;
    leg_msg.phi = leg_model.phi;
    leg_msg.dphi = leg_model.dphi;
    leg_msg.dyaw = leg_model.dyaw;
    
    leg_msg.theta_exp = leg_model.theta_exp;
    leg_msg.dtheta_exp = leg_model.dtheta_exp;
    leg_msg.x_exp = leg_model.x_exp;
    leg_msg.dx_exp = leg_model.dx_exp;
    leg_msg.phi_exp = leg_model.phi_exp;
    leg_msg.dphi_exp = leg_model.dphi_exp;
    leg_msg.x_err = leg_model.x_err;


    leg_msg.speed_now = leg_model.speed_now;
    leg_msg.speed_last = leg_model.speed_last;
    leg_msg.FN = leg_model.FN;
    
    // Assuming L0_pid and theta_filter are pointers to double
    leg_msg.nl = leg_model.nl;
    leg_msg.err_counter = leg_model.err_counter;

    return leg_msg;
}

void VMC::Leg_PID_Clear()
{
    free(theta_pid);
    free(turn_pid);
    free(roll_pid);
    free(lleg->L0_pid);
    free(rleg->L0_pid);
    theta_pid = new PID(1 / CONTROL_RATE, theta_pid_index, 0, 10, yaw);
    turn_pid = new PID(1 / CONTROL_RATE, turn_pid_index, 0, 3, 0);
    roll_pid = new PID(1 / CONTROL_RATE, roll_pid_index, 0, 100, roll_exp);
    lleg->L0_pid = new PID(1 / CONTROL_RATE, l0_pid_index, 40, 40, lleg->L0_exp);
    rleg->L0_pid = new PID(1 / CONTROL_RATE, l0_pid_index, 40, 40, rleg->L0_exp);
    
}

bool VMC::PROTECT_Program()
{
    if(rleg->phi > 0.25)
    {
        robotstate = INIT;


        std_msgs::String msg;
        msg.data = "PROTECT_FALL_BACK";
        test_robotstate_pub.publish(msg);
        
        leg_init(lleg);
        leg_init(rleg);
        return true;
    }
    else if(rleg->phi < -0.25)
    {
        robotstate = INIT;

        std_msgs::String msg;
        msg.data = "PROTECT_FALL_FRONT";
        test_robotstate_pub.publish(msg);
        
        leg_init(lleg);
        leg_init(rleg);
        return true;
    }

    return false;
    
}

