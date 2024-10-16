#include "interface.h"
#include <iostream>

Interface::Interface()
{
    isMinit = false;
    motor_pub = n.advertise<message::interface_controller>("/motor_feedback", 1);         // publisher缓存设定为1，令电机接收最新的消息
    m_motor_pub = n.advertise<message::interface_controller>("/manipulator_feedback", 1); // publisher缓存设定为1，令电机接收最新的消息
    vmc_output_sub = n.subscribe("/vmc_output", 5, &Interface::VMC_Control, this);        // vmc
    m_output_sub = n.subscribe("/manipulator_output", 5, &Interface::M_Control, this);
    test_robotstate_sub = n.subscribe("/robotstate", 5, &Interface::Robot_State_Change, this);
    memset(&motor_feedback_msg, 0, sizeof(message::interface_controller));
    memset(&m_motor_feedback_msg, 0, sizeof(message::interface_controller));
    memset(&pInfo, 0, sizeof(VCI_BOARD_INFO));
    memset(&pInfo1, 0, sizeof(VCI_BOARD_INFO) * 50);



    cmb_output_sub= n.subscribe("/cmb_slave", 5, &Interface::CMB_Control, this);
    // can minipcie
    num = VCI_FindUsbDevice2(pInfo1);
    if (VCI_OpenDevice(VCI_USBCAN2, 0, 1) == 1) // 打开设备
    {
        printf(">>open deivce success!\n"); // 打开设备成功
    }
    else
    {
        printf(">>open deivce error!\n");
        exit(1);
    }
    if (VCI_ReadBoardInfo(VCI_USBCAN2, 0, &pInfo) == 1) // 读取设备序列号、版本等信息。
    {
        printf(">>Get VCI_ReadBoardInfo success!\n");
    }
    else
    {
        printf(">>Get VCI_ReadBoardInfo error!\n");
        exit(1);
    }
    // 初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;     // 接收所有帧
    config.Timing0 = 0x00; /*波特率1000 Kbps*/
    config.Timing1 = 0x14;
    config.Mode = 0; // 正常模式

    if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != 1)
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }

    if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }

    if (VCI_InitCAN(VCI_USBCAN2, 0, 1, &config) != 1)
    {
        printf(">>Init can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }
    if (VCI_StartCAN(VCI_USBCAN2, 0, 1) != 1)
    {
        printf(">>Start can2 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }
    VCI_ClearBuffer(VCI_USBCAN2, 0, 1);
    VCI_ClearBuffer(VCI_USBCAN2, 0, 0);
}

void Interface::Robot_Init(int angle) // 机器人初始化，设置髋关节电机位置为给定值
{
    printf("---------[ 开始初始化。。。。。。 ]---------\n");
    auto startTime_init = ros::Time::now().toNSec(); // 开始计时
    Motor_Pos_Transmit(0x00000143, angle, 180);      // 40000
    Motor_Feedback();
    Motor_Pos_Transmit(0x00000144, -angle, 180); //-40000
    Motor_Feedback();
    Motor_Pos_Transmit(0x00000145, -angle, 180); //-40000
    Motor_Feedback();
    Motor_Pos_Transmit(0x00000146, angle, 180); // 40000
    Motor_Feedback();

    Cyber_Gear_Motor_Select(0x00000001, 0); // 模式选择
    Cyber_Gear_Motor_Enable(0x00000001);    // 使能

    Cyber_Gear_Motor_Select(0x00000002, 0); // 模式选择
    Cyber_Gear_Motor_Enable(0x00000002);    // 使能

    int flag_init = 0;
    usleep(100);
    while (motor_feedback_msg.speed3 != 0.00f || motor_feedback_msg.speed4 != 0.00f || motor_feedback_msg.speed5 != 0.00f || motor_feedback_msg.speed6 != 0.00f)
    {
        Motor_Check(0x00000143);
        Motor_Feedback();
        Motor_Check(0x00000144);
        Motor_Feedback();
        Motor_Check(0x00000145);
        Motor_Feedback();
        Motor_Check(0x00000146);
        Motor_Feedback();
    }
    auto endTime_init = ros::Time::now().toNSec(); // IMU回调函数执行一次计时终点
    if (!flag_init)
    {
        std::cout << "---------[ 初始化成功，历时" << (double)(endTime_init - startTime_init) / 10e6 << "ms ]---------" << std::endl;
    }
    motor_feedback_msg.header.stamp = ros::Time::now();
    motor_pub.publish(motor_feedback_msg);
}

void Interface::ExitCan(int flag) // 报错意外退出can口
{
    // 关闭设备
    VCI_CloseDevice(4, 0);
    exit(0);
}

void Interface::CloseCan() // 关闭can口
{
    VCI_CloseDevice(4, 0);
}

int Interface::sign(int x) // 符号函数
{
    int s;
    if (x >= 0)
        s = 0;
    if (x < 0)
        s = -1;
    return s;
}

void Interface::Motor_State()
{
    Motor_Check(0x00000141);
    Motor_Feedback();
    Motor_Check(0x00000142);
    Motor_Feedback();
    Motor_Check(0x00000143);
    Motor_Feedback();
    Motor_Check(0x00000144);
    Motor_Feedback();
    Motor_Check(0x00000145);
    Motor_Feedback();
    Motor_Check(0x00000146);
    Motor_Feedback();
    motor_feedback_msg.header.stamp = ros::Time::now();
    motor_pub.publish(motor_feedback_msg);
}

void Interface::VMC_Control(const message::vmc_interface::ConstPtr &msg)
{
    printf("---------[ VMC控制 ]---------\n");
    auto startTime_vmc = ros::Time::now().toNSec(); // 开始计时
    temprature_err_flag = false;
    memset(sent_msg, 0, sizeof(VCI_CAN_OBJ) * 1);
    memset(recv_msg, 0, sizeof(VCI_CAN_OBJ) * 10);
    motor_feedback_msg.call1 = false;
    motor_feedback_msg.call2 = false;
    motor_feedback_msg.call3 = false;
    motor_feedback_msg.call4 = false;
    motor_feedback_msg.call5 = false;
    motor_feedback_msg.call6 = false;
    bool isRobotUnsafe = false;
    if (msg->flag == 0 || temprature_err_flag)
    {
        // 0705jidegai
        Motor_F_Transmit(0x00000141, 0);
        Motor_Feedback();
        Motor_F_Transmit(0x00000142, 0);
        Motor_Feedback();

        Motor_F_Transmit(0x00000143, 0);
        Motor_Feedback();
        Motor_F_Transmit(0x00000144, 0);
        Motor_Feedback();
        Motor_F_Transmit(0x00000145, 0);
        Motor_Feedback();
        Motor_F_Transmit(0x00000146, 0);
        Motor_Feedback();
        if (msg->flag == 0)
        {
            std_msgs::String msg;
            // msg.data = "STOP";
            // auto msg_ptr = boost::make_shared<std_msgs::String>(msg);
            // Robot_State_Change(msg_ptr);
            ROS_ERROR("ERR! Controller is closed! \n");
        }
        if (temprature_err_flag)
        {
            ROS_ERROR("电机关闭,温度过高");
        }
        ros::shutdown();
    }
    else if (msg->if_speed_mode)
    {
        motor_feedback_msg.recv = 0;
        int counter_resent = 0;
        const int max_resent = 2;
        // 轮毂扭矩
        counter_resent = 0; // you轮
        while (counter_resent < max_resent)
        {
            if (motor_feedback_msg.call1)
                break;
            // Motor_F_Transmit(0x00000141, (-1) * msg->T_r);
            Cyber_Gear_Motor_Speed(0x00000002, (1)* msg->speed_r);
            Motor_Feedback();
            counter_resent++;
        }
        counter_resent = 0; // zuo轮
        while (counter_resent < max_resent)
        {
            if (motor_feedback_msg.call2)
                break;
            // Motor_F_Transmit(0x00000142, (1) * msg->T_l);
            Cyber_Gear_Motor_Speed(0x00000001, (-1) * msg->speed_l);
            Motor_Feedback();
            counter_resent++;
        }
    }
    else
    {
        motor_feedback_msg.recv = 0;
        int counter_resent = 0;
        const int max_resent = 2;
        // 轮毂扭矩
        counter_resent = 0; // you轮
        while (counter_resent < max_resent)
        {
            if (motor_feedback_msg.call1)
                break;
            // Motor_F_Transmit(0x00000141, (-1) * msg->T_r);
            Cyber_Gear_Motor_Torque(0x00000002, (1) * msg->T_r);
            Motor_Feedback();
            counter_resent++;
        }
        counter_resent = 0; // zuo轮
        while (counter_resent < max_resent)
        {
            if (motor_feedback_msg.call2)
                break;
            // Motor_F_Transmit(0x00000142, (1) * msg->T_l);
            Cyber_Gear_Motor_Torque(0x00000001, (-1) * msg->T_l);
            Motor_Feedback();
            counter_resent++;
        }
        counter_resent = 0; // 右后
        while (counter_resent < max_resent)
        {
            if (motor_feedback_msg.call3)
                break;
            Motor_F_Transmit(0x00000143, (-1) * msg->T2_r);
            Motor_Feedback();
            counter_resent++;
        }
        counter_resent = 0; // 左后
        while (counter_resent < max_resent)
        {
            if (motor_feedback_msg.call4)
                break;
            Motor_F_Transmit(0x00000144, 1 * msg->T2_l);
            Motor_Feedback();
            counter_resent++;
        }
        counter_resent = 0; // 右前
        while (counter_resent < max_resent)
        {
            if (motor_feedback_msg.call5)
                break;
            Motor_F_Transmit(0x00000145, (-1) * msg->T1_r);
            Motor_Feedback();
            counter_resent++;
        }
        counter_resent = 0; // 左前
        while (counter_resent < max_resent)
        {
            if (motor_feedback_msg.call6)
                break;
            Motor_F_Transmit(0x00000146, 1 * msg->T1_l);
            Motor_Feedback();
            counter_resent++;
        }
    }
    motor_feedback_msg.flag = 0;
    motor_feedback_msg.header.stamp = ros::Time::now();
    motor_pub.publish(motor_feedback_msg);
    ROS_INFO("消息已发布");
    VCI_ClearBuffer(4, 0, 0);
}

void Interface::M_Control(const message::manipulator::ConstPtr &msg)
{
    int speed_limit = 300;
    memset(m_sent_msg, 0, sizeof(VCI_CAN_OBJ) * 1);
    memset(m_recv_msg, 0, sizeof(VCI_CAN_OBJ) * 10);
    double angler = -(m_motor_feedback_msg.encoder3) / (65535 / 2 / pi);
    double anglel = (m_motor_feedback_msg.encoder4) / (65535 / 2 / pi);

    cout << "look         " << anglel << "       " << angler << endl;
    if (!isMinit)
    {
        // if (anglel > 2.5 || anglel < -2.5)
        {
            ROS_ERROR("LINIT");
            M_Motor_Pos_I_Transmit(0x0000014A, -600, speed_limit);
            M_Motor_Feedback();
        }
        // if (angler > 2.5 || angler < -2.5)
        {
            ROS_ERROR("RINIT");
            M_Motor_Pos_I_Transmit(0x00000149, 600, speed_limit);
            M_Motor_Feedback();
        }
        if (anglel < 2 && anglel > 0 && angler < 2 && angler > 0)
        {
            isMinit = true;
        }
        ROS_WARN("INITMODE");
    }
    else
    {
        M_Motor_Pos_Transmit(0x0000014A, msg->l1, speed_limit);
        M_Motor_Feedback();
        M_Motor_Pos_Transmit(0x00000149, msg->r1, speed_limit);
        M_Motor_Feedback();
    }
    M_Motor_Pos_Transmit(0x00000148, msg->l0, speed_limit);
    M_Motor_Feedback();
    M_Motor_Pos_Transmit(0x0000014C, msg->l2, speed_limit);
    M_Motor_Feedback();
    M_Motor_Pos_Transmit(0x00000147, msg->r0, speed_limit);
    M_Motor_Feedback();
    M_Motor_Pos_Transmit(0x0000014B, msg->r2, speed_limit);
    M_Motor_Feedback();
    m_motor_feedback_msg.flag = 0;
    m_motor_feedback_msg.header.stamp = ros::Time::now();
    m_motor_pub.publish(m_motor_feedback_msg);
    VCI_ClearBuffer(4, 0, 1);
}

void Interface::M_Motor_Feedback() // 接收电机反馈消息并判断ID
{
    int recvlen;
    int i, j;
    recvlen = VCI_Receive(4, 0, 1, m_recv_msg, 6, 10); // 6叶树：4 0 0 10 0
    for (i = 0; i < recvlen; i++)
    {
        u_int8_t vel_h = m_recv_msg[i].Data[5];
        u_int8_t vel_l = m_recv_msg[i].Data[4];
        int16_t vel = vel_h * 256 + vel_l;
        u_int8_t current_h = m_recv_msg[i].Data[3];
        u_int8_t current_l = m_recv_msg[i].Data[2];
        int16_t current = current_h * 256 + current_l;
        u_int8_t encoder_h = m_recv_msg[i].Data[7];
        u_int8_t encoder_l = m_recv_msg[i].Data[6];
        u_int16_t encoder = encoder_h * 256 + encoder_l;
        if (m_recv_msg[i].Data[1] > 60)
        {
            temprature_err_flag = true;
        }
        if (m_recv_msg[i].ID == 0x00000147)
        {
            m_motor_feedback_msg.speed1 = vel;
            m_motor_feedback_msg.iq1 = current;
            m_motor_feedback_msg.encoder1 = encoder;
            m_motor_feedback_msg.call1 = true;
        }

        if (m_recv_msg[i].ID == 0x00000148)
        {
            m_motor_feedback_msg.speed2 = vel;
            m_motor_feedback_msg.iq2 = current;
            m_motor_feedback_msg.encoder2 = encoder;
            m_motor_feedback_msg.call2 = true;
        }

        if (m_recv_msg[i].ID == 0x00000149)
        {
            m_motor_feedback_msg.speed3 = vel;
            m_motor_feedback_msg.iq3 = current;
            m_motor_feedback_msg.encoder3 = encoder;
            m_motor_feedback_msg.call3 = true;
        }

        if (m_recv_msg[i].ID == 0x0000014A)
        {
            m_motor_feedback_msg.speed4 = vel;
            m_motor_feedback_msg.iq4 = current;
            m_motor_feedback_msg.encoder4 = encoder;
            m_motor_feedback_msg.call4 = true;
        }

        if (m_recv_msg[i].ID == 0x0000014B)
        {
            m_motor_feedback_msg.speed5 = vel;
            m_motor_feedback_msg.iq5 = current;
            m_motor_feedback_msg.encoder5 = encoder;
            m_motor_feedback_msg.call5 = true;
        }

        if (m_recv_msg[i].ID == 0x0000014C)
        {
            m_motor_feedback_msg.speed6 = vel;
            m_motor_feedback_msg.iq6 = current;
            m_motor_feedback_msg.encoder6 = encoder;
            m_motor_feedback_msg.call6 = true;
        }
    }
}

void Interface::Motor_Feedback() // 接收电机反馈消息并判断ID
{
    int recvlen;
    int i, j;
    recvlen = VCI_Receive(4, 0, 0, recv_msg, 6, 10); // 6叶树：4 0 0 10 0
    for (i = 0; i < recvlen; i++)
    {
        u_int8_t vel_h = recv_msg[i].Data[5];
        u_int8_t vel_l = recv_msg[i].Data[4];
        int16_t vel = vel_h * 256 + vel_l;
        u_int8_t current_h = recv_msg[i].Data[3];
        u_int8_t current_l = recv_msg[i].Data[2];
        int16_t current = current_h * 256 + current_l;
        u_int8_t encoder_h = recv_msg[i].Data[7];
        u_int8_t encoder_l = recv_msg[i].Data[6];
        u_int16_t encoder = encoder_h * 256 + encoder_l;
        if (recv_msg[i].Data[1] > 60)
        {
            temprature_err_flag = true;
        }
        /*if (recv_msg[i].ID == 0x00000141)
        {
            motor_feedback_msg.speed1 = -vel;
            motor_feedback_msg.iq1 = current;
            motor_feedback_msg.encoder1 = -encoder;
            motor_feedback_msg.call1 = true;
        }

        if (recv_msg[i].ID == 0x00000142)
        {
            motor_feedback_msg.speed2 = vel;
            motor_feedback_msg.iq2 = current;
            motor_feedback_msg.encoder2 = encoder;
            motor_feedback_msg.call2 = true;
        }*/

        if (recv_msg[i].ID == 0x028002FD) // 小米电机接收部分
        {
            int a = static_cast<int>(recv_msg[i].Data[2] * 256 + recv_msg[i].Data[3]);
            // printf("vel1 = %d\n", a);
            uint16_t angle = static_cast<uint16_t>(recv_msg[i].Data[0] * 256 + recv_msg[i].Data[1]);
            motor_feedback_msg.speed1 = (1)*linearTransform(a, 30); // 是否需要有负号   //0--65535分别对应-30--30
            // 转换完毕的速度是rad/s,因此callback.cpp中也有改动，两个轮毂电机数据去掉了角度转换弧度的pai/180
            motor_feedback_msg.angle1 = (1)*linearTransform(angle, 4*pi);
            motor_feedback_msg.angle1 = Cyber_Gear_Motor_GetAndFix_angle_from_PosEncoder(0x00000002, rwheelangle_loss_sum_due_to_zeroing, motor_feedback_msg.angle1);
            // printf("vel=%f\n", motor_feedback_msg.speed1);
            motor_feedback_msg.call1 = true;
            // 以下两行调试专用
            // motor_feedback_msg.header.stamp = ros::Time::now();
            // motor_pub.publish(motor_feedback_msg);
        }

        if (recv_msg[i].ID == 0x028001FD)
        {
            // vel = static_cast<uint16_t>(recv_msg[i].Data[2] * 256 + recv_msg[i].Data[3]);
            int a = static_cast<int>(recv_msg[i].Data[2] * 256 + recv_msg[i].Data[3]);
            uint16_t angle = static_cast<uint16_t>(recv_msg[i].Data[0] * 256 + recv_msg[i].Data[1]);
            // printf("vel2 = %d\n", vel);
            motor_feedback_msg.speed2 = (-1)*linearTransform(a, 30);
            motor_feedback_msg.angle2 = (-1)*linearTransform(angle, 4*pi);
            motor_feedback_msg.angle2 = Cyber_Gear_Motor_GetAndFix_angle_from_PosEncoder(0x00000001, lwheelangle_loss_sum_due_to_zeroing, motor_feedback_msg.angle2);
            // printf("vel=%f\n", motor_feedback_msg.speed2);
            motor_feedback_msg.call2 = true;
            motor_feedback_msg.header.stamp = ros::Time::now();
            motor_pub.publish(motor_feedback_msg);
        }

        if (recv_msg[i].ID == 0x00000143)
        {
            motor_feedback_msg.speed3 = vel;
            motor_feedback_msg.iq3 = current;
            motor_feedback_msg.encoder3 = encoder;
            motor_feedback_msg.call3 = true;
        }

        if (recv_msg[i].ID == 0x00000144)
        {
            motor_feedback_msg.speed4 = -vel;
            motor_feedback_msg.iq4 = current;
            motor_feedback_msg.encoder4 = -encoder;
            motor_feedback_msg.call4 = true;
        }

        if (recv_msg[i].ID == 0x00000145)
        {
            motor_feedback_msg.speed5 = vel;
            motor_feedback_msg.iq5 = current;
            motor_feedback_msg.encoder5 = encoder;
            motor_feedback_msg.call5 = true;
        }

        if (recv_msg[i].ID == 0x00000146)
        {
            motor_feedback_msg.speed6 = -vel;
            motor_feedback_msg.iq6 = current;
            motor_feedback_msg.encoder6 = -encoder;
            motor_feedback_msg.call6 = true;
        }
    }
}

double Interface::linearTransform(int input, double a)
{
    if (input < 0 || input > 65535)
    {
        // std::cerr << "输入值必须在0到65535之间。" << std::endl;
        return 0;
    }

    double output = (static_cast<double>(input) / 65535) * 2 * a - a;
    return output;
}
void Interface::CMB_Control(const message::cmb_interface::ConstPtr& msg) {

    printf("---------[ CMB对接控制 ]---------\n");

    Cyber_Gear_Motor_Torque(0x00000002, (-1) * msg->T_r);  //轮毂电机还是输出扭矩
    Motor_Feedback();
    Cyber_Gear_Motor_Torque(0x00000001, (1) * msg->T_l); 
    Motor_Feedback();

    Motor_Pos_Transmit(0x00000143, msg->P2_r, 100);    //右后    //关节电机输出关节的位置
    Motor_Feedback();
    Motor_Pos_Transmit(0x00000144, msg->P2_l, 100);    //左后
    Motor_Feedback();
    Motor_Pos_Transmit(0x00000145, msg->P1_r, 100);    //右前
    Motor_Feedback();
    Motor_Pos_Transmit(0x00000146, msg->P2_l, 100);    //左前
    Motor_Feedback();
    motor_feedback_msg.header.stamp = ros::Time::now();
    motor_pub.publish(motor_feedback_msg);
}

void Interface::Robot_State_Change(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data=="NORM")
    {
        for(int i = 0; i < 10; i++)
        {
            Cyber_Gear_Motor_Select(0x00000002, 0);
            Cyber_Gear_Motor_Enable(0x00000002);
            Cyber_Gear_Motor_Select(0x00000001, 0);
            Cyber_Gear_Motor_Enable(0x00000001);
            Cyber_Gear_Motor_Set_Zero(0x00000001);
            Cyber_Gear_Motor_Set_Zero(0x00000002);
        }
        
        Cyber_Gear_Motor_Torque(0x00000001, 0);
        Cyber_Gear_Motor_Torque(0x00000002, 0);
        
    }
    else if(msg->data == "INITING")
    {
        for(int i = 0; i < 3; i++)
        {
            Cyber_Gear_Motor_Select(0x00000002, 0);
            Cyber_Gear_Motor_Enable(0x00000002);
            Cyber_Gear_Motor_Select(0x00000001, 0);
            Cyber_Gear_Motor_Enable(0x00000001);
            Cyber_Gear_Motor_Set_Zero(0x00000001);
            Cyber_Gear_Motor_Set_Zero(0x00000002);
            Cyber_Gear_Motor_Torque(0x00000001, 0);
            Cyber_Gear_Motor_Torque(0x00000002, 0);
        }
    }
    else if(msg->data=="PRONE" || msg->data=="NORM_TO_INIT")
    {
        for(int i = 0; i < 3; i++)
        {
            Cyber_Gear_Motor_Select(0x00000002, 2);
            Cyber_Gear_Motor_Enable(0x00000002);
            Cyber_Gear_Motor_Speed_Limit(0x00000002, 20.0);
            Cyber_Gear_Motor_Select(0x00000001, 2);
            Cyber_Gear_Motor_Enable(0x00000001);
            Cyber_Gear_Motor_Speed_Limit(0x00000001, 20.0);
            Cyber_Gear_Motor_Set_Zero(0x00000001);
            Cyber_Gear_Motor_Set_Zero(0x00000002);
        }

        static double hip_init_angle = 0.6/pi*180;
        Motor_Pos_Transmit(0x00000143, hip_init_angle*100*8, 80);
        Motor_Pos_Transmit(0x00000144, -hip_init_angle*100*8, 80);
        Motor_Pos_Transmit(0x00000145, -hip_init_angle*100*8, 80);
        Motor_Pos_Transmit(0x00000146, hip_init_angle*100*8, 80);
    }
    else if(msg->data=="STOP")
    {
        static double hip_init_angle = 0.6/pi*180;
        Motor_Pos_Transmit(0x00000143, hip_init_angle*100*8, 80);
        Motor_Pos_Transmit(0x00000144, -hip_init_angle*100*8, 80);
        Motor_Pos_Transmit(0x00000145, -hip_init_angle*100*8, 80);
        Motor_Pos_Transmit(0x00000146, hip_init_angle*100*8, 80);
        
        for(int i = 0; i < 3; i++)
        {
            Cyber_Gear_Motor_Select(0x00000002, 0);
            Cyber_Gear_Motor_Enable(0x00000002);
            Cyber_Gear_Motor_Select(0x00000001, 0);
            Cyber_Gear_Motor_Enable(0x00000001);
            Cyber_Gear_Motor_Torque(0x00000001, 0);
            Cyber_Gear_Motor_Torque(0x00000002, 0);
            Cyber_Gear_Motor_Set_Zero(0x00000001);
            Cyber_Gear_Motor_Set_Zero(0x00000002);
        }
        // ros::shutdown();
    }
    else if(msg->data=="PROTECT_FALL_BACK")
    {
        for(int i = 0; i < 3; i++)
        {
            Cyber_Gear_Motor_Select(0x00000002, 1);
            Cyber_Gear_Motor_Enable(0x00000002);
            Cyber_Gear_Motor_Speed_Limit(0x00000002, 10.0);
            Cyber_Gear_Motor_Select(0x00000001, 1);
            Cyber_Gear_Motor_Enable(0x00000001);
            Cyber_Gear_Motor_Speed_Limit(0x00000001, 10.0);
            Cyber_Gear_Motor_Set_Zero(0x00000001);
            Cyber_Gear_Motor_Set_Zero(0x00000002);
            Cyber_Gear_Motor_Loc(0x00000001, 0);
            Cyber_Gear_Motor_Loc(0x00000002, 0);

            printf("counter=%d\n", i);
            // ros::Duration(0.5).sleep();
        }
        
        
        static int Motor_speed = 2*800;
        double angle_f = 0.6/pi*180;
        double angle_b = -45;
        Motor_Pos_Transmit(0x00000143, (angle_b)*100*8, Motor_speed);
        Motor_Pos_Transmit(0x00000144, -(angle_b)*100*8, Motor_speed);
        Motor_Pos_Transmit(0x00000145, -angle_f*100*8, Motor_speed);
        Motor_Pos_Transmit(0x00000146, angle_f*100*8, Motor_speed);
    }
    else if(msg->data=="PROTECT_FALL_FRONT")
    {
        for(int i = 0; i < 3; i++)
        {
            Cyber_Gear_Motor_Select(0x00000002, 1);
            Cyber_Gear_Motor_Enable(0x00000002);
            // Cyber_Gear_Motor_Speed_Limit(0x00000002, 10.0);
            Cyber_Gear_Motor_Select(0x00000001, 1);
            Cyber_Gear_Motor_Enable(0x00000001);
            // Cyber_Gear_Motor_Speed_Limit(0x00000001, 10.0);
            Cyber_Gear_Motor_Set_Zero(0x00000001);
            Cyber_Gear_Motor_Set_Zero(0x00000002);
            Cyber_Gear_Motor_Loc(0x00000001, 0);
            Cyber_Gear_Motor_Loc(0x00000002, 0);
            printf("counter=%d\n", i);
            // ros::Duration(0.5).sleep();
        }
        
        
        static int Motor_speed = 2*800;
        // double angle_f = 0.6/pi*180;
        // double angle_b = -90;
        double angle_f = -45;
        double angle_b = 90;
        // Motor_F_Transmit(0x00000143, 0);
        // Motor_F_Transmit(0x00000144, 0);
        Motor_Pos_Transmit(0x00000143, (angle_b)*100*8, Motor_speed);
        Motor_Pos_Transmit(0x00000144, -(angle_b)*100*8, Motor_speed);
        Motor_Pos_Transmit(0x00000145, -angle_f*100*8, Motor_speed);
        Motor_Pos_Transmit(0x00000146, angle_f*100*8, Motor_speed);
        
    }
    lwheelangle_loss_sum_due_to_zeroing = 0;
    rwheelangle_loss_sum_due_to_zeroing = 0;
}