#include "interface.h"
#include <iostream>
using namespace std;
int main(int argc, char **argv)
{
    setlocale(LC_ALL, ""); // 调整编码，可以显示中文
    ros::init(argc, argv, "robot_init");
    ros::NodeHandle n;
    Interface *init = new Interface();
    VCI_CAN_OBJ init_recv_msg[100];
    int init_mode = 3;
    int counter = 0;

    n.getParam("robot_init_node/mode", init_mode);
    switch (init_mode)
    {
    case -1:
        break;
    case 0: // 机械臂零点写入
        cout << "机械臂零点写入" << endl;
        init->M_Motor_Set_Zero(0x00000147);
        init->M_Motor_Set_Zero(0x00000148);
        init->M_Motor_Set_Zero(0x00000149);
        init->M_Motor_Set_Zero(0x0000014A);
        init->M_Motor_Set_Zero(0x0000014B);
        init->M_Motor_Set_Zero(0x0000014C);
        break;
    case 1: // 机械臂零点测试
        init->M_Motor_Pos_Transmit(0x0000014C, 0, 80);
        // init->M_Motor_Pos_Transmit(0x00000148, 0, 80);
        // init->M_Motor_Pos_Transmit(0x00000149, 0, 80);
        // init->M_Motor_Pos_Transmit(0x0000014A, 0, 80);
        // init->M_Motor_Pos_Transmit(0x0000014B, 0, 80);
        // init->M_Motor_Pos_Transmit(0x0000014C, 0, 80);
        break;
    case 2: // 髋关节零点写入
        // init->Motor_Set_Zero(0x00000143);
        // VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);
        // init->Motor_Set_Zero(0x00000144);
        // VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);
        init->Motor_Set_Zero(0x00000144);
        VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);
        // init->Motor_Set_Zero(0x00000146);
        // VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);
        break;
    case 3: // 髋关节零点测试

        while (counter < 10)
        {
            init->Cyber_Gear_Motor_Select(0x00000002, 2);
            init->Cyber_Gear_Motor_Enable(0x00000002);
            init->Cyber_Gear_Motor_Speed_Limit(0x00000002, 26.0);
            init->Cyber_Gear_Motor_Select(0x00000001, 2);
            init->Cyber_Gear_Motor_Enable(0x00000001);
            init->Cyber_Gear_Motor_Speed_Limit(0x00000001, 26.0);
            counter++;
            printf("counter=%d\n", counter);
            ros::Duration(0.5).sleep();
        }
        // init->Cyber_Gear_Motor_Speed(0x00000001, 1);  //1rad/s
        //
        //  while (1)
        //  {
        //      init->Cyber_Gear_Motor_Torque(0x00000002, 0.1);  //调试专用，设置为5nm的扭矩输出
        //      init->Motor_Feedback();
        //  }

        // init->Motor_Pos_Transmit(0x00000143, 0, 80);
        // VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);

        // init->Motor_Pos_Transmit(0x00000144, 0, 80);
        // VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);

        // init->Motor_Pos_Transmit(0x00000145, 0, 80);
        // VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);

        // init->Motor_Pos_Transmit(0x00000146, 0, 80);
        // VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);
        break;
    case 4:
        // init->M_Motor_Set_Zero(0x00000143);
        // init->M_Motor_Pos_Transmit(0x00000143, 0, 80);
        init->Motor_F_Transmit(0x00000142, 80);
        break;
    case 5:
        init->Cyber_Gear_Motor_Select(0x00000001, 0);
        init->Cyber_Gear_Motor_Enable(0x00000001);
        ros::Duration(1).sleep();
        while (1)
        {
            init->Cyber_Gear_Motor_Torque(0x00000001, 0.1); // 调试专用，设置为5nm的扭矩输出
            init->Motor_Feedback();
        }

        // init->Cyber_Gear_Motor_Torque(0x00000002, -1);  //调试专用，设置为5nm的扭矩输出
        break;
        // init->Motor_Feedback();
    case 6:
        init->Motor_Pos_Transmit(0x00000143, 0, 80);
        VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);

        init->Motor_Pos_Transmit(0x00000144, 0, 80);
        VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);

        init->Motor_Pos_Transmit(0x00000145, 0, 80);
        VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);

        init->Motor_Pos_Transmit(0x00000146, 0, 80);
        VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);
    default:
        break;
    }
    return 0;
}