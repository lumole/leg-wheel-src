#include "interface.h"
#include <iostream>

void Interface::Motor_F_Transmit(int ID, int output) // 电机扭矩闭环控制
{
    sent_msg[0].ID = ID;
    sent_msg[0].RemoteFlag = 0; // unchanged
    sent_msg[0].SendType = 1;   // unchanged
    sent_msg[0].ExternFlag = 0; // unchanged
    sent_msg[0].DataLen = 8;
    sent_msg[0].Data[0] = 0xA1;
    sent_msg[0].Data[1] = 0x00;
    sent_msg[0].Data[2] = 0x00;
    sent_msg[0].Data[3] = 0x00;
    sent_msg[0].Data[4] = (output % 256);                        // 扭矩控制：转矩输入低字节
    sent_msg[0].Data[5] = ((output / 256) % 256) + sign(output); // 扭矩控制：转矩输入高字节
    sent_msg[0].Data[6] = 0x00;
    sent_msg[0].Data[7] = 0x00;
    VCI_Transmit(4, 0, 0, sent_msg, 1);
}

void Interface::Motor_Check(int ID) // 电机状态检测
{
    sent_msg[0].ID = ID;
    sent_msg[0].RemoteFlag = 0; // unchanged
    sent_msg[0].ExternFlag = 0; // unchanged
    sent_msg[0].DataLen = 8;
    sent_msg[0].Data[0] = 0x9C;
    sent_msg[0].Data[1] = 0x00;
    sent_msg[0].Data[2] = 0x00;
    sent_msg[0].Data[3] = 0x00;
    sent_msg[0].Data[4] = 0x00;
    sent_msg[0].Data[5] = 0x00;
    sent_msg[0].Data[6] = 0x00;
    sent_msg[0].Data[7] = 0x00;
    VCI_Transmit(4, 0, 0, sent_msg, 1);
}

void Interface::Motor_Pos_Transmit(int ID, int output, int hip_motor_vel_limit) // 电机位置控制
{
    sent_msg[0].ID = ID;
    sent_msg[0].RemoteFlag = 0; // unchanged
    sent_msg[0].SendType = 0;   // unchanged
    sent_msg[0].ExternFlag = 0; // unchanged
    sent_msg[0].DataLen = 8;
    sent_msg[0].Data[0] = 0xA4; // A8代表相對位移命令
    sent_msg[0].Data[1] = 0x00;
    sent_msg[0].Data[2] = (hip_motor_vel_limit % 256);
    sent_msg[0].Data[3] = ((hip_motor_vel_limit / 256) % 256) + sign(hip_motor_vel_limit); // 代錶速度限制
    sent_msg[0].Data[4] = (output % 256);                                                  // 扭矩控制：电流输入低字节
    sent_msg[0].Data[5] = ((output / 256) % 256) + sign(output);                           // 扭矩控制：电流输入高字节
    sent_msg[0].Data[6] = ((output / 256) / 256) % 256 + sign(output);
    sent_msg[0].Data[7] = (((output / 256) / 256) / 256) % 256 + sign(output);
    VCI_Transmit(4, 0, 0, sent_msg, 1);
}

void Interface::M_Motor_Pos_Transmit(int ID, int output, int hip_motor_vel_limit) // 电机位置控制
{
    m_sent_msg[0].ID = ID;
    m_sent_msg[0].RemoteFlag = 0; // unchanged
    m_sent_msg[0].SendType = 0;   // unchanged
    m_sent_msg[0].ExternFlag = 0; // unchanged
    m_sent_msg[0].DataLen = 8;
    m_sent_msg[0].Data[0] = 0xA4; // A8代表相對位移命令
    m_sent_msg[0].Data[1] = 0x00;
    m_sent_msg[0].Data[2] = (hip_motor_vel_limit % 256);
    m_sent_msg[0].Data[3] = ((hip_motor_vel_limit / 256) % 256) + sign(hip_motor_vel_limit); // 代錶速度限制
    m_sent_msg[0].Data[4] = (output % 256);                                                  // 扭矩控制：电流输入低字节
    m_sent_msg[0].Data[5] = ((output / 256) % 256) + sign(output);                           // 扭矩控制：电流输入高字节
    m_sent_msg[0].Data[6] = ((output / 256) / 256) % 256 + sign(output);
    m_sent_msg[0].Data[7] = (((output / 256) / 256) / 256) % 256 + sign(output);
    VCI_Transmit(4, 0, 1, m_sent_msg, 1);
}

void Interface::M_Motor_Pos_I_Transmit(int ID, int output, int hip_motor_vel_limit) // 电机位置控制
{
    m_sent_msg[0].ID = ID;
    m_sent_msg[0].RemoteFlag = 0; // unchanged
    m_sent_msg[0].SendType = 0;   // unchanged
    m_sent_msg[0].ExternFlag = 0; // unchanged
    m_sent_msg[0].DataLen = 8;
    m_sent_msg[0].Data[0] = 0xA8; // A8代表相對位移命令9C
    m_sent_msg[0].Data[1] = 0x00;
    m_sent_msg[0].Data[2] = (hip_motor_vel_limit % 256);
    m_sent_msg[0].Data[3] = ((hip_motor_vel_limit / 256) % 256) + sign(hip_motor_vel_limit); // 代錶速度限制
    m_sent_msg[0].Data[4] = (output % 256);                                                  // 扭矩控制：电流输入低字节
    m_sent_msg[0].Data[5] = ((output / 256) % 256) + sign(output);                           // 扭矩控制：电流输入高字节
    m_sent_msg[0].Data[6] = ((output / 256) / 256) % 256 + sign(output);
    m_sent_msg[0].Data[7] = (((output / 256) / 256) / 256) % 256 + sign(output);
    VCI_Transmit(4, 0, 1, m_sent_msg, 1);
}

void Interface::Motor_Speed_Transmit(int ID, int speed) // 电机速度闭环控制
{
    sent_msg[0].ID = ID;
    sent_msg[0].RemoteFlag = 0; // unchanged
    sent_msg[0].ExternFlag = 0; // unchanged
    sent_msg[0].DataLen = 8;
    sent_msg[0].Data[0] = 0xA2;
    sent_msg[0].Data[1] = 0x00;
    sent_msg[0].Data[2] = 0x00;
    sent_msg[0].Data[3] = 0x00;
    speed *= 100;
    sent_msg[0].Data[4] = (speed % 256);
    sent_msg[0].Data[5] = ((speed / 256) % 256) + sign(speed);
    sent_msg[0].Data[6] = ((speed / 256) / 256) % 256 + sign(speed);
    sent_msg[0].Data[7] = (((speed / 256) / 256) / 256) % 256 + sign(speed);
    VCI_Transmit(4, 0, 0, sent_msg, 1);
}

void Interface::Motor_Close(int ID) // 关闭电机，不保留之前命令
{
    sent_msg[0].ID = ID;
    sent_msg[0].RemoteFlag = 0; // unchanged
    sent_msg[0].ExternFlag = 0; // unchanged
    sent_msg[0].DataLen = 8;
    sent_msg[0].Data[0] = 0x80; // A8代表相對位移命令
    sent_msg[0].Data[1] = 0x00;
    sent_msg[0].Data[2] = 0x00;
    sent_msg[0].Data[3] = 0x00; // 代錶速度限制
    sent_msg[0].Data[4] = 0x00; // 扭矩控制：电流输入低字节
    sent_msg[0].Data[5] = 0x00; // 扭矩控制：电流输入高字节
    sent_msg[0].Data[6] = 0x00;
    sent_msg[0].Data[7] = 0x00;
    VCI_Transmit(4, 0, 0, sent_msg, 1);
}

void Interface::Motor_Set_Zero(int ID) // 写入零点
{
    // VCI_CAN_OBJ init_recv_msg[100];
    int recvlen = 0;
    sent_msg[0].ID = ID;
    sent_msg[0].RemoteFlag = 0; // unchanged
    sent_msg[0].ExternFlag = 0; // unchanged
    sent_msg[0].DataLen = 8;
    sent_msg[0].Data[0] = 0x19; // A8代表相對位移命令
    sent_msg[0].Data[1] = 0x00;
    sent_msg[0].Data[2] = 0x00;
    sent_msg[0].Data[3] = 0x00; // 代錶速度限制
    sent_msg[0].Data[4] = 0x00; // 扭矩控制：电流输入低字节
    sent_msg[0].Data[5] = 0x00; // 扭矩控制：电流输入高字节
    sent_msg[0].Data[6] = 0x00;
    sent_msg[0].Data[7] = 0x00;
    VCI_Transmit(4, 0, 0, sent_msg, 1);
    usleep(1000);
    recvlen = VCI_Receive(4, 0, 0, recv_msg, 3000, 100);
    printf("%d\n", recvlen);
    if (recvlen == 1)
    {
        printf("%d电机设置0点成功\n", recv_msg[0].ID);
    }
}

void Interface::M_Motor_Set_Zero(int ID) // 写入零点
{
    // VCI_CAN_OBJ init_recv_msg[100];
    int recvlen = 0;
    sent_msg[0].ID = ID;
    sent_msg[0].RemoteFlag = 0; // unchanged
    sent_msg[0].ExternFlag = 0; // unchanged
    sent_msg[0].DataLen = 8;
    sent_msg[0].Data[0] = 0x19; // A8代表相對位移命令
    sent_msg[0].Data[1] = 0x00;
    sent_msg[0].Data[2] = 0x00;
    sent_msg[0].Data[3] = 0x00; // 代錶速度限制
    sent_msg[0].Data[4] = 0x00; // 扭矩控制：电流输入低字节
    sent_msg[0].Data[5] = 0x00; // 扭矩控制：电流输入高字节
    sent_msg[0].Data[6] = 0x00;
    sent_msg[0].Data[7] = 0x00;
    VCI_Transmit(4, 0, 1, sent_msg, 1);
    usleep(1000);
    recvlen = VCI_Receive(4, 0, 1, recv_msg, 3000, 100);
    printf("%d\n", recvlen);
    if (recvlen == 1)
    {
        printf("%d电机设置0点成功\n", recv_msg[0].ID);
    }
}

void Interface::Cyber_Gear_Motor_Torque(int ID, double output)
{

    int torque = static_cast<int>(output * 2730 + 32767);
    sent_msg[0].ID = (ID & 0xFF) |   // ID占据0-7位
                     (torque << 8) | // Torque占据8-23位
                     (1 << 24);
    // sent_msg[0].ID = (ID & 0xFF) | (0x010000 << 8);
    sent_msg[0].RemoteFlag = 0; // unchanged
    sent_msg[0].SendType = 1;   // unchanged
    sent_msg[0].ExternFlag = 1; // 扩展帧，因此ID为29位
    sent_msg[0].DataLen = 8;
    // 由于轮毂电机不需要期望位置和期望速度，因此数据位全部为0
    sent_msg[0].Data[0] = 0x00;
    sent_msg[0].Data[1] = 0x00;
    sent_msg[0].Data[2] = 0x00;
    sent_msg[0].Data[3] = 0x00;
    sent_msg[0].Data[4] = 0x00;
    sent_msg[0].Data[5] = 0x00;
    sent_msg[0].Data[6] = 0x00;
    sent_msg[0].Data[7] = 0x00;
    VCI_Transmit(4, 0, 0, sent_msg, 1);
}
// 12 00 FD 01
void Interface::Cyber_Gear_Motor_Select(int ID, int mode)
{

    sent_msg[0].ID = (ID & 0xFF) | (0x1200FD << 8);
    sent_msg[0].RemoteFlag = 0; // unchanged
    sent_msg[0].SendType = 1;   // unchanged
    sent_msg[0].ExternFlag = 1; // 扩展帧，因此ID为29位
    sent_msg[0].DataLen = 8;

    sent_msg[0].Data[0] = 0x05; // 0x7050是选择模式
    sent_msg[0].Data[1] = 0x70;
    sent_msg[0].Data[2] = 0x00;
    sent_msg[0].Data[3] = 0x00;
    sent_msg[0].Data[4] = mode; // 0为运控模式
    sent_msg[0].Data[5] = 0x00;
    sent_msg[0].Data[6] = 0x00;
    sent_msg[0].Data[7] = 0x00;
    VCI_Transmit(4, 0, 0, sent_msg, 1);
}

void Interface::Cyber_Gear_Motor_Enable(int ID)
{
    sent_msg[0].ID = (ID & 0xFF) | (0x0300FD << 8);
    sent_msg[0].RemoteFlag = 0; // unchanged
    sent_msg[0].SendType = 1;   // unchanged
    sent_msg[0].ExternFlag = 1; // 扩展帧，因此ID为29位
    sent_msg[0].DataLen = 8;

    sent_msg[0].Data[0] = 0x00; 
    sent_msg[0].Data[1] = 0x00;
    sent_msg[0].Data[2] = 0x00;
    sent_msg[0].Data[3] = 0x00;
    sent_msg[0].Data[4] = 0x00; 
    sent_msg[0].Data[5] = 0x00;
    sent_msg[0].Data[6] = 0x00;
    sent_msg[0].Data[7] = 0x00;
    VCI_Transmit(4, 0, 0, sent_msg, 1);
}

void Interface::Cyber_Gear_Motor_Speed_Limit(int ID, float limit) {
    sent_msg[0].ID = (ID & 0xFF) | (0x1200FD << 8);
    sent_msg[0].RemoteFlag = 0; // unchanged
    sent_msg[0].SendType = 1;   // unchanged
    sent_msg[0].ExternFlag = 1; // 扩展帧，因此ID为29位
    sent_msg[0].DataLen = 8;

    sent_msg[0].Data[0] = 0x18; // 0x7018是速度限幅模式
    sent_msg[0].Data[1] = 0x70;
    sent_msg[0].Data[2] = 0x00;
    sent_msg[0].Data[3] = 0x00;

    unsigned char* bytePtr = reinterpret_cast<unsigned char*>(&limit);
    for (int i = 4; i < 8; ++i) {
        sent_msg[0].Data[i] = bytePtr[i-4];
    }
    VCI_Transmit(4, 0, 0, sent_msg, 1);
}


void Interface::Cyber_Gear_Motor_Speed(int ID, float output) {
    sent_msg[0].ID = (ID & 0xFF) | (0x1200FD << 8);
    sent_msg[0].RemoteFlag = 0; // unchanged
    sent_msg[0].SendType = 1;   // unchanged
    sent_msg[0].ExternFlag = 1; // 扩展帧，因此ID为29位
    sent_msg[0].DataLen = 8;

    sent_msg[0].Data[0] = 0x0A; // 0x700A是速度命令
    sent_msg[0].Data[1] = 0x70;
    sent_msg[0].Data[2] = 0x00;
    sent_msg[0].Data[3] = 0x00;
    unsigned char* bytePtr = reinterpret_cast<unsigned char*>(&output);
    for (int i = 4; i < 8; ++i) {
        sent_msg[0].Data[i] = bytePtr[i-4];
    }
    VCI_Transmit(4, 0, 0, sent_msg, 1);
}