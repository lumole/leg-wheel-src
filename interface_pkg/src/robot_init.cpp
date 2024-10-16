#include "interface.h"
#include <iostream>

using namespace std;

class TEST
{
private:
    Interface *init;
    ros::NodeHandle n;
    ros::Publisher motor_pub;
    enum MOTOR_MODE
    {
        POSITION_MODE = 0,
        VELOCITY_MODE = 1
    }motor_mode;
    
    
public:
    TEST(Interface *init_){
        init = init_;
        motor_pub = n.advertise<message::interface_controller>("/test_motor_feedback", 1000);
        static double hip_init_angle = 0.6/pi*180;
        init->Motor_Pos_Transmit(0x00000143, hip_init_angle*100*8, 80);
        init->Motor_Pos_Transmit(0x00000144, -hip_init_angle*100*8, 80);
        init->Motor_Pos_Transmit(0x00000145, -hip_init_angle*100*8, 80);
        init->Motor_Pos_Transmit(0x00000146, hip_init_angle*100*8, 80);
    };
    ~TEST(){};
    
    
    void run()
    {
        char op;
        float num;
        ros::Rate rate1(250);
        cin >> op >> num;
        while(ros::ok())
        {
            if (1||cin >> op >> num) {
                if (op == 'x') {
                    std::cout << "位移模式，移动到绝对值 " << num << std::endl;
                    // 在这里添加移动到绝对值的代码
                    
                } else if (op == 'v') {
                    // std::cout << "速度模式，加速到 " << num << " rad/s" << std::endl;
                    // 在这里添加加速的代码
                    Speed_run(num);
                } else {
                    std::cerr << "输入错误: 未知指令 " << op << std::endl;
                }
            } else {
                std::cerr << "输入错误: 格式不正确，请使用 'x <value>' 或 'v <value>'" << std::endl;
            }
            Speed_run(num);
            rate1.sleep();
        }
    }

    void Speed_run(float speed)
    {
        if(motor_mode!=VELOCITY_MODE)
        {
            for(int i = 0; i < 3; i++)
            {
                init->Cyber_Gear_Motor_Select(0x00000002, 2);
                init->Cyber_Gear_Motor_Enable(0x00000002);
                init->Cyber_Gear_Motor_Speed_Limit(0x00000002, 20.0);
                init->Cyber_Gear_Motor_Select(0x00000001, 2);
                init->Cyber_Gear_Motor_Enable(0x00000001);
                init->Cyber_Gear_Motor_Speed_Limit(0x00000001, 20.0);
                init->Cyber_Gear_Motor_Set_Zero(0x00000001);
                init->Cyber_Gear_Motor_Set_Zero(0x00000002);
            }
            motor_mode = VELOCITY_MODE;
        }
        
        init->Cyber_Gear_Motor_Speed(0x00000001, speed);
        init->Motor_Feedback();
    }
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, ""); // 调整编码，可以显示中文
    ros::init(argc, argv, "robot_init");
    ros::NodeHandle n;
    Interface *init = new Interface();
    VCI_CAN_OBJ init_recv_msg[100];
    int init_mode = 3;
    n.getParam("robot_init_node/mode", init_mode);
    ros::Rate rate1(1);
    double hip_init_angle = 0.6/pi*180;
    TEST test(init);
    int speed = 80;//pi*800
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
        init->Motor_Set_Zero(0x00000143);
        VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);
        init->Motor_Set_Zero(0x00000144);
        VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);
        init->Motor_Set_Zero(0x00000145);
        VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);
        init->Motor_Set_Zero(0x00000146);
        VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);
        break;
    case 3: // 髋关节零点测试
        //10次循环：

        for(int i = 0; i < 3; i++)
        {
            init->Cyber_Gear_Motor_Select(0x00000002, 2);
            init->Cyber_Gear_Motor_Enable(0x00000002);
            init->Cyber_Gear_Motor_Speed_Limit(0x00000002, 10.0);
            init->Cyber_Gear_Motor_Select(0x00000001, 2);
            init->Cyber_Gear_Motor_Enable(0x00000001);
            init->Cyber_Gear_Motor_Speed_Limit(0x00000001, 10.0);
            printf("counter=%d\n", i);
            // ros::Duration(0.5).sleep();
        }
        
        init->Cyber_Gear_Motor_Set_Zero(0x00000001);
        init->Cyber_Gear_Motor_Set_Zero(0x00000002);
        

        
        init->Motor_Pos_Transmit(0x00000143, (0)*100*8, speed);
        VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);
        init->Motor_Pos_Transmit(0x00000144, -(0)*100*8, speed);
        VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);
        init->Motor_Pos_Transmit(0x00000145, -hip_init_angle*100*8, speed);
        VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);
        init->Motor_Pos_Transmit(0x00000146, hip_init_angle*100*8, speed);
        VCI_Receive(4, 0, 0, init_recv_msg, 10, 0);

        // init->Cyber_Gear_Motor_Torque(0x00000001, 0.5);

        
    
        break;
    case 4:
        // init->M_Motor_Set_Zero(0x00000143);
        // init->M_Motor_Pos_Transmit(0x00000143, 0, 80);
        init->Motor_F_Transmit(0x00000142, 80);
        break;
    case 5:
        init->Cyber_Gear_Motor_Select(0x00000002, 0);
        init->Cyber_Gear_Motor_Enable(0x00000002);
        ros::Duration(1).sleep();
        while (1)
        {
            init->Cyber_Gear_Motor_Torque(0x00000002, 0.1);  //调试专用，设置为5nm的扭矩输出
            init->Motor_Feedback();
        }
        
        // init->Cyber_Gear_Motor_Torque(0x00000002, -1);  //调试专用，设置为5nm的扭矩输出
        break;
        // init->Motor_Feedback();
    case 6:
        test.run();
    default:
        break;
    }
    return 0;
}