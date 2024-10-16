#include "controller.h"

void VMC::LQR_K(Leg_Model *leg) // 计算相应腿长下对应的K
{
    double t1, t2, t3;
    t1 = leg->L0;
    t2 = pow(t1, 2);
    t3 = pow(t1, 3);
    leg->K[0][0] = t1 * lqr_index[0][2] + t2 * lqr_index[0][1] + t3 * lqr_index[0][0] + lqr_index[0][3];
    leg->K[1][0] = t1 * lqr_index[1][2] + t2 * lqr_index[1][1] + t3 * lqr_index[1][0] + lqr_index[1][3];
    leg->K[0][1] = t1 * lqr_index[2][2] + t2 * lqr_index[2][1] + t3 * lqr_index[2][0] + lqr_index[2][3];
    leg->K[1][1] = t1 * lqr_index[3][2] + t2 * lqr_index[3][1] + t3 * lqr_index[3][0] + lqr_index[3][3];
    leg->K[0][2] = t1 * lqr_index[4][2] + t2 * lqr_index[4][1] + t3 * lqr_index[4][0] + lqr_index[4][3];
    leg->K[1][2] = t1 * lqr_index[5][2] + t2 * lqr_index[5][1] + t3 * lqr_index[5][0] + lqr_index[5][3];
    leg->K[0][3] = t1 * lqr_index[6][2] + t2 * lqr_index[6][1] + t3 * lqr_index[6][0] + lqr_index[6][3];
    leg->K[1][3] = t1 * lqr_index[7][2] + t2 * lqr_index[7][1] + t3 * lqr_index[7][0] + lqr_index[7][3];
    leg->K[0][4] = t1 * lqr_index[8][2] + t2 * lqr_index[8][1] + t3 * lqr_index[8][0] + lqr_index[8][3];
    leg->K[1][4] = t1 * lqr_index[9][2] + t2 * lqr_index[9][1] + t3 * lqr_index[9][0] + lqr_index[9][3];
    leg->K[0][5] = t1 * lqr_index[10][2] + t2 * lqr_index[10][1] + t3 * lqr_index[10][0] + lqr_index[10][3];
    leg->K[1][5] = t1 * lqr_index[11][2] + t2 * lqr_index[11][1] + t3 * lqr_index[11][0] + lqr_index[11][3];
}

void VMC::T_Calc(Leg_Model *leg) // 轮扭矩 u = K(xd-x)
{
    double t[6] = {0};
    double T_output;
    t[0] = leg->K[0][0] * (leg->theta_exp - leg->theta);
    t[1] = leg->K[0][1] * (leg->dtheta_exp - leg->dtheta);
    t[2] = leg->K[0][2] * (leg->x_exp - leg->x);
    t[3] = leg->K[0][3] * (leg->dx_exp - leg->dx);
    t[4] = leg->K[0][4] * (leg->phi_exp - leg->phi);
    t[5] = leg->K[0][5] * (leg->dphi_exp - leg->dphi);
    T_output = t[0] + t[1] + t[2] + t[3] + t[4] + t[5];
    // printf("轮毂电机的反馈矩阵为\n%.4f+%.4f+%.4f+%.4f+%.4f+%.4f\n", leg->K[0][0], leg->K[0][1], leg->K[0][2], leg->K[0][3], leg->K[0][4], leg->K[0][5]);
    // printf("轮毂电机的输出力矩为\n%.4f+%.4f+%.4f+%.4f+%.4f+%.4f=%.4f\n", t[0], t[1], t[2], t[3], t[4], t[5], T_output);
    if (leg->isRobotOffGround == true)
    {
        T_output = 0;
    }
    else
    {
        T_output = t[0] + t[1] + t[2] + t[3] + t[4] + t[5];
    }
    leg->T = T_output;
}

void VMC::Tp_Calc(Leg_Model *leg) // 髋扭矩 u = K(xd-x)
{
    double t[6] = {0};
    double Tp_output;
    t[0] = leg->K[1][0] * (leg->theta_exp - leg->theta);
    t[1] = leg->K[1][1] * (leg->dtheta_exp - leg->dtheta);
    t[2] = leg->K[1][2] * (leg->x_exp - leg->x);
    t[3] = leg->K[1][3] * (leg->dx_exp - leg->dx);
    t[4] = leg->K[1][4] * (leg->phi_exp - leg->phi);
    t[5] = leg->K[1][5] * (leg->dphi_exp - leg->dphi);
    if (leg->isRobotOffGround == true)
    {
        Tp_output = t[0] + t[1];
    }
    else
    {
        Tp_output = t[0] + t[1] + t[2] + t[3] + t[4] + t[5];
    }
    // printf("关节电机的输出力矩为%f+%f+%f+%f+%f+%f=%f\n", t[0], t[1], t[2], t[3], t[4], t[5], Tp_output);
    leg->Tp = -Tp_output;
}