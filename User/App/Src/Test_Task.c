/*//
// Created by CaoKangqi on 2026/2/27.
//
#include "Test_Task.h"

#include "All_Init.h"
#include "controller.h"

void Test_Init(void) {
    float PID_P_Y[3] = {0,0,0};
    float PID_S_Y[3] = {0,0,0};

    PID_Init(&All_Motor.LK9025_Yaw.PID_P,10000,1000,
        PID_P_Y,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器
    PID_Init(&All_Motor.LK9025_Yaw.PID_P,10000,1000,
        PID_S_Y,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器
}


void Ctrl_Test_Task(void) {
    PID_Calculate(&All_Motor.LK9025_Yaw.PID_P,IMU_Data.YawTotalAngle,C_DBUS.Remote.CH3_int16);
    PID_Calculate(&All_Motor.LK9025_Yaw.PID_S,All_Motor.LK9025_Yaw.DATA.rawSpeed,All_Motor.LK9025_Yaw.PID_P.Output);

    LK_Motor_Iq_Send(&hfdcan2,1,All_Motor.LK9025_Yaw.PID_S.Output);
}

/**
 * @brief 双板通信发送
 #1#
uint8_t Board_Send(FDCAN_HandleTypeDef *hfdcan,uint32_t id,int16_t d1,int16_t d2,int16_t d3,int16_t d4)
{
    uint8_t data[8];

    data[0] = d1 >> 8;
    data[1] = d1;

    data[2] = d2 >> 8;
    data[3] = d2;

    data[4] = d3 >> 8;
    data[5] = d3;

    data[6] = d4 >> 8;
    data[7] = d4;

    return FDCAN_Send_Msg(hfdcan, id, data, 8);
}

/**
 * @brief 双板通信解算
 #1#
void Board_Resolve(uint8_t *rxdata,int16_t *d1,int16_t *d2,int16_t *d3,int16_t *d4)
{
    *d1 = (int16_t)((rxdata[0] << 8) | rxdata[1]);
    *d2 = (int16_t)((rxdata[2] << 8) | rxdata[3]);
    *d3 = (int16_t)((rxdata[4] << 8) | rxdata[5]);
    *d4 = (int16_t)((rxdata[6] << 8) | rxdata[7]);
}*/