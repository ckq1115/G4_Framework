//
// Created by CaoKangqi on 2026/2/27.
//
#include "Test_Task.h"

#include "All_Init.h"
#include "controller.h"

ShootDet_t g_det = {0};

//500Hz调用,传入两个摩擦轮的转速
bool Update_Shoot_Det(float speed1, float speed2, ShootDet_t *det) {
    float val = (fabsf(speed1) + fabsf(speed2)) / 2.0f;
    if (!det->init) {
        det->base = val;
        det->last_val = val;
        det->max_drop_in_round = 0;
        det->cool_down_cnt = 0;
        det->init = true;
        return false;
    }
    float slope = det->last_val - val;
    det->last_val = val;
    if (val > det->base) {
        det->base = (K_UP * val) + (1.0f - K_UP) * det->base;
    } else {
        det->base = (K_DN * val) + (1.0f - K_DN) * det->base;
    }
    float drop = det->base - val;
    bool shoot_done = false;
    if (det->cool_down_cnt > 0) {
        det->cool_down_cnt--;
        det->armed = false;
        return false;
    }
    if (!det->armed) {
        if (drop > TH_FIRE && drop < TH_FIRE_MAX && slope > MIN_SLOPE && val > 4500) {
            det->armed = true;
            det->max_drop_in_round = drop;
            det->t_out = 0;
        }
    } else {
        det->t_out++;
        if (drop > det->max_drop_in_round) {
            det->max_drop_in_round = drop;
        }
        bool condition_relative = (drop < det->max_drop_in_round * (1.0f - RELATIVE_RECOVER));
        bool condition_absolute = (drop < TH_RST_SAFE);
        if (condition_relative || condition_absolute) {
            det->armed = false;
            det->cnt++;
            det->cool_down_cnt = COOL_DOWN_TICKS;
            det->max_drop_in_round = 0;
            shoot_done = true;
        }
        else if (det->t_out >= TIMEOUT_TICKS) {
            det->armed = false;
            det->max_drop_in_round = 0;
        }
    }
    return shoot_done;
}

void Test_Init(void) {
    /*float PID_P_S1[3] = {0.6f,0.0f,0.0f};
    float PID_S_S1[3] = {0.3f,0.002f,0.0f};*/
    float PID_P_S1[3] = {1.36f,0.0f,0.0f};
    float PID_S_S1[3] = {3.0f,0.002f,0.8f};

    float PID_S_SHOOT[3] = {4.0f,0.0f,0.0f};
    PID_Init(&All_Motor.DM4310_Yaw.PID_P,5000,500,
        PID_P_S1,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器
    PID_Init(&All_Motor.DM4310_Yaw.PID_S,100,50,
        PID_S_S1,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器
    PID_Init(&All_Motor.DJI_2006_bo.PID_S,16000,4000,
        PID_S_SHOOT,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器
}

void Ctrl_Test_Task(void) {
    static float target_omega = 0;
    //All_Motor.DM4310_Yaw.PID_P.Ref = 50*cosf(2.0*3.14*1*t);
    if (IMU_Data.YawTotalAngle != 0.0f) {
        target_omega = DBUS.Remote.CH2_int16 * 0.3f + DBUS.KeyBoard.E * 5.0f - DBUS.KeyBoard.Q * 5.0f + DBUS.Mouse.X_Flt * 2.0f;
        All_Motor.DM4310_Yaw.PID_P.Ref += target_omega*0.001f;
    }

    #define VFF_GAIN  0.05f
    float speed_ff = VFF_GAIN * target_omega;

    /*PID_Calculate(&All_Motor.DM4310_Yaw.PID_P,All_Motor.DM4310_Yaw.DATA.Angle_Infinite,All_Motor.DM4310_Yaw.PID_P.Ref);
    PID_Calculate(&All_Motor.DM4310_Yaw.PID_S,All_Motor.DM4310_Yaw.DATA.Speed_now,All_Motor.DM4310_Yaw.PID_P.Output);*/
    PID_Calculate(&All_Motor.DM4310_Yaw.PID_P,IMU_Data.YawTotalAngle,All_Motor.DM4310_Yaw.PID_P.Ref);
    PID_Calculate(&All_Motor.DM4310_Yaw.PID_S,IMU_Data.gyro[2],All_Motor.DM4310_Yaw.PID_P.Output);

    DM_Motor_Send(&hfdcan2,0x3FE,- (All_Motor.DM4310_Yaw.PID_S.Output + speed_ff),0,0,0);
    //DM_Motor_Send(&hfdcan2,0x3FE,All_Motor.DM4310_Yaw.PID_P.Ref,0,0,0);

    //PID_Calculate(&All_Motor.DJI_2006_bo.PID_S,All_Motor.DJI_2006_bo.DATA.Speed_now,All_Motor.DJI_2006_bo.PID_S.Ref);
    //DJI_Motor_Send(&hfdcan2,0x200,All_Motor.DJI_2006_bo.PID_S.Output,0,0,0);
}

Tx_t Tx_Data = {0};
Rx_t Rx_Data = {0};
//自定义长度CAN通讯测试，用于双板通讯
void Test_Tx(void) {
    Tx_Data.ch0 = DBUS.Remote.CH0_int16;
    Tx_Data.ch1 = DBUS.Remote.CH1_int16;
    Tx_Data.ch2 = DBUS.Remote.CH2_int16;
    Tx_Data.ch3 = DBUS.Remote.CH3_int16;
    Tx_Data.s1 = DBUS.Remote.S1_u8;
    Tx_Data.s2 = DBUS.Remote.S2_u8;
    Tx_Data.pitch = IMU_Data.pitch;
    Tx_Data.roll = IMU_Data.roll;
    Tx_Data.yaw = IMU_Data.yaw;
    Tx_Data.accel[0] = IMU_Data.accel[0];
    Tx_Data.accel[1] = IMU_Data.accel[1];
    Tx_Data.accel[2] = IMU_Data.accel[2];
    CAN_TP_Send_Struct(&hfdcan1, &Tx_Data, sizeof(Tx_t));
}

void CAN_TP_On_Struct_Received(uint8_t *data_ptr, uint16_t len)
{
    memcpy(&Rx_Data, data_ptr, sizeof(Rx_t));
}