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

TD_t TD_Y;
TD_t TD_P;
void Test_Init(void) {
    float PID_P_Y[3] = {5.0f,0.0f,0};
    float PID_S_Y[3] = {100.0f,0.0f,0};

    PID_Init(&All_Motor.DJI_6020_Yaw.PID_P,200,50,
        PID_P_Y,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器
    PID_Init(&All_Motor.DJI_6020_Yaw.PID_S,16000,4000,
        PID_S_Y,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器

    float PID_P_P[3] = {7.0f,0.0f,0.0f};
    float PID_S_P[3] = {100.0f,0.0f,0.0f};

    PID_Init(&All_Motor.DJI_6020_Pitch.PID_P,200,50,
        PID_P_P,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器
    PID_Init(&All_Motor.DJI_6020_Pitch.PID_S,16000,4000,
        PID_S_P,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器
}

void Ctrl_Test_Task(void) {

    All_Motor.DJI_6020_Yaw.PID_P.Ref -= DBUS.Remote.CH2_int16 * 0.0001f;
    if (All_Motor.DJI_6020_Yaw.PID_P.Ref > 90.0f) All_Motor.DJI_6020_Yaw.PID_P.Ref = 90.0f;
    else if (All_Motor.DJI_6020_Yaw.PID_P.Ref < -90.0f) All_Motor.DJI_6020_Yaw.PID_P.Ref = -90.0f;

    All_Motor.DJI_6020_Pitch.PID_P.Ref += DBUS.Remote.CH3_int16 * 0.0001f;
    if (All_Motor.DJI_6020_Pitch.PID_P.Ref > 45.0f) All_Motor.DJI_6020_Pitch.PID_P.Ref = 45.0f;
    else if (All_Motor.DJI_6020_Pitch.PID_P.Ref < -45.0f) All_Motor.DJI_6020_Pitch.PID_P.Ref = -45.0f;

    PID_Calculate(&All_Motor.DJI_6020_Yaw.PID_P,IMU_Data.YawTotalAngle,All_Motor.DJI_6020_Yaw.PID_P.Ref);
    PID_Calculate(&All_Motor.DJI_6020_Yaw.PID_S,IMU_Data.gyro[2],All_Motor.DJI_6020_Yaw.PID_P.Output);

    PID_Calculate(&All_Motor.DJI_6020_Pitch.PID_P,IMU_Data.pitch,All_Motor.DJI_6020_Pitch.PID_P.Ref);
    PID_Calculate(&All_Motor.DJI_6020_Pitch.PID_S,IMU_Data.gyro[1],All_Motor.DJI_6020_Pitch.PID_P.Output);
    DJI_Motor_Send(&hfdcan1,0x1FE,0,-All_Motor.DJI_6020_Pitch.PID_S.Output,All_Motor.DJI_6020_Yaw.PID_S.Output,0);
    //LK_Motor_Iq_Send(&hfdcan2,1,All_Motor.LK9025_Yaw.PID_S.Output);
}