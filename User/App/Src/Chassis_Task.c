//
// Created by CaoKangqi on 2026/2/18.
//
#include "Chassis_Task.h"
#include <stdint.h>
#include "All_Motor.h"
#include "All_define.h"
#include "All_Init.h"
#include "Chassis_Calc.h"

OmniInit_typdef OmniInit_t;
PID_t PID_Chassis_Angle;
model_t chassis_model;
float Ramp_Control(float target,
                          float current,
                          float accel_up,     // 加速度
                          float accel_down,   // 减速度
                          float dt)
{
    float diff = target - current;
    float step;
    if (target * current < 0.0f)
    {
        step = accel_down * dt;
    }
    else if (fabs(target) < fabs(current))
    {
        step = accel_down * dt;
    }
    else
    {
        step = accel_up * dt;
    }
    if (diff > step)
        diff = step;
    else if (diff < -step)
        diff = -step;
    float next = current + diff;
    if ((current > 0 && next < 0) ||
        (current < 0 && next > 0))
    {
        next = 0;
    }
    return next;
}


uint8_t Chassis_Control_Init(MOTOR_Typdef *MOTOR)
{
    OmniInit(&OmniInit_t);
    float PID_S_0[3] = {5.0f,0.01f,0};
    float PID_S_1[3] = {5.0f,0.01f,0};
    float PID_S_2[3] = {5.0f,0.01f,0};
    float PID_S_3[3] = {5.0f,0.01f,0};
    float PID_Angle[3] = {9.0f,0,0};

    PID_Init(&MOTOR->DJI_3508_Chassis[0].PID_S, 100000.0f, 1000.0f,
             PID_S_0, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_3508_Chassis[1].PID_S, 100000.0f, 1000.0f,
             PID_S_1, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_3508_Chassis[2].PID_S, 100000.0f, 1000.0f,
             PID_S_2, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_3508_Chassis[3].PID_S, 100000.0f, 1000.0f,
             PID_S_3, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&PID_Chassis_Angle, 100000.0f, 1000.0f,
             PID_Angle, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    Power_control_init(&chassis_model);
    return DF_READY;
}


void Chassis_Control_Task(MOTOR_Typdef *MOTOR)
{
    float wheel_rpm[4];
    float dt = 0.001f;

    float vx_target = 0;
    float vy_target = 0;
    static float yaw_in = 0;

    if (C_DBUS.Remote.S2_u8 == 1) {
        vx_target = C_DBUS.Remote.CH0_int16 * 3.0f;
        vy_target = C_DBUS.Remote.CH1_int16 * 3.0f;
        yaw_in = -C_DBUS.Remote.CH2_int16 * 0.05f + C_DBUS.Remote.Dial_int16 * 0.15f;

    }
    else if (C_DBUS.Remote.S2_u8 == 3) {
        if (C_DBUS.Remote.S1_u8 == 1)
        {
            float world_vx = 0.0f;
            float world_vy = 0.0f;
            world_vx = C_DBUS.Remote.CH0_int16 * 2.0f;
            world_vy = C_DBUS.Remote.CH1_int16 * 1.2f;
            yaw_in = -18;
            float yaw_rad = IMU_Data.yaw * PI / 180.0f;
            vx_target = world_vx * cosf(yaw_rad) + world_vy * sinf(yaw_rad);
            vy_target = -world_vx * sinf(yaw_rad) + world_vy * cosf(yaw_rad);
        }
    }
    PID_Calculate(&PID_Chassis_Angle,
                  IMU_Data.gyro[2],
                  yaw_in);
    float vw_target = -PID_Chassis_Angle.Output;
    vw_target = -PID_Chassis_Angle.Output;
    static float vx_set = 0;
    static float vy_set = 0;
    static float vw_set = 0;

    vx_set = Ramp_Control(vx_target, vx_set, 5000, 7000, dt);
    vy_set = Ramp_Control(vy_target, vy_set, 5000, 7000, dt);
    vw_set = Ramp_Control(vw_target, vw_set, 500, 10000, dt);

    Omni_calc(wheel_rpm, vx_set, vy_set, vw_set, &OmniInit_t);

    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Calculate(&MOTOR->DJI_3508_Chassis[i].PID_S,
                      MOTOR->DJI_3508_Chassis[i].DATA.Speed_now,
                      wheel_rpm[i]);
    }
    chassis_power_control(&contal,&User_data,&chassis_model,&CAP_Get,&All_Motor);
    if (C_DBUS.DBUS_ONLINE_JUDGE_TIME >= 5) {
        //DJI_Motor_Send(&hfdcan1,0x200,I_cmd[0],I_cmd[1],I_cmd[2],I_cmd[3]);
        DJI_Motor_Send(&hfdcan1,0x200,MOTOR->DJI_3508_Chassis[0].PID_S.Output,
            MOTOR->DJI_3508_Chassis[1].PID_S.Output,
            MOTOR->DJI_3508_Chassis[2].PID_S.Output,
            MOTOR->DJI_3508_Chassis[3].PID_S.Output);
    }
    else {
        DJI_Motor_Send(&hfdcan1,0x200,0,0,0,0);
    }
}