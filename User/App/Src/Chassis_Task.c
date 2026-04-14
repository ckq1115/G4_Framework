//
// Created by CaoKangqi on 2026/2/18.
//
#include "Chassis_Task.h"
#include <stdint.h>
#include "All_Motor.h"
#include "All_define.h"
#include "All_Init.h"
#include "Chassis_Calc.h"

model_t chassis_model;

// 加速度外环PID
PID_t PID_Vx, PID_Vy, PID_Vw;
Steer_Cfg_t S_Cfg;
Steer_State_t S_Now;

/**
 * @brief 底盘控制初始化
 * @param MOTOR 电机总结构体指针
 * @return uint8_t 初始化状态
 */
uint8_t Chassis_Control_Init(MOTOR_Typdef *MOTOR)
{
    Steer_Init(&S_Cfg);

    // 底盘速度外环,输出目标加速度
    float PID_V_Param[3] = {0.0f, 0.0f, 0.0f};
    PID_Init(&PID_Vx, 15.0f, 5.0f, PID_V_Param, 0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    PID_Init(&PID_Vy, 15.0f, 5.0f, PID_V_Param, 0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);

    float PID_Vw_Param[3] = {0.0f, 0.0f, 0.0f};
    PID_Init(&PID_Vw, 20.0f, 8.0f, PID_Vw_Param, 0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);

    float PID_6020_Pos[3] = {0.0f, 0.0f, 0.0f}; // 6020 位置环参数
    float PID_6020_Spd[3] = {0.0f, 0.0f, 0.0f}; // 6020 速度环参数
    float PID_3508_Spd[3] = {0.0f, 0.0f, 0.0f}; // 3508 速度环参数

    for (int i = 0; i < 4; i++)
    {
        // 6020 舵向位置环：输入弧度误差，输出目标转速 (RPM)
        PID_Init(&MOTOR->DJI_6020_Steer[i].PID_P, 300.0f, 100.0f, PID_6020_Pos,
                 0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);

        // 6020 舵向速度环：输入 RPM 误差，输出电流值
        PID_Init(&MOTOR->DJI_6020_Steer[i].PID_S, 16000.0f, 5000.0f, PID_6020_Spd,
                 0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);

        // 3508 驱动速度环：输入 RPM 误差，输出电流值
        PID_Init(&MOTOR->DJI_3508_Chassis[i].PID_S, 16000.0f, 3000.0f, PID_3508_Spd,
                 0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    }

    Power_control_init(&chassis_model);
    return DF_READY;
}

void Chassis_Control_Task(MOTOR_Typdef *MOTOR) {
    // 正解算：使用反馈值更新底盘当前状态
    Steer_Forward_Calc(&S_Now, MOTOR, IMU_Data.gyro[2], &S_Cfg);

    float vx_tar = DBUS.Remote.CH0_int16 * 0.004f;
    float vy_tar = DBUS.Remote.CH1_int16 * 0.004f;
    float vw_tar = -DBUS.Remote.CH2_int16 * 0.01f;

    // 3. 外环 PID 计算 -> 产生加速度需求 (ax, ay, aw)
    PID_Calculate(&PID_Vx, S_Now.vx, vx_tar);
    PID_Calculate(&PID_Vy, S_Now.vy, vy_tar);
    PID_Calculate(&PID_Vw, S_Now.vw, vw_tar);

    // 逆解算：根据加速度和速度目标，计算各轮 Aim 角度、Aim 转速及驱动前馈
    float drive_ff[4];
    Steer_Inverse_Calc(drive_ff, MOTOR, PID_Vx.Output, PID_Vy.Output, PID_Vw.Output,
                       vx_tar, vy_tar, vw_tar, &S_Cfg);

    for (int i = 0; i < 4; i++) {
        // 6020舵向控制
        // 位置环：输入当前角度 (弧度)，目标为遥控器输入的 Aim 角度 -> 输出目标转速 (RPM)
        PID_Calculate(&MOTOR->DJI_6020_Steer[i].PID_P,
                      MOTOR->DJI_6020_Steer[i].DATA.Angle_now * ENCODER_TO_RAD,
                      MOTOR->DJI_6020_Steer[i].DATA.Aim);

        // 速度环：输入当前转速 (RPM)，目标为位置环输出的 Aim 转速 -> 输出电流值
        PID_Calculate(&MOTOR->DJI_6020_Steer[i].PID_S,
                      (float)MOTOR->DJI_6020_Steer[i].DATA.Speed_now,
                      MOTOR->DJI_6020_Steer[i].PID_P.Output);

        // 3508速度环
        // 驱动速度环：输入当前转速 (RPM)，目标为逆解算出的 Aim 转速
        PID_Calculate(&MOTOR->DJI_3508_Chassis[i].PID_S,
                      (float)MOTOR->DJI_3508_Chassis[i].DATA.Speed_now,
                      MOTOR->DJI_3508_Chassis[i].DATA.Aim);

        // 叠加动力学前馈
        MOTOR->DJI_3508_Chassis[i].PID_S.Output += drive_ff[i];
    }

    // 5. 电机控制量发送
    if (DBUS.DBUS_ONLINE_JUDGE_TIME >= 5) {
        // 发送驱动电机 (3508) 电流
        DJI_Motor_Send(&hfdcan1, 0x200,
                       MOTOR->DJI_3508_Chassis[0].PID_S.Output,
                       MOTOR->DJI_3508_Chassis[1].PID_S.Output,
                       MOTOR->DJI_3508_Chassis[2].PID_S.Output,
                       MOTOR->DJI_3508_Chassis[3].PID_S.Output);

        // 发送舵向电机 (6020) 电流 (注意：此处必须发送速度环 PID_S 的输出)
        DJI_Motor_Send(&hfdcan1, 0x1FE,
                       MOTOR->DJI_6020_Steer[0].PID_S.Output,
                       MOTOR->DJI_6020_Steer[1].PID_S.Output,
                       MOTOR->DJI_6020_Steer[2].PID_S.Output,
                       MOTOR->DJI_6020_Steer[3].PID_S.Output);
    }
}

/*OmniInit_typdef OmniInit_t;
PID_t PID_Chassis_Angle;
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
}*/