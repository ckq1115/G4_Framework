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
    float PID_F_0[3] = {0,0,0};
    float PID_S_0[3] = {5.0f,0.01f,0};
    float PID_F_1[3] = {0,0,0};
    float PID_S_1[3] = {5.0f,0.01f,0};
    float PID_F_2[3] = {0,0,0};
    float PID_S_2[3] = {5.0f,0.01f,0};
    float PID_F_3[3] = {0,0,0};
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

const float k1 = 1.5756155501e-02f;//kt：M3508鼙鼓的转矩常数Nm/A,对应机械功率项，与转速和电流乘积成正比
const float k2 = 1.1584598349e-01f;//kr：M3508鼙鼓和C620电调的电阻Ω，对应铜损项，与电流平方成正比
const float k3 = 1.9202168378e-05f;//k_iron：M3508电机铁损系数 (W/(rad/s)²)，对应铁损项（磁滞/涡流损耗），与转速平方成正比
const float k4 = 2.157075052e+00f;//k0：M3508电机和C620电调的静态功率W，对应固定损耗项，与转速和电流无关
const float rpm_to_rad = 2.0f * 3.1415926f / 60.0f;//RPM 转 rad/s 的换算常数
float Chassis_Power_Model(MOTOR_Typdef *MOTOR)
{
    float sum_wi = 0.0f;
    float sum_i2 = 0.0f;
    float sum_w2 = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        float w = MOTOR->DJI_3508_Chassis[i].DATA.Speed_now;
        float I = MOTOR->DJI_3508_Chassis[i].DATA.current * 20 / 16384;

        w = w * rpm_to_rad;

        sum_wi += w * I;
        sum_i2 += I * I;
        sum_w2 += w * w;
    }

    float p_dyn = k1 * sum_wi
                + k2 * sum_i2
                + k3 * sum_w2
                + k4 * 4;
    return p_dyn;
}

static void Chassis_Power_Distribute(MOTOR_Typdef *MOTOR, float I_cmd[4], float P_limit)
{
    float A = 0.0f;
    float B = 0.0f;
    float C = 4- P_limit; // 4个电机的静态功耗减去总功率限制

    for(int i = 0; i < 4; i++)
    {
        float w = MOTOR->DJI_3508_Chassis[i].DATA.Speed_now * rpm_to_rad;
        float I = I_cmd[i] * 20/16384;

        A += k2 * I * I;
        B += k1 * w * I;
        C += k3 * w * w;
    }
    // 预测如果不限制电流，总功率是否超标
    float P_predict = A + B + C + P_limit;
    if(P_predict <= P_limit)
    {
        return; // 不超功率，直接返回原电流
    }
    // 解一元二次方程 As^2 + Bs + C = 0 求缩放系数 s
    float s = 1.0f;
    // 如果 A 非常小（比如目标电流全为0），为了防止除以0，直接跳过求解
    if(A < 1e-5f)
    {
        s = 0.0f;
    }
    else
    {
        float discriminant = B * B - 4.0f * A * C;

        if(discriminant >= 0.0f){
            s = (-B + sqrtf(discriminant)) / (2.0f * A);// 选择正根，因为负根会反向放大电流，违背限制的目的
        }
        else{
            s = 0.0f;// 理论上只要 A>0 且 C因超功率而为负，判别式必定大于0，极端异常情况下强制归零保护。
        }
    }
    if(s > 1.0f) s = 1.0f;
    if(s < 0.0f) s = 0.0f;
    // 等比例缩放所有电机的电流
    for(int i = 0; i < 4; i++)
    {
        I_cmd[i] *= s;
    }
}

float m = 0;
float power_radio = 0;
void Chassis_Control_Task(MOTOR_Typdef *MOTOR)
{
    float wheel_rpm[4];
    float dt = 0.001f;

    float vx_target = 0;
    float vy_target = 0;
    static float yaw_in = 0;

    static float Chassis_Target_Yaw = 0.0f;
    static float theta = 0.0f;
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
        else if (C_DBUS.Remote.S1_u8 == 3) {
            vx_target = 0;
            vy_target = 0;
            yaw_in = -18;
        }
        else if (C_DBUS.Remote.S1_u8 == 2) {
            static uint32_t move_timer_3 = 0;
            static int move_dir_3 = 1;
            float world_vx = 0.0f;
            float world_vy = 0.0f;

            yaw_in = -18;

            world_vx = -150.0f * move_dir_3;
            world_vy = 1000.0f * move_dir_3;

            move_timer_3++;
            if (move_timer_3 >= 3800) {
                move_timer_3 = 0;
                move_dir_3 = -move_dir_3;
            }

            float yaw_rad = IMU_Data.yaw * PI / 180.0f;

            vx_target = world_vx * cosf(yaw_rad) + world_vy * sinf(yaw_rad);
            vy_target = -world_vx * sinf(yaw_rad) + world_vy * cosf(yaw_rad);
        }
    }
    PID_Calculate(&PID_Chassis_Angle,
                  IMU_Data.gyro[2],
                  yaw_in);
    float vw_target = -PID_Chassis_Angle.Output;
    float yaw_error = 0 - IMU_Data.yaw * PI / 180.0f;
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

    /*m = Chassis_Power_Model(MOTOR);

    float I_cmd[4];
    for(uint8_t i = 0; i < 4; i++)
    {
        I_cmd[i] = MOTOR->DJI_3508_Chassis[i].PID_S.Output;
    }
    float P_limit = User_data.robot_status.chassis_power_limit;
    Chassis_Power_Distribute(MOTOR, I_cmd, P_limit);
    if (C_DBUS.Remote.S2_u8 == 2) {
        for (int i = 0; i < 4; i++) {
            MOTOR->DJI_3508_Chassis[i].PID_S.Output = 0;
            I_cmd[i] = 0;
        }
    }*/
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