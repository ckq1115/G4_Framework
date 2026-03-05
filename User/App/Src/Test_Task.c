//
// Created by CaoKangqi on 2026/2/27.
//
#include "Test_Task.h"

#include "All_Init.h"

TD_t TD_Pitch;
LESO_t LESO_Pitch;

/**
 * @brief 位置环 PID 钩子函数：集成 TD 平滑与速度前馈
 * @note  挂载在 PID_P 的 User_Func1_f 指针上
 */
void Pitch_Pos_TD_Hook(PID_t *pid)
{
    TD_Calculate(&TD_Pitch, pid->Ref);
    pid->Ref = TD_Pitch.x;

    pid->Err = pid->Ref - pid->Measure;
}
void Test_Init(void)
{
    float PID_P_0[3] = {1.0f,0.005f,0};
    float PID_S_0[3] = {12.0f,0.0f,0};
    PID_Init(&All_Motor.DJI_6020_Pitch.PID_P, 300.0f, 100.0f,
             PID_P_0, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    All_Motor.DJI_6020_Pitch.PID_P.User_Func1_f = Pitch_Pos_TD_Hook;
    PID_Init(&All_Motor.DJI_6020_Pitch.PID_S, 16384.0f, 1000.0f,
             PID_S_0, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    TD_Init(&TD_Pitch, 38000, 0.005f);
    LESO_Init(&LESO_Pitch,7.0f,600.0f);
}
float Target = 0;

float Sine_Amplitude = 2000.0f;  // 振幅 (根据你的单位调整，例如45度)
float Sine_Frequency = 2.0f;   // 频率 (单位：Hz，即每秒钟往返一次)
float Sine_Offset = 0.0f;      // 中心偏移量
float Sine_Time = 0.0f;        // 时间累积变量
float Control_Freq = 1000.0f;  // 任务执行频率 (如果是1ms运行一次，就是1000Hz)
float ldob_comp = 0;
void Ctrl_Test_Task(void)
{
    float Pitch_raw = C_DBUS.Remote.CH2_int16;
    static float Pitch_Target = 0;
    Pitch_Target += Pitch_raw * 0.01f;

    /*Target = Sine_Amplitude * sinf(2.0f * 3.1415926f * Sine_Frequency * Sine_Time) + Sine_Offset;
    Sine_Time += (1.0f / Control_Freq);*/
    if(Sine_Time > 100.0f) Sine_Time = 0.0f;
    PID_Calculate(&All_Motor.DJI_6020_Pitch.PID_P,
                  All_Motor.DJI_6020_Pitch.DATA.Angle_Infinite,
                  Target);
    PID_Calculate(&All_Motor.DJI_6020_Pitch.PID_S,
                  All_Motor.DJI_6020_Pitch.DATA.Speed_now,
                  All_Motor.DJI_6020_Pitch.PID_P.Output);

    float disturb_est = LESO_Calculate(&LESO_Pitch,
                                   All_Motor.DJI_6020_Pitch.DATA.Speed_now,
                                   All_Motor.DJI_6020_Pitch.PID_S.Output);

    static float Final_Output = 0;
    Final_Output = All_Motor.DJI_6020_Pitch.PID_S.Output + disturb_est;
    DJI_Motor_Send(&hfdcan3,0x1FE,
               Final_Output,
               0,
               Final_Output,
               0);
    /*DJI_Motor_Send(&hfdcan3,0x1FE,All_Motor.DJI_6020_Pitch.PID_S.Output,
                        0,All_Motor.DJI_6020_Pitch.PID_S.Output, 0);*/

}
