//
// Created by CaoKangqi on 2026/3/4.
//
#include "Boomerang_Task.h"

#include "All_Init.h"
#include <stdbool.h>
TD_t TD_Pitch;
LESO_t LESO_Pitch;

uint8_t ids[] = {1, 2};
uint16_t angles[]  = {450, 700};
uint16_t angles2[] = {500, 630};
uint16_t angles3[] = {2000, 600};
uint16_t angles4[] = {730, 600};
uint16_t angles5[] = {500, 650};
uint16_t time_ms = 1000;

// 将角度（0~180°）转换为 CCR 值
int Angle_To_CCR(float angle)
{
    float pulse_min = 0.5f;
    float pulse_max = 2.5f;
    float pulse_width = pulse_min + (angle / 180.0f) * (pulse_max - pulse_min);
    return (uint32_t)((pulse_width / 20.0f) * 20000); // ARR+1 = 20000
}

// 舵机角度约束与设置
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t Channel, float angle)
{
    if (angle < 0) angle = 0;
    else if (angle > 270) angle = 270;
    uint32_t ccr = (uint32_t)((angle / 270.0f) * 2000.0f + 500.0f);
    __HAL_TIM_SET_COMPARE(htim, Channel, ccr);
}

typedef struct {
    uint8_t state;       // 整体射击阶段 (0=Idle, 1=第一发, 2=第二发, 3=三发装, 4=三发打, 5=四发)
    uint8_t step;        // 当前阶段的执行步骤
    uint32_t wait_ticks; // 延时计数器 (每5ms递减)
    bool is_running;     // 自动模式使能标志
} AutoShootFSM_t;

AutoShootFSM_t AutoShoot = {0, 0, 0, false};

// 设置延时函数 (单位: 毫秒)
static inline void FSM_SetWait_ms(uint32_t ms) {
    AutoShoot.wait_ticks = ms / 5; // 200Hz任务，1 tick = 5ms
}

// 检查是否在延时中
static inline bool FSM_IsWaiting(void) {
    if (AutoShoot.wait_ticks > 0) {
        AutoShoot.wait_ticks--;
        return true;
    }
    return false;
}

// 核心非阻塞状态机
void Auto_Shoot_Process(void)
{
    if (!AutoShoot.is_running) return;
    if (FSM_IsWaiting()) return; // 如果处于延时状态，直接跳过本次循环

    // 读取限位开关
    uint8_t pin_switch = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);

    switch (AutoShoot.state) {
        case 1: // ========= 第一发打出与第二发装填 =========
            switch (AutoShoot.step) {
                case 0:
                    All_Motor.DJI_3508_Pull.DATA.Aim -= 0.5f; // 寻找限位
                    if (pin_switch == 1) {
                        All_Motor.DJI_3508_Pull.DATA.Aim += (445000.0f * 5.0f); // 拉簧蓄力
                        FSM_SetWait_ms(2000);
                        AutoShoot.step++;
                    }
                    break;
                case 1:
                    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Angle_To_CCR(150));
                    FSM_SetWait_ms(1000);
                    AutoShoot.step++;
                    break;
                case 2:
                    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Angle_To_CCR(90)); // 第一发打出
                    Servo_SetAngle(&htim5, TIM_CHANNEL_2, 102.0f);
                    FSM_SetWait_ms(500);
                    AutoShoot.step++;
                    break;
                case 3:
                    ServoMoveMulti(2, ids, angles, time_ms);
                    FSM_SetWait_ms(1000);
                    AutoShoot.step++;
                    break;
                case 4:
                    ServoMoveMulti(2, ids, angles2, time_ms);
                    FSM_SetWait_ms(1000);
                    AutoShoot.step++;
                    break;
                case 5:
                    ServoMoveMulti(2, ids, angles3, time_ms);
                    FSM_SetWait_ms(1000);
                    AutoShoot.step++;
                    break;
                case 6:
                    Servo_SetAngle(&htim5, TIM_CHANNEL_2, 126.0f);
                    FSM_SetWait_ms(500);
                    AutoShoot.step++;
                    break;
                case 7:
                    ServoMoveMulti(2, ids, angles4, time_ms);
                    // 阶段一结束，直接切换到阶段二
                    AutoShoot.state = 2;
                    AutoShoot.step = 0;
                    break;
            }
            break;

        case 2: // ========= 第二发打出与第三发准备 =========
            switch (AutoShoot.step) {
                case 0:
                    All_Motor.DJI_3508_Pull.DATA.Aim -= 0.5f;
                    if (pin_switch == 1) {
                        All_Motor.DJI_3508_Pull.DATA.Aim += (445000.0f * 5.0f);
                        FSM_SetWait_ms(2000);
                        AutoShoot.step++;
                    }
                    break;
                case 1:
                    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Angle_To_CCR(150));
                    FSM_SetWait_ms(1000);
                    AutoShoot.step++;
                    break;
                case 2:
                    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Angle_To_CCR(90)); // 第二发打出
                    AutoShoot.state = 3;
                    AutoShoot.step = 0;
                    break;
            }
            break;

        case 3: // ========= 第三发装填 =========
            switch (AutoShoot.step) {
                case 0:
                    Servo_SetAngle(&htim5, TIM_CHANNEL_2, 79.0f);
                    FSM_SetWait_ms(500);
                    AutoShoot.step++;
                    break;
                case 1:
                    ServoMoveMulti(2, ids, angles, time_ms);
                    FSM_SetWait_ms(1000);
                    AutoShoot.step++;
                    break;
                case 2:
                    ServoMoveMulti(2, ids, angles2, time_ms);
                    FSM_SetWait_ms(1000);
                    AutoShoot.step++;
                    break;
                case 3:
                    ServoMoveMulti(2, ids, angles3, time_ms);
                    FSM_SetWait_ms(1000);
                    AutoShoot.step++;
                    break;
                case 4:
                    Servo_SetAngle(&htim5, TIM_CHANNEL_2, 126.0f);
                    FSM_SetWait_ms(500);
                    AutoShoot.step++;
                    break;
                case 5:
                    ServoMoveMulti(2, ids, angles4, time_ms);
                    AutoShoot.step++;
                    break;
                case 6:
                    All_Motor.DJI_3508_Pull.DATA.Aim -= 0.5f;
                    if (pin_switch == 1) {
                        All_Motor.DJI_3508_Pull.DATA.Aim += (445000.0f * 5.0f); // 第三发装完
                        AutoShoot.state = 4;
                        AutoShoot.step = 0;
                    }
                    break;
            }
            break;

        case 4: // ========= 第三发打出 =========
            switch (AutoShoot.step) {
                case 0:
                    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Angle_To_CCR(150));
                    FSM_SetWait_ms(1000);
                    AutoShoot.step++;
                    break;
                case 1:
                    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Angle_To_CCR(90)); // 第三发打出
                    FSM_SetWait_ms(500);
                    AutoShoot.state = 5;
                    AutoShoot.step = 0;
                    break;
            }
            break;

        case 5: // ========= 第四发装填与打出 =========
            switch (AutoShoot.step) {
                case 0:
                    Servo_SetAngle(&htim5, TIM_CHANNEL_2, 154.0f);
                    FSM_SetWait_ms(500);
                    AutoShoot.step++;
                    break;
                case 1:
                    ServoMoveMulti(2, ids, angles, time_ms);
                    FSM_SetWait_ms(1000);
                    AutoShoot.step++;
                    break;
                case 2:
                    ServoMoveMulti(2, ids, angles2, time_ms);
                    FSM_SetWait_ms(1000);
                    AutoShoot.step++;
                    break;
                case 3:
                    ServoMoveMulti(2, ids, angles3, time_ms);
                    FSM_SetWait_ms(1000);
                    AutoShoot.step++;
                    break;
                case 4:
                    Servo_SetAngle(&htim5, TIM_CHANNEL_2, 126.0f);
                    FSM_SetWait_ms(500);
                    AutoShoot.step++;
                    break;
                case 5:
                    ServoMoveMulti(2, ids, angles4, time_ms);
                    FSM_SetWait_ms(500);
                    AutoShoot.step++;
                    break;
                case 6:
                    All_Motor.DJI_3508_Pull.DATA.Aim -= 0.5f;
                    if (pin_switch == 1) {
                        All_Motor.DJI_3508_Pull.DATA.Aim += (445000.0f * 5.0f); // 第四发装填完成
                        FSM_SetWait_ms(2000);
                        AutoShoot.step++;
                    }
                    break;
                case 7:
                    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Angle_To_CCR(150));
                    FSM_SetWait_ms(1000);
                    AutoShoot.step++;
                    break;
                case 8:
                    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Angle_To_CCR(90)); // 第四发打出
                    FSM_SetWait_ms(500);
                    // 全部流程结束，恢复 Idle
                    AutoShoot.state = 0;
                    AutoShoot.step = 0;
                    AutoShoot.is_running = false;
                    break;
            }
            break;
    }
}
void Test_Init(void)
{
    static float PID_P_Pull[3] = {0.1f,0.0f,0.0f};
    static float PID_S_Pull[3] = {5.0f,0.0f,2.0f};
    PID_Init(&All_Motor.DJI_3508_Pull.PID_P, 10000.0f, 0.0f,
            PID_P_Pull, 0, 0,
            0, 0, 0,
            Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
                //梯形积分,变速积分
                );//微分先行,微分滤波器
    PID_Init(&All_Motor.DJI_3508_Pull.PID_S, 28000.0f, 0.0f,
            PID_S_Pull, 0, 0,
            0, 0, 0,
            Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
                //梯形积分,变速积分
                );//微分先行,微分滤波器
    static float PID_P_Trigger[3] = {0.1f,0.0f,0.0f};
    static float PID_S_Trigger[3] = {10.0f,0.0f,5.0f};
    PID_Init(&All_Motor.DJI_2006_Trigger.PID_P, 10000.0f, 0.0f,
            PID_P_Trigger, 0, 0,
            0, 0, 0,
            Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
                //梯形积分,变速积分
                );//微分先行,微分滤波器
    PID_Init(&All_Motor.DJI_2006_Trigger.PID_S, 7000.0f, 0.0f,
            PID_S_Trigger, 0, 0,
            0, 0, 0,
            Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
                //梯形积分,变速积分
                );//微分先行,微分滤波器
    static float PID_P_Yaw[3] = {   0.2f,   0.0f,   0.0f   };
    static float PID_S_Yaw[3] = {   2.0f,   0.0f,   1.0f  };
    PID_Init(&All_Motor.DJI_2006_Yaw.PID_P, 10000.0f, 0.0f,
            PID_P_Yaw, 0, 0,
            0, 0, 0,
            Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
                //梯形积分,变速积分
                );//微分先行,微分滤波器
    PID_Init(&All_Motor.DJI_2006_Yaw.PID_S, 7000.0f, 0.0f,
            PID_S_Yaw, 0, 0,
            0, 0, 0,
            Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
                //梯形积分,变速积分
                );//微分先行,微分滤波器
}

void Ctrl_Test_Task(void) {
    if (C_DBUS.Remote.S1_u8 == 2 && AutoShoot.is_running == false) {
        AutoShoot.is_running = true;
        AutoShoot.state = 1;
        AutoShoot.step = 0;
    }

    Auto_Shoot_Process();

    if (!AutoShoot.is_running) {
        All_Motor.DJI_3508_Pull.DATA.Aim += C_DBUS.Remote.CH1_int16 * 5.0f;
        All_Motor.DJI_2006_Trigger.DATA.Aim -= C_DBUS.Remote.CH3_int16 * 3.0f;
        All_Motor.DJI_2006_Yaw.DATA.Aim -= C_DBUS.Remote.CH2_int16 * 3.0f;
    }

    PID_Calculate(&All_Motor.DJI_3508_Pull.PID_P,
                  All_Motor.DJI_3508_Pull.DATA.Angle_Infinite,
                  All_Motor.DJI_3508_Pull.DATA.Aim);
    PID_Calculate(&All_Motor.DJI_3508_Pull.PID_S,
                  All_Motor.DJI_3508_Pull.DATA.Speed_now,
                  All_Motor.DJI_3508_Pull.PID_P.Output);

    PID_Calculate(&All_Motor.DJI_2006_Trigger.PID_P,
                  All_Motor.DJI_2006_Trigger.DATA.Angle_Infinite,
                  All_Motor.DJI_2006_Trigger.DATA.Aim);
    PID_Calculate(&All_Motor.DJI_2006_Trigger.PID_S,
                  All_Motor.DJI_2006_Trigger.DATA.Speed_now,
                  All_Motor.DJI_2006_Trigger.PID_P.Output);

    PID_Calculate(&All_Motor.DJI_2006_Yaw.PID_P,
                  All_Motor.DJI_2006_Yaw.DATA.Angle_Infinite,
                  All_Motor.DJI_2006_Yaw.DATA.Aim);
    PID_Calculate(&All_Motor.DJI_2006_Yaw.PID_S,
                  All_Motor.DJI_2006_Yaw.DATA.Speed_now,
                  All_Motor.DJI_2006_Yaw.PID_P.Output);

    //DJI_Motor_Send(&hfdcan3, 0x200, All_Motor.DJI_3508_Pull.PID_S.Output, 0, 0, 0);
}
