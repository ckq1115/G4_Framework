//
// Created by CaoKangqi on 2026/2/14.
//
#include "System_Status.h"

#include <stdbool.h>

#include "All_define.h"
#include "All_Init.h"
#include "tim.h"
#include "TIM_PWM.h"

void System_Root(ROOT_STATUS_Typedef *Root, DBUS_Typedef *DBUS, MOTOR_Typdef *MOTOR, Cap_t *CAP_GET)
{
    All_Status(Root, DBUS, MOTOR, CAP_GET);
    static uint32_t led_tick = 0;
    if (led_tick++ % 20 == 0) {

    }
    LED_Show_Status(Root);
}

/**
 * @brief 大疆电机在线状态监测函数
 * @param DATA 电机数据结构体指针，包含在线计时器
 * @return DEVICE_ONLINE 或 DEVICE_OFFLINE
 * @note 该函数通过递减在线计时器来判断电机是否在线，若计时器小于5则认为离线，并重置计时器为0
 */
uint8_t DJI_MOTOR_STATUS(DJI_MOTOR_DATA_Typedef* DATA)
{
    DATA->ONLINE_JUDGE_TIME--;

    if (DATA->ONLINE_JUDGE_TIME < 5)
    {
        DATA->ONLINE_JUDGE_TIME = 0;
        return DEVICE_OFFLINE;
    }
    else
        return DEVICE_ONLINE;
}

/**
 * @brief 达妙电机在线状态监测函数
 * @param DATA 电机数据结构体指针，包含在线计时器
 * @return DEVICE_ONLINE 或 DEVICE_OFFLINE
 * @note 该函数通过递减在线计时器来判断电机是否在线，若计时器小于5则认为离线，并重置计时器为0
 */
uint8_t DM_MOTOR_STATUS(DM_MOTOR_DATA_Typdef* DATA)
{
    DATA->ONLINE_JUDGE_TIME--;

    if (DATA->ONLINE_JUDGE_TIME < 5)
    {
        DATA->ONLINE_JUDGE_TIME = 0;
        return DEVICE_OFFLINE;
    }
    return DEVICE_ONLINE;
}

void All_Status(ROOT_STATUS_Typedef *Root, DBUS_Typedef *DBUS, MOTOR_Typdef *MOTOR, Cap_t *CAP_GET)
{
    if (DBUS->ONLINE_JUDGE_TIME < 5)
    {
        DBUS->ONLINE_JUDGE_TIME = 3;
        Root->RM_DBUS = DEVICE_OFFLINE;
    }
    else
    {
        Root->RM_DBUS = DEVICE_ONLINE;
    }
    DBUS->ONLINE_JUDGE_TIME--;

    //电容在线监测
    if(CAP_GET->get.ONLINE_JUDGE_TIME < 5)
    {
        CAP_GET->get.ONLINE_JUDGE_TIME = 3;
        Root->Cap = DEVICE_OFFLINE;
    }
    else
    {
        Root->Cap = DEVICE_ONLINE;
    }
    CAP_GET->get.ONLINE_JUDGE_TIME--;
    Root->MOTOR_Chassis_1 = DJI_MOTOR_STATUS(&All_Motor.DJI_3508_Chassis[0].DATA);
    Root->MOTOR_Chassis_2 = DJI_MOTOR_STATUS(&All_Motor.DJI_3508_Chassis[1].DATA);
    Root->MOTOR_Chassis_3 = DJI_MOTOR_STATUS(&All_Motor.DJI_3508_Chassis[2].DATA);
    Root->MOTOR_Chassis_4 = DJI_MOTOR_STATUS(&All_Motor.DJI_3508_Chassis[3].DATA);
    //Root->MOTOR_HEAD_Pitch = DM_MOTOR_STATUS(&All_Motor.DM4310_Pitch.DATA);
    //Root->MOTOR_HEAD_Yaw = DM_MOTOR_STATUS(&All_Motor.DM4310_Yaw.DATA);
}

static const Status_t Display_Map[] = {
    [DEVICE_ONLINE]  = { .r = 0,   .g = 60, .b = 0, .breathe = 2.0f, .buzzer = false },
    [DEVICE_OFFLINE] = { .r = 150, .g = 0, .b = 0, .breathe = 0.5f, .buzzer = true  }
};

void LED_Show_Status(ROOT_STATUS_Typedef *Root)
{
    const Status_t *dbus  = &Display_Map[Root->RM_DBUS];
    const Status_t *motor = &Display_Map[Root->MOTOR_Chassis_1&&Root->MOTOR_Chassis_2&&Root->MOTOR_Chassis_3&&Root->MOTOR_Chassis_4];

    WS2812_SetPixel(2, motor->r, motor->g, motor->b);
    WS2812_UpdateBreathing(2, motor->breathe);
    WS2812_SetPixel(3, dbus->r, dbus->g, dbus->b);
    WS2812_UpdateBreathing(3, dbus->breathe);

    (imu_ctrl_state == ERROR_STATE) ? WS2812_UpdateBreathing(0, 0.2f) : WS2812_UpdateBreathing(0, 2.0f);
    /*if (motor->buzzer && dbus->buzzer) {
        Buzzer_UpdateCycle(0.25f, 0.5f, 20);
    }
    else if (dbus->buzzer || motor->buzzer) {
        Buzzer_UpdateCycle(0.5f, 1.0f, 20);
    }
    else {
        Buzzer_Stop();
    }*/
    WS2812_Send();
}