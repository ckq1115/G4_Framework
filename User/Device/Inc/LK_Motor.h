//
// Created by CaoKangqi on 2026/3/15.
//

#ifndef G4_FRAMEWORK_LK_MOTOR_H
#define G4_FRAMEWORK_LK_MOTOR_H

#include "controller.h"
#include "fdcan.h"

typedef struct
{
    uint8_t  temp;          // 电机温度
    uint16_t Current;       // 电流

    int16_t  rawSpeed;
    int16_t  lastRawSpeed;

    uint16_t rawEncode;
    uint16_t lastRawEncode;

    int32_t round;

    float conEncode;
    float lastConEncode;

    uint8_t State;

}LK_MOTOR_DATA_Typedef;

typedef struct
{
    LK_MOTOR_DATA_Typedef DATA;
    PID_t PID_P;
    PID_t PID_S;
}LK_MOTOR_Typedef;

void LK_Motor_Resolve(void *instance, uint8_t *RxMessage);
void LK_Motor_Iq_Send(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, int16_t iq);
void LK_Motor_Data_Read(FDCAN_HandleTypeDef* hcan, uint16_t motor_id);

#endif //G4_FRAMEWORK_LK_MOTOR_H