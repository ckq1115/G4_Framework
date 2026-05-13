//
// Created by CaoKangqi on 2026/2/14.
//

#ifndef G4_FRAMEWORK_DJI_MOTOR_H
#define G4_FRAMEWORK_DJI_MOTOR_H

#include "BSP-FDCAN.h"
#include "CKQ_MATH.h"
#include "controller.h"

typedef struct {
    int8_t ONLINE_JUDGE_TIME;
    int16_t Angle_last;
    int16_t Angle_now;
    int32_t round;
    int32_t conEncode;
    int16_t Speed_last;
    int16_t Speed_now;
    int16_t acceleration;
    int16_t current;
    int8_t temperature;
    int32_t Angle_Infinite;
    int64_t Stuck_Time;
    int16_t Recovery_Count;
    uint16_t Stuck_Flag[2];
    int16_t Laps;
    float dt;
} DJI_MOTOR_DATA_Typedef;

typedef struct {
    uint8_t PID_INIT;
    DJI_MOTOR_DATA_Typedef DATA;
    Feedforward_t PID_F;
    PID_t PID_P;
    PID_t PID_S;
} DJI_MOTOR_Typedef;

void DJI_Motor_Dispatch(FDCAN_HandleTypeDef *hfdcan, uint32_t FIFO_x);
void DJI_Motor_Resolve(void* instance, uint8_t* rx_data);
void DJI_Motor_Send(FDCAN_HandleTypeDef* hcan, uint32_t stdid, int16_t n1, int16_t n2, int16_t n3, int16_t n4);
void DJI_Motor_Clear(DJI_MOTOR_Typedef* motor);
void DJI_Motor_Stuck_Check(DJI_MOTOR_Typedef* motor, float current_limit, float speed_limit, uint16_t time_limit, uint16_t recovery_limit);

#endif //G4_FRAMEWORK_DJI_MOTOR_H