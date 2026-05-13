//
// Created by CaoKangqi on 2026/2/14.
//
#include "DJI_Motor.h"
#include "All_define.h"
#include "TIM_PWM.h"

/**
 * @brief DJI 电机协议解析内核 (3508/2006/6020 通用)
 */
void DJI_Motor_Resolve(void* instance, uint8_t* rx_data) {
    DJI_MOTOR_DATA_Typedef* DATA = &((DJI_MOTOR_Typedef*)instance)->DATA;

    DATA->Angle_last = DATA->Angle_now;
    DATA->Angle_now  = (int16_t)((rx_data[0] << 8) | rx_data[1]);
    DATA->Speed_last = DATA->Speed_now;
    int16_t spd_raw = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    DATA->Speed_now  = OneFilter1(spd_raw,DATA->Speed_last, 25000);
    DATA->current    = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    DATA->temperature = rx_data[6]; // 6020/3508有温度，2006不看即可

    // 统一处理越界/圈数逻辑
    int16_t diff = DATA->Angle_now - DATA->Angle_last;
    if      (diff < -4000) DATA->Laps++;
    else if (diff >  4000) DATA->Laps--;

    // 圈数异常保护
    if (DATA->Laps > 32500 || DATA->Laps < -32500) {
        DATA->Laps = 0;
    }

    DATA->ONLINE_JUDGE_TIME = MOTOR_OFFLINE_TIME;
    DATA->Angle_Infinite = (int32_t)((DATA->Laps << 13) + DATA->Angle_now);
}

/**
 * @brief 通用发送函数
 * @param hcan
 */
void DJI_Motor_Send(FDCAN_HandleTypeDef* hcan, uint32_t stdid, int16_t n1, int16_t n2, int16_t n3, int16_t n4) {
    uint8_t data[8];
    data[0] = n1 >> 8; data[1] = n1;
    data[2] = n2 >> 8; data[3] = n2;
    data[4] = n3 >> 8; data[5] = n3;
    data[6] = n4 >> 8; data[7] = n4;
    FDCAN_Send_Msg(hcan, stdid, data, 8);
}


void DJI_Motor_Clear(DJI_MOTOR_Typedef* motor) {
    motor->PID_S.Output = 0.0;
    motor->PID_S.Iout = 0.0;
}
/**
 * @brief 电机堵转检测与自动恢复
 * @param motor 电机结构体
 * @param current_limit 触发堵转的最小电流阈值
 * @param speed_limit   判定为停止的最高速度阈值
 * @param time_limit    判断堵转的持续时间 (ms)
 * @param recovery_limit 堵转后停止运行的持续时间 (ms)
 */
void DJI_Motor_Stuck_Check(DJI_MOTOR_Typedef* motor, float current_limit, float speed_limit, uint16_t time_limit, uint16_t recovery_limit) {

    if (motor->DATA.Recovery_Count > 0) {
        motor->DATA.Recovery_Count--;

        DJI_Motor_Clear(motor);

        if (motor->DATA.Recovery_Count == 0) {
            Buzzer_Stop(); // 停止蜂鸣器报警
        }
        return; // 恢复期内不进行常规堵转检测
    }

    // --- 逻辑 2：常规堵转检测 ---
    float current_feedback = motor->DATA.current;

    if (MATH_ABS_float(current_feedback) > current_limit && MATH_ABS_float(motor->DATA.Speed_now) < speed_limit) {
        motor->DATA.Stuck_Time++;

        if (motor->DATA.Stuck_Time > time_limit) {
            // 【触发保护】
            motor->DATA.Stuck_Time = 0;
            motor->DATA.Stuck_Flag[0]++;

            // 进入锁定恢复模式
            motor->DATA.Recovery_Count = recovery_limit;

            //DJI_Motor_Clear(motor);
            motor->DATA.Stuck_Flag[1] = 1;
            // 报警提示
            Buzzer_Run();
        }
    } else {
        // 如果电机还在动，重置检测计数
        motor->DATA.Stuck_Time = 0;
    }
}