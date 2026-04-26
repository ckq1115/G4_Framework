//
// Created by CaoKangqi on 2026/2/23.
//

#include "Power_Ctrl.h"
#include <stdint.h>
#include "DJI_Motor.h"
#include "All_Define.h"
#include <math.h>

void Power_control_init(model_t *model) {
    model->Kp = 3.0f;
    model->Remaining_Buffer = 25.0f;
    model->rpm_to_rad = 2.0f * 3.14159265f / 60.0f;

    // M3508 模型参数
    model->m3508.k1 = 1.5756e-02f;
    model->m3508.k2 = 1.94e-01f;
    model->m3508.k3 = 1.9202e-05f;
    model->m3508.k4 = 1.15f;
    model->m3508.current_convert = 20.0f / 16384.0f;

    model->m6020.k1 = 0.751f;  // 6020转矩常数项
    model->m6020.k2 = 2.5f;  // 6020内阻项
    model->m6020.k3 = 2.1e-5f;// 6020铁损项4444
    model->m6020.k4 = 1.15f;  // 6020静态功率
    model->m6020.current_convert = 3.0f / 16384.0f;
}

// 功率预测函数
float get_motor_power(DJI_MOTOR_Typedef *motor, motor_model_t *m_params, float rpm_to_rad) {
    float w = motor->DATA.Speed_now * rpm_to_rad;
    float I = motor->PID_S.Output * m_params->current_convert;
    float p = m_params->k1 * w * I + m_params->k2 * I * I + m_params->k3 * w * w + m_params->k4;
    return p;
}

void solve_motor_group(DJI_MOTOR_Typedef *motors[4], float I_cmd[4], float P_limit, motor_model_t *m_params, float rpm_to_rad) {
    float A = 0, B = 0, C = 4 * m_params->k4 - P_limit;

    for (int i = 0; i < 4; i++) {
        float w = motors[i]->DATA.Speed_now * rpm_to_rad;
        float I = I_cmd[i] * m_params->current_convert;
        A += m_params->k2 * I * I;
        B += m_params->k1 * w * I;
        C += m_params->k3 * w * w;
    }

    if (A + B + C + P_limit <= P_limit) return; // 未超功率

    float s = 1.0f;
    if (A > 1e-6f) {
        float delta = B * B - 4.0f * A * C;
        if (delta >= 0) s = (-B + sqrtf(delta)) / (2.0f * A);
        else s = 0.0f;
    } else if (B > 1e-6f) {
        s = (P_limit - (C + P_limit - B)) / B;
    }

    s = (s > 1.0f) ? 1.0f : ((s < 0) ? 0 : s);
    for (int i = 0; i < 4; i++) I_cmd[i] *= s;
}

float pall = 0;
uint8_t chassis_power_control(CONTAL_Typedef *RUI_V_CONTAL_V, User_Data_T *usr_data,
                              model_t *model, CAP_RXDATA *CAP_GET, MOTOR_Typdef *MOTOR)
{
    pall = 0;
    const uint16_t SuperMaxPower = 200;
    float chassis_max_power = (usr_data->robot_status.chassis_power_limit != 0) ?
                               usr_data->robot_status.chassis_power_limit : 50.0f - model->Kp*(model->Remaining_Buffer - All_Power.P_Chassis.buffer_energy);

    if (RUI_V_CONTAL_V->BOTTOM.CAP != 0) {
        chassis_max_power += (float)SuperMaxPower;
    }

    float p3508_pred = 0, p6020_pred = 0;
    float I_cmd_3508[4], I_cmd_6020[4];
    DJI_MOTOR_Typedef *ptr_3508[4], *ptr_6020[4];

    for (int i = 0; i < 4; i++) {
        ptr_3508[i] = &MOTOR->DJI_3508_Chassis[i];
        ptr_6020[i] = &MOTOR->DJI_6020_Steer[i];

        I_cmd_3508[i] = ptr_3508[i]->PID_S.Output;
        I_cmd_6020[i] = ptr_6020[i]->PID_S.Output;

        p3508_pred += get_motor_power(ptr_3508[i], &model->m3508, model->rpm_to_rad);
        p6020_pred += get_motor_power(ptr_6020[i], &model->m6020, model->rpm_to_rad);
    }
    if (p3508_pred < 0) p3508_pred = 0;
    if (p6020_pred < 0) p6020_pred = 0;

    float total_pred = p3508_pred + p6020_pred;

    if (total_pred > chassis_max_power) {
        /*float ratio_3508 = p3508_pred / total_pred;
        float ratio_6020 = p6020_pred / total_pred;*/

        float limit_3508 = chassis_max_power -p6020_pred;
        if (limit_3508 < 0) limit_3508 = 0;
        //float limit_6020 = chassis_max_power * ratio_6020;
        solve_motor_group(ptr_3508, I_cmd_3508, limit_3508, &model->m3508, model->rpm_to_rad);
        if (p6020_pred > chassis_max_power) solve_motor_group(ptr_6020, I_cmd_6020, chassis_max_power, &model->m6020, model->rpm_to_rad);
    }

    for (int i = 0; i < 4; i++) {
        MOTOR->DJI_3508_Chassis[i].PID_S.Output = I_cmd_3508[i];
        MOTOR->DJI_6020_Steer[i].PID_S.Output = I_cmd_6020[i];
        pall += get_motor_power(ptr_3508[i], &model->m3508, model->rpm_to_rad)+get_motor_power(ptr_6020[i], &model->m6020, model->rpm_to_rad);
    }

    return DF_READY;
}

//功率计接收解算函数
void CAN_POWER_Rx(Power_Typedef* Power, uint8_t *rx_data)
{
    int16_t raw_shunt = (int16_t)((int16_t)rx_data[0] << 8 | rx_data[1]);
    int16_t raw_bus   = (int16_t)((int16_t)rx_data[2] << 8 | rx_data[3]);
    int16_t raw_curr  = (int16_t)((int16_t)rx_data[4] << 8 | rx_data[5]);
    //int16_t raw_pwr   = (int16_t)((int16_t)rx_data[6] << 8 | rx_data[7]);

    Power->shunt_volt = (float)raw_shunt / 1000.0f;
    Power->bus_volt   = (float)raw_bus   / 1000.0f;
    Power->current    = (float)raw_curr  / 1000.0f;
    //Power->power      = (float)raw_pwr   / 100.0f;
    Power->power      = Power->bus_volt * Power->current;
}
//缓冲能量计算
/*void Buffer_Calc(Power_Typedef* Power, User_Data_T *user_data)
{
    static uint8_t is_initialized = 0;
    static uint8_t calc_counter = 0;

    if (!is_initialized) {
        Power->buffer_energy = 60.0f;
        is_initialized = 1;
    }

    float power_limit = user_data->robot_status.chassis_power_limit;
    float max_buffer_energy = 60.0f;
    float now_power = Power->power;

    calc_counter++;

    if (calc_counter >= 10)
    {
        Power->buffer_energy += (power_limit - now_power) * 0.01f;
        calc_counter = 0;
    }

    if (Power->buffer_energy > max_buffer_energy) {
        Power->buffer_energy = max_buffer_energy;
    }
    else if (Power->buffer_energy < 0.0f) {
        Power->buffer_energy = 0.0f;
    }
}*/
void Buffer_Calc(Power_Typedef* Power, User_Data_T *user_data)
{
    static uint8_t is_initialized = 0;

    if (!is_initialized) {
        Power->buffer_energy = 60.0f;
        is_initialized = 1;
    }
    float power_limit = 50.0f;
    float max_buffer_energy = 60.0f;
    //power_limit = user_data->robot_status.chassis_power_limit;
    float now_power = Power->power;
    Power->buffer_energy += (power_limit - now_power) * 0.001f;

    if (Power->buffer_energy > max_buffer_energy) {
        Power->buffer_energy = max_buffer_energy;
    }
    else if (Power->buffer_energy < 0.0f) {
        Power->buffer_energy = 0.0f;
    }
}