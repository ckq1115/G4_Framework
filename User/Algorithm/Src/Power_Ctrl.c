//
// Created by CaoKangqi on 2026/2/23.
//

#include "Power_Ctrl.h"
#include <stdint.h>
#include "DJI_Motor.h"
#include "All_Define.h"
#include <math.h>

void Power_control_init(model_t *model)
{
    model->Kp = 0.3f;
    model->Remaining_Buffer = 25.0f;
    //对于使用M3508电机鼙鼓的底盘，以下参数不要改，参数和减速箱以及底盘均无关，无论负载多少以及减速箱类型，这四个参数均适用
    model->k1 = 1.5756155501e-02f;//kt：M3508鼙鼓的转矩常数Nm/A,对应机械功率项，与转速和电流乘积成正比
    model->k2 = 1.1584598349e-01f;//kr：M3508鼙鼓和C620电调的电阻Ω，对应铜损项，与电流平方成正比
    model->k3 = 1.9202168378e-05f;//k_iron：M3508电机铁损系数 (W/(rad/s)²)，对应铁损项（磁滞/涡流损耗），与转速平方成正比
    model->k4 = 1.5617225437e+00f;//k0：M3508电机和C620电调的静态功率W，对应固定损耗项，与转速和电流无关

    model->rpm_to_rad = 2.0f * 3.1415926f / 60.0f;//RPM转rad/s
}

/**
  * @author: 楠
  * @performance: 电机功率计算函数
  * @parameter: 电机结构体
  * @time: 24-4-1
  * @ReadMe: 依靠电机功率模型计算电机功率
 */
float get_initial_power(DJI_MOTOR_Typedef *MOTOR, model_t *model)
{
    float w = MOTOR->DATA.Speed_now * model->rpm_to_rad;   // rad/s
    float I = MOTOR->PID_S.Output * 20 / 16384;

    float power = model->k1 * w * I
                + model->k2 * I * I
                + model->k3 * w * w
                + model->k4;

    if(power < 0) power = 0;

    return power;
}

void chassis_power_distribute(DJI_MOTOR_Typedef *motor[4],
                              float I_cmd[4],
                              float P_limit,
                              model_t *model)
{
    float A = 0.0f;
    float B = 0.0f;
    float C = 4 * model->k4 - P_limit;

    for(int i = 0; i < 4; i++)
    {
        float w = motor[i]->DATA.Speed_now * model->rpm_to_rad;

        float I = I_cmd[i] * 20.0f / 16384.0f;

        A += model->k2 * I * I;
        B += model->k1 * w * I;
        C += model->k3 * w * w;
    }
    float P_predict = A + B + C + P_limit;
    if(P_predict <= P_limit)
    {
        return; // 不超功率，不做任何处理
    }
    float s = 1.0f;
    if(A < 1e-6f)
    {
        s = 0.0f;
    }
    else
    {
        float delta = B * B - 4.0f * A * C;

        if(delta >= 0.0f)
        {
            s = (-B + sqrtf(delta)) / (2.0f * A);
        }
        else
        {
            s = 0.0f;
        }
    }
    if(s > 1.0f) s = 1.0f;
    if(s < 0.0f) s = 0.0f;
    for(int i = 0; i < 4; i++)
    {
        I_cmd[i] *= s;
    }
}

float SectionLimit_f(float max, float min, float data)
{
    if(max < min)
    {
        float t = max;
        max = min;
        min = t;
    }

    if(data > max) return max;
    if(data < min) return min;

    return data;
}

/**
  * @author: 楠
  * @performance: 功率控制总函数
  * @parameter: 电容标志位（是否开启）
  * @time: 24-4-1
  * @ReadMe: 放在底盘PID解算后即可
 */
uint8_t chassis_power_control(CONTAL_Typedef *RUI_V_CONTAL_V,
                           User_Data_T *usr_data,
                           model_t *model,
                           CAP_RXDATA *CAP_GET,
                           MOTOR_Typdef *MOTOR)
{
    //*可编辑部分*begin*//
    const int16_t PowerCompensation = 15;  //正常模式下的功率补偿
    const uint16_t SuperMaxPower = 200;	    //超级电容下的功率补偿
    const uint16_t capValt = 170;	        //强制退出的电压阈值
    //*可编辑部分*end*//

    uint16_t max_power_limit = 500;  //最大功率限制
    float input_power = 0;		    // 输入功率（裁判系统）
    float chassis_max_power = 0;
    float initial_give_power[4];    // 初始功率由PID计算以及电机数据得到
    float initial_total_power = 0;

    if(usr_data->robot_status.chassis_power_limit != 0 )
    {
        max_power_limit = usr_data->robot_status.chassis_power_limit;	// 得到最大功率限制
    }
//    float chassis_power = usr_data->power_heat_data.chassis_power_reserved;		// 得到底盘功率
    float chassis_power_buffer = usr_data->power_heat_data.buffer_energy;	// 得到缓冲能量

    /*没电容时开启*/
    input_power = (float)max_power_limit;  // 加入缓冲能量
    //input_power = (float)max_power_limit;
    if(CAP_GET->CAP_VOLT > (float)capValt)
    {
        if(RUI_V_CONTAL_V->BOTTOM.CAP == 0)
        {
            // 功率设置略大于最大输入功率，提高电容能量利用率
            chassis_max_power = input_power + (float)PowerCompensation;
        }else
        {
            // 开启电容
            chassis_max_power = input_power + (float)SuperMaxPower;
        }
    }
    else
    {
        // 电容电量低或电容离线时无补偿
        chassis_max_power = input_power;
    }

    initial_give_power[0] = get_initial_power(&MOTOR->DJI_3508_Chassis[0], model);
    initial_give_power[1] = get_initial_power(&MOTOR->DJI_3508_Chassis[1], model);
    initial_give_power[2] = get_initial_power(&MOTOR->DJI_3508_Chassis[2], model);
    initial_give_power[3] = get_initial_power(&MOTOR->DJI_3508_Chassis[3], model);

    for(uint8_t i = 0; i < 4; i++)
    {
        if (initial_give_power[i] < 0) // 不考虑负功(反向电动势)
            continue;
        initial_total_power += initial_give_power[i]; // 获得底盘总功率
    }

    float I_cmd[4];

    I_cmd[0] = MOTOR->DJI_3508_Chassis[0].PID_S.Output;
    I_cmd[1] = MOTOR->DJI_3508_Chassis[1].PID_S.Output;
    I_cmd[2] = MOTOR->DJI_3508_Chassis[2].PID_S.Output;
    I_cmd[3] = MOTOR->DJI_3508_Chassis[3].PID_S.Output;

    DJI_MOTOR_Typedef *motor_ptr[4] = {
        &MOTOR->DJI_3508_Chassis[0],
        &MOTOR->DJI_3508_Chassis[1],
        &MOTOR->DJI_3508_Chassis[2],
        &MOTOR->DJI_3508_Chassis[3]
    };

    chassis_power_distribute(motor_ptr, I_cmd, chassis_max_power, model);

    MOTOR->DJI_3508_Chassis[0].PID_S.Output = I_cmd[0];
    MOTOR->DJI_3508_Chassis[1].PID_S.Output = I_cmd[1];
    MOTOR->DJI_3508_Chassis[2].PID_S.Output = I_cmd[2];
    MOTOR->DJI_3508_Chassis[3].PID_S.Output = I_cmd[3];
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
    float power_limit = 75.0f;
    float max_buffer_energy = 60.0f;
    power_limit = user_data->robot_status.chassis_power_limit;
    float now_power = Power->power;
    Power->buffer_energy += (power_limit - now_power) * 0.001f;

    if (Power->buffer_energy > max_buffer_energy) {
        Power->buffer_energy = max_buffer_energy;
    }
    else if (Power->buffer_energy < 0.0f) {
        Power->buffer_energy = 0.0f;
    }
}