//
// Created by CaoKangqi on 2026/2/23.
//

#include "Power_Ctrl.h"
#include <stdint.h>
#include "DJI_Motor.h"
#include "All_Define.h"
#include <math.h>
CCM_DATA ALL_POWER_RX All_Power;

void Power_control_init(model_t *model)
{
    model->PID_Buffer.Kp = 2;
    model->PID_Buffer.Ki = 0;
    model->PID_Buffer.Kd = 0;
    model->PID_Buffer.ILt = 0;
    model->PID_Buffer.AlLt = 100;

    model->k1 = 1.9851375306e-02f;//kt：M3508鼙鼓的转矩常数Nm/A,对应机械功率项，与转速和电流乘积成正比
    model->k2 = 1.6020757489e-01f;//kr：M3508鼙鼓和C620电调的电阻Ω，对应铜损项，与电流平方成正比
    model->k3 = 1.6222387185e-05f;//k_iron：M3508电机铁损系数 (W/(rad/s)²)，对应铁损项（磁滞/涡流损耗），与转速平方成正比
    model->k4 = 2.2053869352e+00f;//k0：M3508电机和C620电调的静态功率W，对应固定损耗项，与转速和电流无关

    model->rpm_to_rad = 2.0f * 3.1415926f / 60.0f;//RPM转rad/s
}

float chassis_power_model(MOTOR_Typdef *MOTOR, model_t *model)
{
    DJI_MOTOR_Typedef *motor[4] =
    {
        &MOTOR->DJI_3508_Chassis_1,
        &MOTOR->DJI_3508_Chassis_2,
        &MOTOR->DJI_3508_Chassis_3,
        &MOTOR->DJI_3508_Chassis_4
    };

    float sum_wi = 0;
    float sum_i2 = 0;
    float sum_w2 = 0;

    for(int i = 0; i < 4; i++)
    {
        float w = motor[i]->DATA.Speed_now * model->rpm_to_rad;
        float I = motor[i]->PID_S.Output * 0.001f;

        sum_wi += w * I;
        sum_i2 += I * I;
        sum_w2 += w * w;
    }

    float power = model->k1 * sum_wi
                + model->k2 * sum_i2
                + model->k3 * sum_w2
                + model->k4 * 4;

    if(power < 0) power = 0;

    return power;
}

void chassis_power_distribute(MOTOR_Typdef *MOTOR,
                              float P_limit,
                              model_t *model)
{
    DJI_MOTOR_Typedef *motor[4] =
    {
        &MOTOR->DJI_3508_Chassis_1,
        &MOTOR->DJI_3508_Chassis_2,
        &MOTOR->DJI_3508_Chassis_3,
        &MOTOR->DJI_3508_Chassis_4
    };

    float I_cmd[4];

    float A = 0.0f;
    float B = 0.0f;
    float C = 4 * model->k4 - P_limit;

    for(int i = 0; i < 4; i++)
    {
        I_cmd[i] = motor[i]->PID_S.Output;
    }

    for(int i = 0; i < 4; i++)
    {
        float w = motor[i]->DATA.Speed_now * model->rpm_to_rad;
        float I = I_cmd[i] * 0.001f;

        A += model->k2 * I * I;
        B += model->k1 * w * I;
        C += model->k3 * w * w;
    }

    float P_predict = A + B + C;

    if(P_predict <= P_limit)
        return;

    float s = 1.0f;

    if(A < 1e-6f)
    {
        s = 0.0f;
    }
    else
    {
        float discriminant = B * B - 4.0f * A * C;

        if(discriminant >= 0)
            s = (-B + sqrtf(discriminant)) / (2.0f * A);
        else
            s = 0.0f;
    }

    if(s > 1.0f) s = 1.0f;
    if(s < 0.0f) s = 0.0f;

    for(int i = 0; i < 4; i++)
    {
        motor[i]->PID_S.Output *= s;
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
  * @performance: 缓冲能量PID计算
  * @parameter: PID 缓冲能量 要求剩余的最低缓冲能量
  * @time: 24-4-1
  * @ReadMe:
 */
void PID_buffer(PID_buffer_t *PID_buffer, float power_buffer, float temp)//
{
    PID_buffer->Error[0] = temp - power_buffer;
    /*比例输出*/
    PID_buffer->P_out = (PID_buffer->Error[0] * PID_buffer->Kp);
    /*积分输出*/
    PID_buffer->I_out += (PID_buffer->Error[0] * PID_buffer->Ki);
    /*积分限幅*/
    PID_buffer->I_out = SectionLimit_f(PID_buffer->ILt, -PID_buffer->ILt, PID_buffer->I_out);
    /*微分输出*/
    PID_buffer->D_out = -(PID_buffer->Error[0] - PID_buffer->Error[1]) * PID_buffer->Kd;
    /*数据迭代*/
    PID_buffer->Error[1] = PID_buffer->Error[0];
    /*角度环总输出*/
    PID_buffer->All_out = (PID_buffer->P_out + PID_buffer->I_out + PID_buffer->D_out);
    /*总输出限幅*/
    PID_buffer->All_out = SectionLimit_f(PID_buffer->AlLt, -PID_buffer->AlLt, PID_buffer->All_out);
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

    uint16_t max_power_limit = 80;  //最大功率限制
    float input_power = 0;		    // 输入功率（裁判系统）
    float chassis_max_power = 0;

    if(usr_data->robot_status.chassis_power_limit != 0 )
    {
        max_power_limit = usr_data->robot_status.chassis_power_limit;	// 得到最大功率限制
    }
//    float chassis_power = usr_data->power_heat_data.chassis_power_reserved;		// 得到底盘功率
    float chassis_power_buffer = usr_data->power_heat_data.buffer_energy;	// 得到缓冲能量

    /*没电容时开启*/
    PID_buffer(&model->PID_Buffer, chassis_power_buffer, 25);  // 缓冲能量闭环

    input_power = (float)max_power_limit + model->PID_Buffer.All_out;  // 加入缓冲能量

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

    chassis_power_distribute(MOTOR,
                         chassis_max_power,
                         model);
    return DF_READY;
}

//功率计接收解算函数
void CAN_POWER_Rx(Power_Typedef* Power, uint8_t *rx_data)
{
    int16_t raw_shunt = (int16_t)((int16_t)rx_data[0] << 8 | rx_data[1]);
    int16_t raw_bus   = (int16_t)((int16_t)rx_data[2] << 8 | rx_data[3]);
    int16_t raw_curr  = (int16_t)((int16_t)rx_data[4] << 8 | rx_data[5]);
    int16_t raw_pwr   = (int16_t)((int16_t)rx_data[6] << 8 | rx_data[7]);

    Power->shunt_volt = (float)raw_shunt / 1000.0f;
    Power->bus_volt   = (float)raw_bus   / 1000.0f;
    Power->current    = (float)raw_curr  / 1000.0f;
    //Power->power      = (float)raw_pwr   / 100.0f;
    Power->power      = Power->bus_volt * Power->current;
}
