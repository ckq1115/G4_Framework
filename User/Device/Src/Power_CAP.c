#include "Power_CAP.h"
#include <string.h>

Cap_t cap;
int open_cap_flag = 0;

/**
 * @brief 电容接收数据解算
 * @param cap_ptr 电容数据结构体指针
 * @param rx_buf  CAN接收缓冲区 (8 bytes)
 */
void Power_Cap_Rx(Cap_t *cap_ptr, const uint8_t *rx_buf)
{
    if (rx_buf[7] == 0xAA)
    {
        cap_ptr->get.cap_key   = rx_buf[0];
        cap_ptr->get.cap_state = rx_buf[1];

        memcpy(&cap_ptr->get.nowPower, &rx_buf[2], 4);

        cap_ptr->get.capVolt    = rx_buf[6];
        cap_ptr->get.check_code = rx_buf[7];
        cap_ptr->get.ONLINE_JUDGE_TIME = 10;
    }
}

/**
 * @brief 电容控制数据发送
 * @param hcan         CAN句柄
 * @param can_id       电容CAN ID
 * @param cap_ptr      电容数据结构体指针
 * @param referee_data 裁判系统数据指针
 */
void Power_Cap_Tx(hcan_t *hcan, uint16_t can_id, Cap_t *cap_ptr, User_Data_T *referee_data)
{
    if (cap_ptr == NULL || referee_data == NULL) return;

    cap_ptr->set.Control.power_key      = (uint8_t)open_cap_flag;
    //cap_ptr->set.Control.capPowerLimit = (uint8_t)referee_data->robot_status.chassis_power_limit;
    cap_ptr->set.Control.capPowerLimit = 55;
    //cap_ptr->set.Control.buffer_now     = (uint8_t)referee_data->power_heat_data.buffer_energy;

    //cap_ptr->set.Control.robot_state    = (referee_data->robot_status.current_HP > 0) ? 1 : 0;
    cap_ptr->set.Control.robot_state    = 1;
    cap_ptr->set.Control.check_code     = 0xAA;

    FDCAN_Send_Msg(hcan, can_id, cap_ptr->set.raw_data, 8);
}