#ifndef G4_FRAMEWORK_POWER_CAP_H
#define G4_FRAMEWORK_POWER_CAP_H

#include <stdint.h>
#include "BSP-FDCAN.h"
#include "Referee.h"

#pragma pack(1) // 确保结构体按1字节对齐，防止CAN数据错位

/**
 * @brief 电容接收数据结构体
 */
typedef struct {
    uint8_t  cap_key;       // FSBB开关状态
    uint8_t  cap_state;     // 状态位 (0:正常, 1:故障)
    float    nowPower;      // 当前输入功率
    uint8_t  capVolt;       // 电容电压
    uint8_t  check_code;    // 校验位 (0xAA)
    uint8_t ONLINE_JUDGE_TIME;
} CapRxData_t;

/**
 * @brief 电容发送控制联合体
 */
typedef union {
    struct {
        uint8_t power_key;       // 开关控制
        uint8_t capPowerLimit;   // 功率限制值 (W)
        uint8_t buffer_now;      // 裁判系统当前剩余缓冲能量
        uint8_t robot_state;     // 机器人存活状态
        uint8_t check_code;      // 校验位 (0xAA)
        uint8_t reserved[3];     // 填充位，凑足CAN帧长度
    } Control;
    uint8_t raw_data[8];         // CAN发送数据缓冲区 (修正为8字节)
} CapSetData_t;

#pragma pack()

typedef struct {
    CapSetData_t set;
    CapRxData_t  get;
} Cap_t;

/* 外部变量声明 */
extern Cap_t cap;
extern int open_cap_flag;

/* 函数声明 */
void Power_Cap_Rx(Cap_t *cap_ptr, const uint8_t *rx_buf);
void Power_Cap_Tx(hcan_t *hcan, uint16_t can_id, Cap_t *cap_ptr, User_Data_T *referee_data);

#endif //G4_FRAMEWORK_POWER_CAP_H