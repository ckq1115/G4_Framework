//
// Created by CaoKangqi on 2026/2/27.
//

#ifndef G4_FRAMEWORK_TEST_TASK_H
#define G4_FRAMEWORK_TEST_TASK_H
#include <stdbool.h>
#include <stdint.h>

#pragma pack(1)
typedef struct {
    uint8_t  header[2]; // 帧头: 0xAA 0xBB
    float    speed_i2;
    float    speed_i3;
} SpeedData_t;
#pragma pack()

// 接收缓冲区
extern uint8_t rx_buffer[64];
extern SpeedData_t current_data;



void Test_Init(void);
void Ctrl_Test_Task(void);

typedef struct __attribute__((packed))
{
    int16_t ch0;
    int16_t ch1;
    float pitch;
    float roll;
    float yaw;
    int16_t ch2;
    int16_t ch3;
    uint8_t s1:2;
    uint8_t s2:2;
}B2B_Tx_t;//双板通讯发送

typedef struct __attribute__((packed)){
    int16_t ch0;
    int16_t ch1;
    float pitch;
    float roll;
    float yaw;
    int16_t ch2;
    int16_t ch3;
    uint8_t s1:2;
    uint8_t s2:2;
}B2B_Rx_t;//双板通讯接收

void Test_Tx(void);
#endif //G4_FRAMEWORK_TEST_TASK_H