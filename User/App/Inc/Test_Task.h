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

//这下面是射击检测的
#define K_UP       0.672f   // 上升系数
#define K_DN       0.141f   // 下降系数
#define TH_FIRE    200.0f   // 触发阈值
#define TH_RST     100.0f   // 复位阈值
#define MAX_DROP   1000.0f  // 最大跌落阈值
#define MIN_SLOPE  100.0f   // 最小斜率阈值
#define TIMEOUT_TICKS 10    // 超时计数器上限
#define TH_INT_FIRE   100.0f  // 积分触发阈值

typedef struct {
    float base;      // 基准线
    float last_val;  // 记录上一次的转速，用于算斜率
    float integral;  // 跌落量积分
    bool armed;      // 触发状态
    uint32_t cnt;    // 计数器
    uint8_t  t_out;  // 超时计数器
    bool init;       // 初始化标志
} ShootDet_t;

extern ShootDet_t g_det;
bool Update_Shoot_Det(float speed1, float speed2, ShootDet_t *det);

void Test_Init(void);
void Ctrl_Test_Task(void);

#endif //G4_FRAMEWORK_TEST_TASK_H