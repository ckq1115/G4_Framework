//
// Created by CaoKangqi on 2026/4/7.
//
#include "RMUI.h"
#include <string.h>

#include "BSP_DWT.h"

#define TXBUF_SIZE 1024
static uint8_t txbuf[TXBUF_SIZE];
static size_t head = 0, tail = 0;

// 频率控制 (30Hz -> 约33000us)
#define SEND_INTERVAL_US 33000

// 限制数值范围
static float clamp(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

void UI_AddData(const uint8_t data[], const size_t len) {
    for (size_t i = 0; i < len; i++) {
        if ((tail + 1) % TXBUF_SIZE == head) break;
        txbuf[tail] = data[i];
        tail = (tail + 1) % TXBUF_SIZE;
    }
}

void UI_Init(UI_t* ui, const ui_config_t* config) {
    ui->config = *config;

    ui->state = UI_STATE_INIT;
    ui->init_index = 0;
    ui->update_index = 0;
    ui->last_poll_tick = 0;

    // 默认数据初始化
    ui->yaw_ecd = 0.0f;
    ui->yaw = 0.0f;
    ui->pitch = 0.0f;
    ui->is_hurt = false;
    ui->is_detect = false;

    // 机器人组件颜色初始化，统一默认为绿色
    ui->robot.w1 = ui->robot.w2 = ui->robot.w3 = ui->robot.w4 = UI_COLOR_GREEN;
    ui->robot.s1 = ui->robot.s2 = ui->robot.s3 = ui->robot.s4 = UI_COLOR_GREEN;
    ui->robot.yaw1 = ui->robot.yaw2 = ui->robot.pitch = UI_COLOR_GREEN;
    ui->robot.rub1 = ui->robot.rub2 = ui->robot.rub3 = ui->robot.rub4 = UI_COLOR_GREEN;
    ui->robot.shoot = UI_COLOR_GREEN;
    ui->robot.referee = UI_COLOR_GREEN;
    ui->robot.aim = UI_COLOR_GREEN;
}

static void updateLib(UI_t* ui) {
    int start_angle, end_angle, angle;

    // 云台相对底盘角度 (12点钟方向为0°，顺时针绘制)
    start_angle = (int)(ui->yaw_ecd - 20.0f);
    end_angle = (int)(ui->yaw_ecd + 20.0f);
    if (start_angle < 0) start_angle += 360;
    if (end_angle < 0) end_angle += 360;
    ui_g_1_dir->start_angle = start_angle;
    ui_g_1_dir->end_angle = end_angle;

    // 云台 IMU 绝对角度显示
    ui_g_2_yaw->number = (int32_t)ui->yaw;
    ui_g_2_pitch->number = (int32_t)ui->pitch;

    // 伤害方向提示
    if (ui->is_hurt) {
        start_angle = (int)(-ui->hurt_dir - 20.0f);
        end_angle = (int)(-ui->hurt_dir + 20.0f);
        if (start_angle < 0) start_angle += 360;
        if (end_angle < 0) end_angle += 360;
        ui_g_1_hurt->width = 10; // 宽度非0代表显示
        ui_g_1_hurt->start_angle = start_angle;
        ui_g_1_hurt->end_angle = end_angle;
    } else {
        ui_g_1_hurt->width = 0;  // 隐藏
    }

    // 自瞄识别框状态
    ui_g_1_detect->width = ui->is_detect ? 21 : 0;

    // 提示条：能量、转速、弹频、电流

    // 超级电容能量比例 (范围 275°~313°, 共38°)
    angle = (int)((ui->cap / ui->config.max_cap) * 38.0f);
    angle = (int)clamp((float)angle, 1.0f, 38.0f);
    ui_g_1_bar0->start_angle = 275;
    ui_g_1_bar0->end_angle = 275 + angle;
    ui_g_2_number0->number = (int32_t)ui->cap;

    // 底盘旋转速度 (范围 227°~265°, 反向绘制)
    float abs_wr = (ui->wr < 0) ? -ui->wr : ui->wr;
    angle = (int)((abs_wr / ui->config.max_wr) * 38.0f);
    angle = (int)clamp((float)angle, 1.0f, 38.0f);
    ui_g_1_bar1->start_angle = 265 - angle;
    ui_g_1_bar1->end_angle = 265;
    ui_g_2_number1->number = (int32_t)ui->wr;

    // 弹频/弹速 (范围 95°~133°)
    angle = (int)((ui->bullet / ui->config.max_bullet) * 38.0f);
    angle = (int)clamp((float)angle, 1.0f, 38.0f);
    ui_g_1_bar2->start_angle = 95;
    ui_g_1_bar2->end_angle = 95 + angle;
    ui_g_2_number2->number = (int32_t)ui->bullet;

    // 拨弹电机电流 (范围 47°~85°, 反向绘制)
    float abs_shoot = (ui->shoot < 0) ? -ui->shoot : ui->shoot;
    angle = (int)((abs_shoot / ui->config.max_shoot) * 38.0f);
    angle = (int)clamp((float)angle, 1.0f, 38.0f);
    ui_g_1_bar3->start_angle = 85 - angle;
    ui_g_1_bar3->end_angle = 85;
    ui_g_2_number3->number = (int32_t)ui->shoot;

    // 模块在线状态颜色同步

    // 图层 30: 底盘
    ui_g_30_w1->color = ui->robot.w1;
    ui_g_30_w2->color = ui->robot.w2;
    ui_g_30_w3->color = ui->robot.w3;
    ui_g_30_w4->color = ui->robot.w4;
    ui_g_30_s1->color = ui->robot.s1;
    ui_g_30_s2->color = ui->robot.s2;
    ui_g_30_s3->color = ui->robot.s3;

    // 图层 31: 云台与摩擦轮
    ui_g_31_s4->color = ui->robot.s4;
    ui_g_31_cap->color = ui->robot.referee;
    ui_g_31_yaw1->color = ui->robot.yaw1;
    ui_g_31_yaw2->color = ui->robot.yaw2;
    ui_g_31_pitch->color = ui->robot.pitch;
    ui_g_31_rub_left1->color = ui->robot.rub1;
    ui_g_31_rub_right1->color = ui->robot.rub2;

    // 图层 32: 拨弹与自瞄
    ui_g_32_rub_left2->color = ui->robot.rub3;
    ui_g_32_rub_right2->color = ui->robot.rub4;
    ui_g_32_shoot->color = ui->robot.shoot;
    ui_g_32_aim->color = ui->robot.aim;
}

void UI_OnLoop(UI_t* ui) {
    update_rmui_robot_id();
    updateLib(ui);

    uint64_t now_us = DWT_GetTimeline_us();

    if (now_us - ui->last_poll_tick >= SEND_INTERVAL_US) {
        ui->last_poll_tick = now_us;

        if (ui->state == UI_STATE_INIT) {
            ui_init_g[ui->init_index]();
            ui->init_index++;
            if (ui->init_index >= UI_INIT_G_COUNT) {
                ui->state = UI_STATE_UPDATE;
            }
        } else if (ui->state == UI_STATE_UPDATE) {
            ui_update_g[ui->update_index]();
            ui->update_index++;
            if (ui->update_index >= UI_UPDATE_G_COUNT) {
                ui->update_index = 0;
            }
        }
    }
}

void UI_SendUartCmd(UI_t* ui) {
    if (head != tail) {
        uint16_t send_len = 0;
        static uint8_t temp_send_buf[TXBUF_SIZE]; // 提取缓冲区

        while (head != tail && send_len < TXBUF_SIZE) {
            temp_send_buf[send_len++] = txbuf[head];
            head = (head + 1) % TXBUF_SIZE;
        }
        HAL_UART_Transmit_DMA(&huart1, temp_send_buf, send_len);
    }
}