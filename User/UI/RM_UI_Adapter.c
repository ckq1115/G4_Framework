//
// Created by CaoKangqi on 2026/4/7.
//
#include "RM_UI_Adapter.h"

#include "RMUI.h"

extern int ui_self_id;

void print_message(const uint8_t* message, const int length) {
    UI_AddData(message, length);
}

void update_rmui_robot_id(void) {
    ui_self_id = User_data.robot_status.robot_id; // 从全局 User_data 获取机器人ID
}

// 数组初始化
void (*ui_init_g[])(void) = {
    ui_init_g_00, ui_init_g_01, ui_init_g_02, ui_init_g_03,
    ui_init_g_04, ui_init_g_05, ui_init_g_1,  ui_init_g_2,
    ui_init_g_30, ui_init_g_31, ui_init_g_32
};

void (*ui_update_g[])(void) = {
    ui_update_g_1,  ui_update_g_2,
    ui_update_g_1,  ui_update_g_30,
    ui_update_g_1,  ui_update_g_31,
    ui_update_g_1,  ui_update_g_32
};