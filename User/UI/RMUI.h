//
// Created by CaoKangqi on 2026/4/7.
//

#ifndef G4_FRAMEWORK_UI_H
#define G4_FRAMEWORK_UI_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "RM_UI_Adapter.h"

typedef enum {
    UI_COLOR_MAIN   = 0,
    UI_COLOR_YELLOW = 1,
    UI_COLOR_GREEN  = 2,
    UI_COLOR_ORANGE = 3,
    UI_COLOR_PURPLE = 4,
    UI_COLOR_PINK   = 5,
    UI_COLOR_CYAN   = 6,
    UI_COLOR_BLACK  = 7,
    UI_COLOR_WHITE  = 8,
} ui_color_e;

typedef struct {
    uint8_t can_port;
    uint32_t master_id;
    uint32_t slave_id;

    float max_cap;
    float max_wr;
    float max_bullet;
    float max_shoot;
} ui_config_t;

typedef struct {
    ui_config_t config;

    // 状态数据
    float yaw_ecd;
    float yaw, pitch;
    bool is_hurt;
    float hurt_dir;
    bool is_detect;

    // 提示条数据
    float cap;
    float wr;
    float bullet;
    float shoot;

    // 机器人组件颜色状态
    struct {
        ui_color_e w1, w2, w3, w4;
        ui_color_e s1, s2, s3, s4;
        ui_color_e yaw1, yaw2, pitch;
        ui_color_e rub1, rub2, rub3, rub4;
        ui_color_e shoot;
        ui_color_e referee;
        ui_color_e aim;
    } robot;

    enum { UI_STATE_INIT, UI_STATE_UPDATE } state;
    size_t init_index;
    size_t update_index;

    uint64_t last_poll_tick;
} UI_t;

void UI_Init(UI_t* ui, const ui_config_t* config);
void UI_OnLoop(UI_t* ui);
void UI_SendUartCmd(UI_t* ui);
void UI_AddData(const uint8_t data[], size_t len);

#endif //G4_FRAMEWORK_UI_H