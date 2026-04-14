//
// Created by CaoKangqi on 2026/4/7.
//

#ifndef G4_FRAMEWORK_RM_UI_ADAPTER_H
#define G4_FRAMEWORK_RM_UI_ADAPTER_H

#include <stdint.h>
#include "Referee.h"
#include "UI.h"

void print_message(const uint8_t* message, const int length);

void update_rmui_robot_id(void);

extern void (*ui_init_g[])(void);
extern void (*ui_update_g[])(void);

#define UI_INIT_G_COUNT 11
#define UI_UPDATE_G_COUNT 8


#endif //G4_FRAMEWORK_RM_UI_ADAPTER_H