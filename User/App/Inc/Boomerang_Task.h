//
// Created by CaoKangqi on 2026/3/4.
//

#ifndef G4_FRAMEWORK_BOOMERANG_H
#define G4_FRAMEWORK_BOOMERANG_H

#include "DBUS.h"

#define FRAME_HEADER_1      0x55
#define FRAME_HEADER_2      0x55
#define CMD_SERVO_MOVE      0x03
#define CMD_GET_BATTERY     0x0F
#define RX_BUF_SIZE 32

extern int pin_switch;
extern int  Angle_To_CCR(float angle);
void ServoMove(uint8_t id, uint16_t angle, uint16_t time_ms);
void BuildServoCommand(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t params_len, uint8_t *buffer);
void Controlservo(uint8_t mod);
void ServoMotorModeWrite(uint8_t id, uint8_t mode, uint8_t turn_mode, int16_t speed) ;
void ServoPosRead(uint8_t id);
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t Channel, float angle);

void ServoMoveMulti(uint8_t num_servos, uint8_t *ids, uint16_t *angles, uint16_t time_ms);

#endif //G4_FRAMEWORK_BOOMERANG_H