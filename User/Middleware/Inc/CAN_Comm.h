//
// Created by CaoKangqi on 2026/2/19.
//

#ifndef FDCAN_TEST_G4_CAN_COMM_H
#define FDCAN_TEST_G4_CAN_COMM_H

#include "stdint.h"
#include "string.h"
#include "BSP-FDCAN.h"
#include "BSP_DWT.h"

#define CAN_TP_SINGLE_ID       0x500
#define CAN_TP_MAX_PAYLOAD     256

typedef struct {
    uint8_t  buf[CAN_TP_MAX_PAYLOAD];
    uint16_t total_len;
    uint16_t offset;
    uint8_t  seq;
    uint32_t last_tick;
    uint8_t  is_sending;
} CAN_TP_Tx_State_t;

typedef struct {
    uint8_t  buf[CAN_TP_MAX_PAYLOAD];
    uint16_t current_len;
    uint8_t  next_seq;
    uint8_t  is_active;
} CAN_TP_Rx_t;

uint8_t CAN_TP_Send_Struct(FDCAN_HandleTypeDef *hfdcan, void *data_ptr, uint16_t len);
void CAN_TP_Rx_Parser(uint8_t *rx_data, uint8_t dlc_len);
void CAN_TP_On_Struct_Received(uint8_t *data_ptr, uint16_t len);

#endif //FDCAN_TEST_G4_CAN_COMM_H