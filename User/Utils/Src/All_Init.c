//
// Created by CaoKangqi on 2026/2/13.
//
#include "All_Init.h"

//DBUS
uint8_t DBUS_RX_DATA[18];
DBUS_Typedef C_DBUS = { 0 };
DBUS_UNION_Typdef C_DBUS_UNION = { 0 };

CCM_DATA MOTOR_Typdef All_Motor;
CCM_DATA ROOT_STATUS_Typedef ROOT_Status;

CONTAL_Typedef contal;
CAP_RXDATA CAP_Get;

User_Data_T User_data;
ALL_RX_Data_T Referee_Rx_Buf;

uint32_t stm32_id[3];
void Get_UID(uint32_t *uid) {
    uid[0] = HAL_GetUIDw0();
    uid[1] = HAL_GetUIDw1();
    uid[2] = HAL_GetUIDw2();
}
void All_Init() {
    DWT_Init(170);
    Get_UID(stm32_id);

    HAL_UART_DMAStop(&huart3);
    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
    volatile uint32_t tmp3 = huart3.Instance->RDR;
    (void)tmp3;
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);//关闭 DMA 半传中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3,DBUS_RX_DATA,18);//DBUS接收

    HAL_UART_DMAStop(&huart1);
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
    volatile uint32_t tmp1 = huart1.Instance->RDR;
    (void)tmp1;
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);//关闭 DMA 半传中断
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Referee_Rx_Buf.Data,255);//裁判系统串口

    FDCAN_Config(&hfdcan1, FDCAN_RX_FIFO0);
    FDCAN_Config(&hfdcan2, FDCAN_RX_FIFO1);
    FDCAN_Config(&hfdcan3, FDCAN_RX_FIFO0);

    WS2812_Init();
    W25N01GV_Init();
    IMU_Gyro_Calib_Initiate();
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);//IMU加热
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);//蜂鸣器
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_2, 100.0);
    HAL_Delay(500);
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_2, 0);
    HAL_TIM_PWM_Stop(&htim20, TIM_CHANNEL_2);
}