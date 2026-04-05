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
ALL_POWER_RX All_Power = {0};

CONTAL_Typedef contal;
CAP_RXDATA CAP_Get;

User_Data_T User_data;
uint8_t Referee_Rx_Buf[2][REFEREE_RXFRAME_LENGTH];

uint8_t rx_buffer[64];
SpeedData_t current_data;

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
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Referee_Rx_Buf[0], REFEREE_RXFRAME_LENGTH);//裁判系统串口
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);//关闭 DMA 半传中断

    HAL_UART_DMAStop(&huart2);
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
    volatile uint32_t tmp2 = huart2.Instance->RDR;
    (void)tmp2;
    __HAL_DMA_CLEAR_FLAG(&hdma_usart2_rx, DMA_FLAG_TC8 | DMA_FLAG_HT8 | DMA_FLAG_TE8);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, 64);
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

    FDCAN_Config(&hfdcan1, FDCAN_RX_FIFO0);
    FDCAN_Config(&hfdcan2, FDCAN_RX_FIFO1);
    FDCAN_Config(&hfdcan3, FDCAN_RX_FIFO0);

    WS2812_Init();
    W25N01GV_Init();
    IMU_Gyro_Calib_Initiate();
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);//IMU加热
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);//蜂鸣器
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_2, 100.0);
    HAL_Delay(500);
    __HAL_TIM_SET_COMPARE(&htim20, TIM_CHANNEL_2, 0);
    HAL_TIM_PWM_Stop(&htim20, TIM_CHANNEL_2);
}