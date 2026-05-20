//
// Created by CaoKangqi on 2026/2/27.
//
#include "Test_Task.h"

#include "All_Init.h"
#include "controller.h"

void Test_Init(void) {
    /*float PID_P_S1[3] = {0.6f,0.0f,0.0f};
    float PID_S_S1[3] = {0.3f,0.002f,0.0f};*/
    float PID_P_Y[3] = {1.36f,0.0f,0.0f};
    float PID_S_Y[3] = {3.0f,0.002f,0.8f};

    PID_Init(&All_Motor.DM4310_Yaw.PID_P,5000,500,
        PID_P_Y,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器
    PID_Init(&All_Motor.DM4310_Yaw.PID_S,100,50,
        PID_S_Y,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器

    float PID_P_P[3] = {0.0f,0.0f,0.0f};
    float PID_S_P[3] = {0.0f,0.0f,0.0f};

    PID_Init(&All_Motor.DM4310_Pitch.PID_P,5000,500,
        PID_P_P,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器

    PID_Init(&All_Motor.DM4310_Pitch.PID_S,64,30,
        PID_S_P,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器

    float PID_3508_Sp[3] = {5.0f, 0.1f, 0.0f}; // 3508 速度环参数
    PID_Init(&All_Motor.DJI_3508_Pull.PID_S, 16384.0f, 3000.0f, PID_3508_Sp,
                 0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);

}

float a = 0.0f;
void Ctrl_Test_Task(void) {
    static float target_omega = 0;
    //All_Motor.DM4310_Yaw.PID_P.Ref = 50*cosf(2.0*3.14*1*t);
    if (IMU_Data.YawTotalAngle != 0.0f) {
        target_omega = DBUS.Remote.CH2 * 0.3f + DBUS.KeyBoard.E * 5.0f - DBUS.KeyBoard.Q * 5.0f + DBUS.Mouse.X_Flt * 2.0f;
        All_Motor.DM4310_Yaw.PID_P.Ref += target_omega*0.001f;
    }
    /*PID_Calculate(&All_Motor.DM4310_Yaw.PID_P,All_Motor.DM4310_Yaw.DATA.Angle_Infinite,All_Motor.DM4310_Yaw.PID_P.Ref);
    PID_Calculate(&All_Motor.DM4310_Yaw.PID_S,All_Motor.DM4310_Yaw.DATA.Speed_now,All_Motor.DM4310_Yaw.PID_P.Output);*/
    PID_Calculate(&All_Motor.DM4310_Yaw.PID_P,IMU_Data.YawTotalAngle,All_Motor.DM4310_Yaw.PID_P.Ref);
    PID_Calculate(&All_Motor.DM4310_Yaw.PID_S,IMU_Data.gyro[2],All_Motor.DM4310_Yaw.PID_P.Output);

    PID_Calculate(&All_Motor.DM4310_Pitch.PID_P,IMU_Data.pitch,All_Motor.DM4310_Pitch.PID_P.Ref);
    PID_Calculate(&All_Motor.DM4310_Pitch.PID_S,IMU_Data.gyro[1],All_Motor.DM4310_Pitch.PID_P.Output);

    a = 10*cosf(IMU_Data.pitch * DEG2RAD);
    DM_Motor_Send(&hfdcan2,0x3FE,-All_Motor.DM4310_Yaw.PID_S.Output,-All_Motor.DM4310_Pitch.PID_S.Output-a,0,0);
    //DM_Motor_Send(&hfdcan2,0x3FE,All_Motor.DM4310_Yaw.PID_P.Ref,0,0,0);

    All_Motor.DJI_3508_Pull.PID_S.Ref = DBUS.Remote.CH0 *13.7f;
    PID_Calculate(&All_Motor.DJI_3508_Pull.PID_S,
                      (float)All_Motor.DJI_3508_Pull.DATA.Speed_now,
                      All_Motor.DJI_3508_Pull.PID_S.Ref);
    DJI_Motor_Send(&hfdcan2,0x200,All_Motor.DJI_3508_Pull.PID_S.Output,0,0,0);
}

B2B_Tx_t Tx_Data = {0};
B2B_Rx_t Rx_Data = {0};
custom_client_data_t my_control = {0};
//自定义长度CAN通讯测试，用于双板通讯
void Test_Tx(void) {
    Tx_Data.ch0 = DBUS.Remote.CH0;
    Tx_Data.ch1 = DBUS.Remote.CH1;
    Tx_Data.ch2 = DBUS.Remote.CH2;
    Tx_Data.ch3 = DBUS.Remote.CH3;
    Tx_Data.s1 = DBUS.Remote.S1;
    Tx_Data.s2 = DBUS.Remote.S2;
    Tx_Data.pitch = IMU_Data.pitch;
    Tx_Data.roll = IMU_Data.roll;
    Tx_Data.yaw = IMU_Data.yaw;
    CAN_TP_Send_Struct(&hfdcan1, &Tx_Data, sizeof(B2B_Tx_t));

    my_control.key_value = (1 << 0); // 按下 W 键

    my_control.x_position = 10;
    my_control.y_position = 0;

    // 3. 设置鼠标点击
    my_control.mouse_left = 0; // 0抬起，1按下

    // 4. 发送
    Referee_Send_KeyMouse(&my_control);
}

void CAN_TP_On_Struct_Received(uint8_t *data_ptr, uint16_t len)
{
    memcpy(&Rx_Data, data_ptr, sizeof(B2B_Rx_t));
}


/**
 * @brief Modbus CRC16 校验计算函数
 */
static uint16_t Modbus_CRC16(uint8_t *buf, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (uint8_t i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief  从K型热电偶采集器读取当前温度
 * @param  huart: 指向 STM32 UART 句柄的指针 (如 &huart1)
 * @return float: 返回真实温度值（度），若读取失败或校验错误则返回 -999.0f
 */
float Thermocouple_Read_Temp(UART_HandleTypeDef *huart) {
    // Modbus 查询指令：01 03 00 00 00 01 84 0A
    uint8_t tx_cmd[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
    uint8_t rx_buf[7] = {0};
    float temperature = -999.0f; // 默认错误返回值
    // 切换为发送模式，发送查询指令
    RS485_SET_TX_MODE();
    HAL_UART_Transmit(huart, tx_cmd, 8, 50);
    // 切换回接收模式
    RS485_SET_RX_MODE();
    // 阻塞接收接收传感器的 7 字节回传数据（超时时间设为 100ms）
    if (HAL_UART_Receive(huart, rx_buf, 7, 100) == HAL_OK) {
        // 验证设备地址、功能码以及数据字节数是否匹配
        if (rx_buf[0] == 0x01 && rx_buf[1] == 0x03 && rx_buf[2] == 0x02) {
            // CRC 校验验证
            uint16_t calc_crc = Modbus_CRC16(rx_buf, 5); // 计算前5个字节的CRC
            uint16_t recv_crc = (rx_buf[6] << 8) | rx_buf[5]; // 接收到的CRC（低字节在前，高字节在后）
            if (calc_crc == recv_crc) {
                // 解析温度数据（第4、5字节组合成16位整型）
                int16_t raw_temp = (int16_t)((rx_buf[3] << 8) | rx_buf[4]);
                // 真实温度 = 原始数据 / 10.0
                temperature = (float)raw_temp / 10.0f;
            }
        }
    }
    return temperature;
}