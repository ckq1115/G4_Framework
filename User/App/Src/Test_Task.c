//
// Created by CaoKangqi on 2026/2/27.
//
#include "Test_Task.h"

#include "All_Init.h"
#include "controller.h"

void Test_Init(void) {
    /*float PID_P_S1[3] = {0.6f,0.0f,0.0f};
    float PID_S_S1[3] = {0.3f,0.002f,0.0f};*/
    float PID_P_S1[3] = {1.36f,0.0f,0.0f};
    float PID_S_S1[3] = {3.0f,0.002f,0.8f};



    PID_Init(&All_Motor.DM4310_Yaw.PID_P,5000,500,
        PID_P_S1,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器
    PID_Init(&All_Motor.DM4310_Yaw.PID_S,100,50,
        PID_S_S1,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器

}

void Ctrl_Test_Task(void) {
    static float target_omega = 0;
    //All_Motor.DM4310_Yaw.PID_P.Ref = 50*cosf(2.0*3.14*1*t);
    if (IMU_Data.YawTotalAngle != 0.0f) {
        target_omega = DBUS.Remote.CH2_int16 * 0.3f + DBUS.KeyBoard.E * 5.0f - DBUS.KeyBoard.Q * 5.0f + DBUS.Mouse.X_Flt * 2.0f;
        All_Motor.DM4310_Yaw.PID_P.Ref += target_omega*0.001f;
    }

    #define VFF_GAIN  0.05f
    float speed_ff = VFF_GAIN * target_omega;

    /*PID_Calculate(&All_Motor.DM4310_Yaw.PID_P,All_Motor.DM4310_Yaw.DATA.Angle_Infinite,All_Motor.DM4310_Yaw.PID_P.Ref);
    PID_Calculate(&All_Motor.DM4310_Yaw.PID_S,All_Motor.DM4310_Yaw.DATA.Speed_now,All_Motor.DM4310_Yaw.PID_P.Output);*/
    PID_Calculate(&All_Motor.DM4310_Yaw.PID_P,IMU_Data.YawTotalAngle,All_Motor.DM4310_Yaw.PID_P.Ref);
    PID_Calculate(&All_Motor.DM4310_Yaw.PID_S,IMU_Data.gyro[2],All_Motor.DM4310_Yaw.PID_P.Output);

    DM_Motor_Send(&hfdcan2,0x3FE,- (All_Motor.DM4310_Yaw.PID_S.Output + speed_ff),0,0,0);
    //DM_Motor_Send(&hfdcan2,0x3FE,All_Motor.DM4310_Yaw.PID_P.Ref,0,0,0);


}

B2B_Tx_t Tx_Data = {0};
B2B_Rx_t Rx_Data = {0};
custom_client_data_t my_control = {0};
//自定义长度CAN通讯测试，用于双板通讯
void Test_Tx(void) {
    Tx_Data.ch0 = DBUS.Remote.CH0_int16;
    Tx_Data.ch1 = DBUS.Remote.CH1_int16;
    Tx_Data.ch2 = DBUS.Remote.CH2_int16;
    Tx_Data.ch3 = DBUS.Remote.CH3_int16;
    Tx_Data.s1 = DBUS.Remote.S1_u8;
    Tx_Data.s2 = DBUS.Remote.S2_u8;
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