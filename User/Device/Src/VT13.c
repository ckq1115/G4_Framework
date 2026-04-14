//
// Created by CaoKangqi on 2026/3/21.
//

#include "VT13.h"
#include "CKQ_MATH.h"

#define RUI_DF_KEY_UP    0
#define RUI_DF_KEY_CLICK 1
#define RUI_DF_KEY_PRESS 2

static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);
static bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len);
static float VT_OneFilter(float last, float now, float thresholdValue);
static float VT_deadband(float Value, float minValue, float maxValue);
static uint8_t VT_UpdateKeyStatus(uint8_t is_pressed, uint8_t *press_time);
static void VT_HandleKeyToggle(uint8_t is_pressed, uint8_t *lock_flag, uint8_t *toggle_number);

/**
 * @brief VT13 协议解析
 */
void VT13_Resolved(uint8_t* Data, VT13_Typedef* VT13)
{
    // 校验帧头 0xA9, 0x53 及 CRC16
    if (Data[0] == 0XA9 && Data[1] == 0X53 && verify_crc16_check_sum(Data, 21))
    {
        VT13->CRC_flag = true;

        // 遥控器数据解算
        VT13->Remote.Channel[0] = ((int16_t)(((uint16_t)(Data)[2] | (uint16_t)(Data)[3] << 8) & 0x07FF)) - 1024;
        VT13->Remote.Channel[1] = ((int16_t)(((uint16_t)(Data)[3] >> 3 | (uint16_t)(Data)[4] << 5) & 0x07FF)) - 1024;
        VT13->Remote.Channel[2] = ((int16_t)(((uint16_t)(Data)[4] >> 6 | (uint16_t)(Data)[5] << 2 | (uint16_t)(Data)[6] << 10) & 0x07FF)) - 1024;
        VT13->Remote.Channel[3] = ((int16_t)(((uint16_t)(Data)[6] >> 1 | (uint16_t)(Data)[7] << 7) & 0x07FF)) - 1024;
        VT13->Remote.wheel      = ((int16_t)(((uint16_t)(Data)[8] >> 1 | (uint16_t)(Data)[9] << 7) & 0x07FF)) - 1024;

        VT13->Remote.mode_sw = (Data[7] >> 4) & 0x03;
        VT13->Remote.pause   = (Data[7] >> 6) & 0x01;
        VT13->Remote.fn_1    = (Data[7] >> 7) & 0x01;
        VT13->Remote.fn_2    = (Data[8] >> 0) & 0x01;
        VT13->Remote.trigger = (Data[9] >> 4) & 0x01;

        // 鼠标数据解析
        int16_t m_x = (int16_t)(Data[10] | (Data[11] << 8));
        int16_t m_y = (int16_t)(Data[12] | (Data[13] << 8));
        int16_t m_z = (int16_t)(Data[14] | (Data[15] << 8));

        // 鼠标数值滤波与死区处理
        VT13->Mouse.X_Flt = VT_deadband(VT_OneFilter(VT13->Mouse.X_Flt, (float)m_x, 500), -3.0e-3f, 3.0e-3f);
        VT13->Mouse.Y_Flt = VT_deadband(VT_OneFilter(VT13->Mouse.Y_Flt, (float)m_y, 500), -3.0e-3f, 3.0e-3f);
        VT13->Mouse.Z_Flt = VT_deadband(VT_OneFilter(VT13->Mouse.Z_Flt, (float)m_z, 500), -3.0e-3f, 3.0e-3f);

        // 鼠标按键状态更新 (L/R/M)
        VT13->Mouse.L_State = VT_UpdateKeyStatus((Data[16] >> 0) & 0x01, (uint8_t*)&VT13->Mouse.L_PressTime);
        VT13->Mouse.R_State = VT_UpdateKeyStatus((Data[16] >> 2) & 0x01, (uint8_t*)&VT13->Mouse.R_PressTime);
        VT13->Mouse.M_State = VT_UpdateKeyStatus((Data[16] >> 4) & 0x01, (uint8_t*)&VT13->Mouse.M_PressTime);

        // 键盘数据解析
        uint16_t kb = (uint16_t)(Data[17] | (Data[18] << 8));

        // 基础 WASD
        VT13->KeyBoard.W = VT_UpdateKeyStatus((kb >> 0) & 1, &VT13->KeyBoard.W_PressTime);
        VT13->KeyBoard.S = VT_UpdateKeyStatus((kb >> 1) & 1, &VT13->KeyBoard.S_PressTime);
        VT13->KeyBoard.A = VT_UpdateKeyStatus((kb >> 2) & 1, &VT13->KeyBoard.A_PressTime);
        VT13->KeyBoard.D = VT_UpdateKeyStatus((kb >> 3) & 1, &VT13->KeyBoard.D_PressTime);

        // 键盘功能键状态同步 (基础按下位)
        VT13->KeyBoard.Shift = (kb >> 4) & 1;
        VT13->KeyBoard.Ctrl  = (kb >> 5) & 1;
        VT13->KeyBoard.Q     = (kb >> 6) & 1;
        VT13->KeyBoard.E     = (kb >> 7) & 1;
        VT13->KeyBoard.R     = (kb >> 8) & 1;
        VT13->KeyBoard.F     = (kb >> 9) & 1;
        VT13->KeyBoard.G     = (kb >> 10) & 1;
        VT13->KeyBoard.Z     = (kb >> 11) & 1;
        VT13->KeyBoard.X     = (kb >> 12) & 1;
        VT13->KeyBoard.C     = (kb >> 13) & 1;
        VT13->KeyBoard.V     = (kb >> 14) & 1;
        VT13->KeyBoard.B     = (kb >> 15) & 1;

        // 键盘锁定切换逻辑 (按下瞬时取反)
        static uint8_t KeyLocks[12] = {0};
        VT_HandleKeyToggle(VT13->KeyBoard.Shift, &KeyLocks[0],  &VT13->KeyBoard.Shift_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.Ctrl,  &KeyLocks[1],  &VT13->KeyBoard.Ctrl_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.Q,     &KeyLocks[2],  &VT13->KeyBoard.Q_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.E,     &KeyLocks[3],  &VT13->KeyBoard.E_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.R,     &KeyLocks[4],  &VT13->KeyBoard.R_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.F,     &KeyLocks[5],  &VT13->KeyBoard.F_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.G,     &KeyLocks[6],  &VT13->KeyBoard.G_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.Z,     &KeyLocks[7],  &VT13->KeyBoard.Z_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.X,     &KeyLocks[8],  &VT13->KeyBoard.X_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.C,     &KeyLocks[9],  &VT13->KeyBoard.C_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.V,     &KeyLocks[10], &VT13->KeyBoard.V_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.B,     &KeyLocks[11], &VT13->KeyBoard.B_PreeNumber);
    }
    else
    {
        VT13->CRC_flag = false;
    }
}

/**
 * @brief 更新按键按压时间，并判断当前状态（点按/长按）
 */
static uint8_t VT_UpdateKeyStatus(uint8_t is_pressed, uint8_t *press_time)
{
    if (is_pressed)
    {
        if (*press_time < 250) (*press_time)++;
        return (*press_time <= 20) ? RUI_DF_KEY_CLICK : RUI_DF_KEY_PRESS;
    }
    else
    {
        *press_time = 0;
        return RUI_DF_KEY_UP;
    }
}

/**
 * @brief 处理按键切换锁定逻辑（按下瞬时改变一次状态）
 */
static void VT_HandleKeyToggle(uint8_t is_pressed, uint8_t *lock_flag, uint8_t *toggle_number)
{
    if (is_pressed && !(*lock_flag))
    {
        *toggle_number = !(*toggle_number);
        *lock_flag = 1;
    }
    else if (!is_pressed)
    {
        *lock_flag = 0;
    }
}

/**
 * @brief 鼠标一阶低通滤波
 */
static float VT_OneFilter(float last, float now, float thresholdValue)
{
    const float sensitivlFilter = 0.8f;
    const float numbFilter = 0.2f;
    if (MATH_ABS_float(last - now) >= thresholdValue)
        return (now * sensitivlFilter + last * (1.0f - sensitivlFilter));
    else
        return (now * numbFilter + last * (1.0f - numbFilter));
}

/**
 * @brief 浮点死区
 */
static float VT_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue) return 0.0f;
    return Value;
}

/**
 * @brief 计算 CRC16 校验和
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    while(len--)
    {
        crc16 = (crc16 >> 8) ^ crc16_tab[(crc16 ^ *p_msg++) & 0x00ff];
    }
    return crc16;
}

/**
 * @brief 验证数据帧 CRC16
 */
static bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    if (p_msg == NULL || len <= 2) return false;
    uint16_t w_expected = get_crc16_check_sum(p_msg, len - 2, 0xFFFF);
    return ((w_expected & 0xFF) == p_msg[len - 2] && (w_expected >> 8) == p_msg[len - 1]);
}