//
// Created by CaoKangqi on 2026/5/13.
//
#include "Shoot_Task.h"

Feeder_t g_feeder = {0};

#define COUNTS_PER_SHOT 36864.0f

void Shoot_Control_Init() {
    float PID_P_SHOOT[3] = {0.23f,0.0f,0.0f};
    float PID_S_SHOOT[3] = {15.0f,0.01f,0.0f};

    PID_Init(&All_Motor.DJI_2006_bo.PID_P,20000,4000,
        PID_P_SHOOT,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器
    PID_Init(&All_Motor.DJI_2006_bo.PID_S,10000,2000,
        PID_S_SHOOT,0,0,
        0,0,0,
        Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
        //梯形积分,变速积分
        );//微分先行,微分滤波器
}

/*void Ctrl_Shoot_Task() {
    static uint8_t last_s1 = 3;
    static float smooth_ref = 0.0f;
    static uint32_t last_shot_time = 0;
    static bool is_init = false;

    // 1. 初始化
    if (!is_init) {
        smooth_ref = All_Motor.DJI_2006_bo.DATA.Angle_Infinite;
        g_feeder.target_pos_cnt = (int32_t)(smooth_ref / COUNTS_PER_SHOT);
        is_init = true;
    }

    // 2. 频率限幅与判定间隔
    g_feeder.target_freq = MATH_Limit_float(30.0f, 0.1f, g_feeder.target_freq);
    uint32_t now = HAL_GetTick();
    float interval = 1000.0f / g_feeder.target_freq;

    // 3. 目标计数更新 (单发/连发统一限频)
    if ((DBUS.Remote.S1_u8 == 2 && last_s1 == 3) || (DBUS.Remote.S1_u8 == 1)) {
        if (now - last_shot_time >= interval) {
            g_feeder.target_pos_cnt++;
            last_shot_time = now;
        }
    }
    last_s1 = DBUS.Remote.S1_u8;

    // 4. 限速阶跃逻辑
    float final_target = (float)g_feeder.target_pos_cnt * COUNTS_PER_SHOT;

    // 定义一个恒定的拨动速度 (例如按照 20Hz 的速度去转动每一发)
    // 这样即使在 1Hz 频率下，也是快转 50ms，停 950ms，而不是慢吞吞转 1000ms
    const float shoot_speed_hz = 20.0f;
    float step = (shoot_speed_hz * COUNTS_PER_SHOT) / 1000.0f;

    if (smooth_ref < final_target) {
        smooth_ref += step;
        if (smooth_ref > final_target) smooth_ref = final_target;
    }
    // 删除了 smooth_ref = final_target 的直接赋值，统一由 step 推进实现限速

    // 5. 电机控制
    All_Motor.DJI_2006_bo.PID_P.Ref = smooth_ref;
    PID_Calculate(&All_Motor.DJI_2006_bo.PID_P, All_Motor.DJI_2006_bo.DATA.Angle_Infinite, smooth_ref);
    PID_Calculate(&All_Motor.DJI_2006_bo.PID_S, All_Motor.DJI_2006_bo.DATA.Speed_now, All_Motor.DJI_2006_bo.PID_P.Output);

    DJI_Motor_Stuck_Check(&All_Motor.DJI_2006_bo, 6000, 100, 100, 500);
    DJI_Motor_Send(&hfdcan2, 0x200, 0, 0, All_Motor.DJI_2006_bo.PID_S.Output, 0);
}*/

void Ctrl_Shoot_Task() {
    static uint8_t last_s1 = 3;
    static uint32_t last_shot_time = 0;
    static bool is_init = false;

    if (!is_init) {
        g_feeder.target_pos_cnt = (int32_t)(All_Motor.DJI_2006_bo.DATA.Angle_Infinite / COUNTS_PER_SHOT);
        is_init = true;
    }

    g_feeder.target_freq = MATH_Limit_float(20.0f, 0.0f, 5.0f - DBUS.Remote.Dial_int16/30);
    uint32_t now = HAL_GetTick();
    float interval = 1000.0f / g_feeder.target_freq;

    if ((DBUS.Remote.S1_u8 == 2 && last_s1 == 3) || (DBUS.Remote.S1_u8 == 1)) {
        if (now - last_shot_time >= (uint32_t)interval) {
            g_feeder.target_pos_cnt++;
            last_shot_time = now;
        }
    }
    last_s1 = DBUS.Remote.S1_u8;

    if (All_Motor.DJI_2006_bo.DATA.Stuck_Flag[1] == 1) {
        g_feeder.target_pos_cnt--;
        All_Motor.DJI_2006_bo.DATA.Stuck_Flag[1] = 0;
    }
    All_Motor.DJI_2006_bo.PID_P.Ref = (float)g_feeder.target_pos_cnt * COUNTS_PER_SHOT;
    PID_Calculate(&All_Motor.DJI_2006_bo.PID_P, All_Motor.DJI_2006_bo.DATA.Angle_Infinite, All_Motor.DJI_2006_bo.PID_P.Ref);
    PID_Calculate(&All_Motor.DJI_2006_bo.PID_S, All_Motor.DJI_2006_bo.DATA.Speed_now, All_Motor.DJI_2006_bo.PID_P.Output);

    DJI_Motor_Stuck_Check(&All_Motor.DJI_2006_bo, 6000, 100, 100, 500);
    DJI_Motor_Send(&hfdcan2, 0x200, 0, 0, All_Motor.DJI_2006_bo.PID_S.Output, 0);
}

ShootDet_t g_det = {0};

bool Update_Shoot_Det(float speed1, float speed2, ShootDet_t *det) {
    det->last_cnt = det->cnt;
    float val = (fabsf(speed1) + fabsf(speed2)) / 2.0f;
    if (!det->init) {
        det->base = val;
        det->last_val = val;
        det->max_drop_in_round = 0;
        det->cool_down_cnt = 0;
        det->init = true;
        return false;
    }
    float slope = det->last_val - val;
    det->last_val = val;
    if (val > det->base) {
        det->base = (K_UP * val) + (1.0f - K_UP) * det->base;
    } else {
        det->base = (K_DN * val) + (1.0f - K_DN) * det->base;
    }
    float drop = det->base - val;
    bool shoot_done = false;
    if (det->cool_down_cnt > 0) {
        det->cool_down_cnt--;
        det->armed = false;
        return false;
    }
    if (!det->armed) {
        if (drop > TH_FIRE && drop < TH_FIRE_MAX && slope > MIN_SLOPE && val > 4500) {
            det->armed = true;
            det->max_drop_in_round = drop;
            det->t_out = 0;
        }
    } else {
        det->t_out++;
        if (drop > det->max_drop_in_round) {
            det->max_drop_in_round = drop;
        }
        bool condition_relative = (drop < det->max_drop_in_round * (1.0f - RELATIVE_RECOVER));
        bool condition_absolute = (drop < TH_RST_SAFE);
        if (condition_relative || condition_absolute) {
            det->armed = false;
            det->cnt++;
            det->cool_down_cnt = COOL_DOWN_TICKS;
            det->max_drop_in_round = 0;
            shoot_done = true;
        }
        else if (det->t_out >= TIMEOUT_TICKS) {
            det->armed = false;
            det->max_drop_in_round = 0;
        }
    }
    return shoot_done;
}

void Heat_Freq_Ctrl(float kp, User_Data_T *user_data, Feeder_t *feeder, ShootDet_t *det, float speed1, float speed2) {
    static float heat = 0.0f;

    if (Update_Shoot_Det(speed1, speed2, det)) {
        heat += 10.0f;
    }

    float max_heat = (float)user_data->robot_status.shooter_barrel_heat_limit;
    heat = MATH_Limit_float(max_heat, 0.0f, heat);

    heat -= user_data->robot_status.shooter_barrel_cooling_value * 0.005f;
    if (heat < user_data->power_heat_data.shooter_17mm_barrel_heat) {
        heat = (float)user_data->power_heat_data.shooter_17mm_barrel_heat;
    }
    feeder->target_freq = kp * (max_heat - heat);
    feeder->target_freq = MATH_Limit_float(20.0f, 0.0f, feeder->target_freq);
}