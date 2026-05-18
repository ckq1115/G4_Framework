//
// Created by CaoKangqi on 2026/5/13.
//
#include "Shoot_Task.h"

Feeder_t g_feeder = {0};

#define COUNTS_PER_SHOT 36864.0f*2.5f*8/9

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
    g_feeder.target_freq = 18.0f;
    uint32_t now = HAL_GetTick();
    float interval = 1000.0f / g_feeder.target_freq;

    // 3. 目标计数更新 (单发/连发统一限频)
    if ((DBUS.Remote.S1_u8 == 2 && last_s1 == 3) || (DBUS.Remote.S1_u8 == 1)) {
        if (now - last_shot_time >= interval) {
            g_feeder.target_pos_cnt--;
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

    if (smooth_ref > final_target) {
        smooth_ref -= step;
        if (smooth_ref < final_target) smooth_ref = final_target;
    }
    // 删除了 smooth_ref = final_target 的直接赋值，统一由 step 推进实现限速

    // 5. 电机控制
    All_Motor.DJI_2006_bo.PID_P.Ref = smooth_ref;
    PID_Calculate(&All_Motor.DJI_2006_bo.PID_P, All_Motor.DJI_2006_bo.DATA.Angle_Infinite, smooth_ref);
    PID_Calculate(&All_Motor.DJI_2006_bo.PID_S, All_Motor.DJI_2006_bo.DATA.Speed_now, All_Motor.DJI_2006_bo.PID_P.Output);

    DJI_Motor_Stuck_Check(&All_Motor.DJI_2006_bo, 6000, 100, 100, 500);
    DJI_Motor_Send(&hfdcan2, 0x200, 0, 0, All_Motor.DJI_2006_bo.PID_S.Output, 0);
}*/

/*void Ctrl_Shoot_Task() {
    static uint8_t last_s1 = 3;
    static uint32_t last_shot_time = 0;
    static bool is_init = false;

    if (!is_init) {
        g_feeder.target_pos_cnt = (int32_t)(All_Motor.DJI_2006_bo.DATA.Angle_Infinite / COUNTS_PER_SHOT);
        is_init = true;
    }

    g_feeder.target_freq = MATH_Limit_float(20.0f, 0.0f, 10.0f - DBUS.Remote.Dial_int16/30);
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
}*/

void Ctrl_Shoot_Task() {
    // 0. 安全守护：若 S2 处于非合法档位，断电停机，防止飞车
    if (DBUS.Remote.S2_u8 != 1 && DBUS.Remote.S2_u8 != 2) {
        DJI_Motor_Send(&hfdcan2, 0x200, 0, 0, 0, 0);
        return;
    }

    // 【一键方向切换开关】 1 默认方向，-1 整体反转方向
    // 改变这个符号，模式一和模式二的正常转动方向会同时改变
    const int32_t dir_sign = -1;

    static uint8_t last_s1 = 3;
    static uint32_t last_shot_time = 0;
    static float smooth_ref = 0.0f;
    static bool is_init = false;

    // 1. 统一初始化
    if (!is_init) {
        smooth_ref = All_Motor.DJI_2006_bo.DATA.Angle_Infinite;
        g_feeder.target_pos_cnt = (int32_t)(smooth_ref / COUNTS_PER_SHOT);
        is_init = true;
    }

    // 2. 状态机分流：配置不同模式的“控制特性”
    float target_freq = 0.0f;
    bool use_smoothing = false;

    // 两个模式的正常转动方向完全相同，统一使用 dir_sign
    int32_t dir = 1 * dir_sign;

    if (DBUS.Remote.S2_u8 == 1) {
        float raw_freq = 18.0f - (float)DBUS.Remote.Dial_int16 / 20.0f;
        target_freq = MATH_Limit_float(25.0f, 0.0f, raw_freq);
        use_smoothing = true;   // 模式一：开启平滑
    }
    else if (DBUS.Remote.S2_u8 == 2) {
        float raw_freq = 10.0f - (float)DBUS.Remote.Dial_int16 / 30.0f;
        target_freq = MATH_Limit_float(25.0f, 0.0f, raw_freq);
        use_smoothing = false;  // 模式二：关闭平滑
    }

    // 3. 统一频率限幅与时间判定
    uint32_t now = HAL_GetTick();
    float interval = 1000.0f / target_freq;

    // 4. 统一正常射击触发判定
    if ((DBUS.Remote.S1_u8 == 2 && last_s1 == 3) || (DBUS.Remote.S1_u8 == 1)) {
        if (now - last_shot_time >= (uint32_t)interval) {
            g_feeder.target_pos_cnt += dir;
            last_shot_time = now;
        }
    }
    last_s1 = DBUS.Remote.S1_u8;

    // 5. 【核心变更】分模式处理卡弹（堵转）逻辑
    if (All_Motor.DJI_2006_bo.DATA.Stuck_Flag[1] == 1)
    {
        All_Motor.DJI_2006_bo.PID_S.Output = 0.0f;
        All_Motor.DJI_2006_bo.PID_S.Iout = 0.0f;
        float current_exact_pop = All_Motor.DJI_2006_bo.DATA.Angle_Infinite / COUNTS_PER_SHOT;

        if (dir > 0) {
            // 正转时卡弹（例如10.1）：必须去 11，使用 ceilf 向上取整
            g_feeder.target_pos_cnt = (int32_t)ceilf(current_exact_pop);

            // 如果正好卡在整数点（概率极低），强制向前再推一发，避免目标没有更新
            if ((float)g_feeder.target_pos_cnt - current_exact_pop < 0.01f) {
                g_feeder.target_pos_cnt += 1;
            }
        } else {
            // 反转时卡弹（例如 -10.1）：必须去 -11，使用 floorf 向下取整
            g_feeder.target_pos_cnt = (int32_t)floorf(current_exact_pop);

            // 如果正好卡在整数点，强制向前（负方向）再推一发
            if (current_exact_pop - (float)g_feeder.target_pos_cnt < 0.01f) {
                g_feeder.target_pos_cnt -= 1;
            }
        }

        // 让平滑参考值立刻对齐当前实际位置，这样下一帧它会顺着‘前进方向’平滑移动到新的整弹点
        smooth_ref = All_Motor.DJI_2006_bo.DATA.Angle_Infinite;
        // 处理完毕，手动清除堵转标志位
        All_Motor.DJI_2006_bo.DATA.Stuck_Flag[1] = 0;
    }

    // 6. 统一目标计算与运动平滑控制 (Ramp 阶跃生成器)
    float final_target = (float)g_feeder.target_pos_cnt * COUNTS_PER_SHOT;

    if (use_smoothing) {
        float step = (target_freq * COUNTS_PER_SHOT) / 1000.0f;

        if (smooth_ref > final_target) {
            smooth_ref -= step;
            if (smooth_ref < final_target) smooth_ref = final_target;
        } else if (smooth_ref < final_target) {
            smooth_ref += step;
            if (smooth_ref > final_target) smooth_ref = final_target;
        }
    } else {
        // 模式二关闭平滑，目标值直接阶跃过去，爆发力最强
        smooth_ref = final_target;
    }

    // 7. 统一底层电机控制与 CAN 发送
    All_Motor.DJI_2006_bo.PID_P.Ref = smooth_ref;
    PID_Calculate(&All_Motor.DJI_2006_bo.PID_P, All_Motor.DJI_2006_bo.DATA.Angle_Infinite, smooth_ref);
    PID_Calculate(&All_Motor.DJI_2006_bo.PID_S, All_Motor.DJI_2006_bo.DATA.Speed_now, All_Motor.DJI_2006_bo.PID_P.Output);

    // 该函数在内部判定是否堵转并更新 Stuck_Flag[1]
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