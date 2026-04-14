//
// Created by CaoKangqi on 2026/2/23.
//
#include "Chassis_Calc.h"
#include <math.h>
#include "All_define.h"
#include "CKQ_MATH.h"

static float ClampFloat(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static float AbsFloat(float val)
{
    return (val < 0.0f) ? -val : val;
}

static float WrapToPi(float angle)
{
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

uint8_t MecanumInit(mecanumInit_typdef *mecanumInitT)
{
    /*初始化参数*/
    mecanumInitT->deceleration_ratio = 3591/187; // 减速比1/19
    mecanumInitT->max_vw_speed       = 50000;     // r方向上的最大速度单位：毫米/秒
    mecanumInitT->max_vx_speed       = 50000;     // x方向上的最大速度单位：毫米/秒
    mecanumInitT->max_vy_speed       = 50000;     // y方向上的最大速度单位：毫米/秒
    mecanumInitT->max_wheel_ramp     = 8000;      // 3508最大转速不包含限速箱
    mecanumInitT->rotate_x_offset    = 00.0f;     // 云台在x轴的偏移量  mm
    mecanumInitT->rotate_y_offset    = 00.0f;     // 云台在y轴的偏移量  mm
    mecanumInitT->wheelbase          = 300;       // 轮距	mm
    mecanumInitT->wheeltrack         = 300;       // 轴距	mm
    mecanumInitT->wheel_perimeter    = 478;       // 轮子的周长(mm)

    /*计算旋转比率*/
    mecanumInitT->raid_fr = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f -
                             mecanumInitT->rotate_x_offset + mecanumInitT->rotate_y_offset) /
                            57.3f;
    mecanumInitT->raid_fl = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f -
                             mecanumInitT->rotate_x_offset - mecanumInitT->rotate_y_offset) /
                            57.3f;
    mecanumInitT->raid_bl = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f +
                             mecanumInitT->rotate_x_offset - mecanumInitT->rotate_y_offset) /
                            57.3f;
    mecanumInitT->raid_br = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f +
                             mecanumInitT->rotate_x_offset + mecanumInitT->rotate_y_offset) /
                            57.3f;
// 将算出来的数据转化到转每分钟上去 raid = 60/(电机减速比*轮的周长)
    mecanumInitT->wheel_rpm_ratio = 60.0f / (mecanumInitT->wheel_perimeter * mecanumInitT->deceleration_ratio);

    return 0;
}

void MecanumResolve(float *wheel_rpm, float vx_temp, float vy_temp, float vr, mecanumInit_typdef *mecanumInit_t)
{
    float vx = ClampFloat(vx_temp, -mecanumInit_t->max_vx_speed, mecanumInit_t->max_vx_speed);
    float vy = ClampFloat(vy_temp, -mecanumInit_t->max_vy_speed, mecanumInit_t->max_vy_speed);
    float vw = ClampFloat(vr, -mecanumInit_t->max_vw_speed, mecanumInit_t->max_vw_speed);

    wheel_rpm[0] = (-vx + vy + vw * mecanumInit_t->raid_fr) * mecanumInit_t->wheel_rpm_ratio;
    wheel_rpm[1] = ( vx + vy + vw * mecanumInit_t->raid_fl) * mecanumInit_t->wheel_rpm_ratio;
    wheel_rpm[2] = ( vx - vy + vw * mecanumInit_t->raid_bl) * mecanumInit_t->wheel_rpm_ratio;
    wheel_rpm[3] = (-vx - vy + vw * mecanumInit_t->raid_br) * mecanumInit_t->wheel_rpm_ratio;

    float max_abs = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
        float a = AbsFloat(wheel_rpm[i]);
        if (a > max_abs) max_abs = a;
    }
    if (max_abs > (float)mecanumInit_t->max_wheel_ramp && max_abs > 0.0f) {
        float rate = (float)mecanumInit_t->max_wheel_ramp / max_abs;
        for (uint8_t i = 0; i < 4; i++) {
            wheel_rpm[i] *= rate;
        }
    }
}

uint8_t OmniInit(OmniInit_typdef *OmniInitT)
{
    OmniInitT->wheel_perimeter = 155*PI;// 轮子的周长(mm)
    OmniInitT->CHASSIS_DECELE_RATIO = 3591/187;
    OmniInitT->LENGTH_A = 180;
    OmniInitT->LENGTH_B = 180;
    OmniInitT->max_vx_speed = 50000;
    OmniInitT->max_vy_speed = 50000;
    OmniInitT->max_vw_speed = 50000;
    OmniInitT->max_wheel_ramp = 8000;

    OmniInitT->rotate_radius = (OmniInitT->LENGTH_A + OmniInitT->LENGTH_B) / 57.3f;
    OmniInitT->wheel_rpm_ratio = 60.0f / (OmniInitT->wheel_perimeter) * OmniInitT->CHASSIS_DECELE_RATIO;
    return 0;
}

/* 计算每个轮子的转速
 * @param wheel_rpm 输出参数，长度为4的数组，分别对应4个轮子的转速(rpm)
 * @param vx_temp 期望的x轴速度(mm/s)
 * @param vy_temp 期望的y轴速度(mm/s)
 * @param vr 期望的自转速度(deg/s)
 * @param OmniInit_t 指向已初始化的OmniInit_typdef结构体的指针，包含底盘参数和限速设置
 * @return void
 */
void Omni_calc(float *wheel_rpm, float vx_temp, float vy_temp, float vr, OmniInit_typdef *OmniInit_t)
{
    float vx = ClampFloat(vx_temp, -OmniInit_t->max_vx_speed, OmniInit_t->max_vx_speed);
    float vy = ClampFloat(vy_temp, -OmniInit_t->max_vy_speed, OmniInit_t->max_vy_speed);
    float vw = ClampFloat(vr, -OmniInit_t->max_vw_speed, OmniInit_t->max_vw_speed);
    float rot = vw * OmniInit_t->rotate_radius;// 将角速度转换为线速度，单位mm/s

    wheel_rpm[0] = (  vx + vy + rot) * OmniInit_t->wheel_rpm_ratio;//left//x，y方向速度,w底盘转动速度
    wheel_rpm[1] = (  vx - vy + rot) * OmniInit_t->wheel_rpm_ratio;//forward
    wheel_rpm[2] = ( -vx - vy + rot) * OmniInit_t->wheel_rpm_ratio;//right
    wheel_rpm[3] = ( -vx + vy + rot) * OmniInit_t->wheel_rpm_ratio;//back

    float max_abs = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
        float a = AbsFloat(wheel_rpm[i]);
        if (a > max_abs) max_abs = a;
    }
    if (max_abs > (float)OmniInit_t->max_wheel_ramp && max_abs > 0.0f) {
        float rate = (float)OmniInit_t->max_wheel_ramp / max_abs;
        for (uint8_t i = 0; i < 4; i++) {
            wheel_rpm[i] *= rate;
        }
    }
}

#define M3508_NM_TO_RAW ( (1.0f / (268/17 * 0.3f * 0.85f)) * (16384.0f / 20.0f) )

uint8_t Steer_Init(Steer_Cfg_t *cfg) {
    cfg->m = 22.5f;       // 整车质量 (kg)
    cfg->J = 0.85f;       // 绕中心转动惯量 (kg·m²)
    cfg->R = 0.247f;      // 旋转半径 (中心到轮心距离 m)
    cfg->r = 0.06f;      // 轮半径 (m)
    cfg->gear_d = 268/17;  // 驱动减速比

    // 轮角排布 (RAD)
    cfg->phi[0] = 0.25f * PI;  cfg->phi[1] = 0.75f * PI;
    cfg->phi[2] = 1.25f * PI;  cfg->phi[3] = 1.75f * PI;
    return 0;
}

void Steer_Forward_Calc(Steer_State_t *now, MOTOR_Typdef *motor, float gyro_vw, Steer_Cfg_t *cfg) {
    float b_x = 0, b_y = 0;

    for (int i = 0; i < 4; i++) {
        float v_w_mag = (motor->DJI_3508_Chassis[i].DATA.Speed_now * RPM_TO_RADS / cfg->gear_d) * cfg->r;

        float theta = motor->DJI_6020_Steer[i].DATA.Angle_now * ENCODER_TO_RAD;

        float vix = v_w_mag * cosf(theta);
        float viy = v_w_mag * sinf(theta);

        b_x += (vix + gyro_vw * cfg->R * sinf(cfg->phi[i]));
        b_y += (viy - gyro_vw * cfg->R * cosf(cfg->phi[i]));
    }

    now->vx = b_x / 4.0f;
    now->vy = b_y / 4.0f;
    now->vw = gyro_vw;
}

void Steer_Inverse_Calc(float *ff_out, MOTOR_Typdef *motor,
                        float ax, float ay, float aw,
                        float vx, float vy, float vw, Steer_Cfg_t *cfg)
{
    for (int i = 0; i < 4; i++) {
        // 运动学解算：计算轮中心的目标线速度向量
        float vix = vx - cfg->R * vw * sinf(cfg->phi[i]);
        float viy = vy + cfg->R * vw * cosf(cfg->phi[i]);
        float v_mag = sqrtf(vix * vix + viy * viy);

        // 获取当前电机角度反馈（弧度）
        float current_theta = motor->DJI_6020_Steer[i].DATA.Angle_now * ENCODER_TO_RAD;

        // 舵向角目标计算
        float target_theta;
        // 如果线速度目标极小，则切换到加速度向量引导方向,这样可以解决起步时舵轮方向不明确导致的抖动问题
        if (v_mag < 0.005f) {
            float aix = ax - cfg->R * aw * sinf(cfg->phi[i]);
            float aiy = ay + cfg->R * aw * cosf(cfg->phi[i]);
            // 如果加速度也为0，保持当前角度；否则指向加速度方向
            target_theta = (sqrtf(aix * aix + aiy * aiy) < 0.005f) ? current_theta : atan2f(aiy, aix);
        } else {
            target_theta = atan2f(viy, vix);
        }

        // 就近转向逻辑
        float diff = target_theta - current_theta;
        while (diff > PI)  diff -= 2.0f * PI;
        while (diff < -PI) diff += 2.0f * PI;

        float speed_dir = 1.0f;
        if (fabsf(diff) > PI / 2.0f) {
            target_theta = (diff > 0) ? target_theta - PI : target_theta + PI;
            speed_dir = -1.0f;
        }

        motor->DJI_6020_Steer[i].DATA.Aim = target_theta;
        motor->DJI_3508_Chassis[i].DATA.Aim = speed_dir * v_mag / cfg->r * cfg->gear_d / RPM_TO_RADS;

        // 动力学前馈优化
        // 基于拉格朗日乘数法求得的解析解：在保证底盘合力的前提下，使四个驱动轮的负载最均衡
        // F_ix 和 F_iy 是地面坐标系下该轮应提供的最优力向量
        float F_ix = (cfg->m * ax - (cfg->J * aw / cfg->R) * sinf(cfg->phi[i])) / 4.0f;
        float F_iy = (cfg->m * ay + (cfg->J * aw / cfg->R) * cosf(cfg->phi[i])) / 4.0f;

        // 将最优力向量投影到轮子“当前”实际的角度上
        // 核心思路：即便舵机还没转到位，驱动电机也应该按照当前角度能贡献的分量来输出，从而实现平滑过渡
        float F_drive = F_ix * cosf(current_theta) + F_iy * sinf(current_theta);

        // 换算为电流原始控制值并存入输出数组
        ff_out[i] = speed_dir * (F_drive * cfg->r) * M3508_NM_TO_RAW;
    }
}