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

#define M3508_NM_TO_RAW ( (1.0f / (15.76f * 0.0157f * 0.85f)) * (16384.0f / 20.0f) )

uint8_t Swerve_Init(Swerve_Cfg_t *cfg) {
    cfg->m = 22.5f;       // 整车质量 (kg)
    cfg->J = 0.5f;       // 绕中心转动惯量 (kg·m²)
    cfg->R = 0.24f;      // 旋转半径 (中心到轮心距离 m)
    cfg->r = 0.06f;      // 轮半径 (m)
    cfg->gear_d = 15.76f;  // 驱动减速比

    cfg->Swerve_offset[0] =(float)(-7.5 * 2*PI / 360.0f);
    cfg->Swerve_offset[1] =(float)(82.5 * 2*PI / 360.0f);
    cfg->Swerve_offset[2] =(float)(97.5 * 2*PI / 360.0f);
    cfg->Swerve_offset[3] =(float)(127.5 * 2*PI / 360.0f);

    cfg->drive_dir[0] = 1;
    cfg->drive_dir[1] = -1;
    cfg->drive_dir[2] = 1;
    cfg->drive_dir[3] = -1;

    // 轮角排布 (RAD)
    cfg->phi[0] = 0.25f * PI;  cfg->phi[1] = 0.75f * PI;
    cfg->phi[2] = 1.25f * PI;  cfg->phi[3] = 1.75f * PI;
    return 0;
}

void Swerve_Forward_Calc(Swerve_State_t *now, MOTOR_Typdef *motor, float gyro_vw, Swerve_Cfg_t *cfg) {
    float b_x = 0, b_y = 0;

    for (int i = 0; i < 4; i++) {
        float v_w_mag =(motor->DJI_3508_Chassis[i].DATA.Speed_now * RPM_TO_RADS / cfg->gear_d) * cfg->r * cfg->drive_dir[i];///////////////
        // 补偿零点偏角，将电机角度转为底盘坐标系角度
        float theta = (motor->DJI_6020_Steer[i].DATA.Angle_now * ENCODER_TO_RAD) - cfg->Swerve_offset[i];

        float vix = v_w_mag * cosf(theta);
        float viy = v_w_mag * sinf(theta);

        b_x += (vix + cfg->drive_dir[i] * gyro_vw * cfg->R * sinf(cfg->phi[i]));///////////////
        b_y += (viy - cfg->drive_dir[i] * gyro_vw * cfg->R * cosf(cfg->phi[i]));///////////////
    }

    now->vx = b_x / 4.0f;
    now->vy = b_y / 4.0f;
    now->vw = gyro_vw;
}

void Swerve_Inverse_Calc(float *ff_out, MOTOR_Typdef *motor,
                        float ax, float ay, float aw,
                        float vx, float vy, float vw, Swerve_Cfg_t *cfg)
{
    for (int i = 0; i < 4; i++) {
        // 运动学解算：计算轮中心的目标线速度向量
        float vix = vx - cfg->drive_dir[i] * cfg->R * vw * sinf(cfg->phi[i]);
        float viy = vy + cfg->drive_dir[i] * cfg->R * vw * cosf(cfg->phi[i]);
        float v_mag = sqrtf(vix * vix + viy * viy);
        // 获取当前电机连续编码器反馈并转换为弧度
        float current_theta_motor = motor->DJI_6020_Steer[i].DATA.Angle_Infinite * ENCODER_TO_RAD;
        // 补偿零点偏角：得到底盘坐标系下的当前角度
        float current_theta_chassis = current_theta_motor - cfg->Swerve_offset[i];
        // 舵向角目标计算
        float target_theta_raw;
        // 如果线速度目标极小，则切换到加速度向量引导方向,这样可以解决起步时舵轮方向不明确导致的抖动问题
        if (fabs(v_mag) < 0.005f) {
            target_theta_raw = current_theta_chassis;
        } else {
            target_theta_raw = atan2f(viy, vix);
        }
        // 就近转向逻辑
        // 计算目标角度与当前连续角度的相位差，使用 fmodf 提取当前位置在单一圈内的相位，以便与 atan2f 的结果 [-PI, PI] 进行对比
        float diff = target_theta_raw - fmodf(current_theta_chassis, 2.0f * PI);
        // 标准化 diff 到 [-PI, PI] 之间
        while (diff >  PI) diff -= 2.0f * PI;
        while (diff < -PI) diff += 2.0f * PI;

        float speed_dir = 1.0f;
        // 优弧劣弧判定：如果旋转超过 90 度，则选择反向旋转 180 度走更近的路径
        if (fabsf(diff) > PI / 2.0f) {
            diff = (diff > 0) ? diff - PI : diff + PI;
            speed_dir = -1.0f;
        }
        //得到舵向角度目标和轮向速度目标
        motor->DJI_6020_Steer[i].PID_P.Ref = current_theta_motor + diff;
        motor->DJI_3508_Chassis[i].PID_S.Ref = speed_dir * v_mag / cfg->r * cfg->gear_d / RPM_TO_RADS;
        // 动力学前馈优化
        // 基于拉格朗日乘数法求得的解析解：在保证底盘合力的前提下，使四个驱动轮的负载最均衡
        // F_ix 和 F_iy 是地面坐标系下该轮应提供的最优力向量
        float F_ix = (cfg->m * ax - cfg->drive_dir[i] * (cfg->J * aw / cfg->R) * sinf(cfg->phi[i])) / 4.0f;///////////////
        float F_iy = (cfg->m * ay + cfg->drive_dir[i] * (cfg->J * aw / cfg->R) * cosf(cfg->phi[i])) / 4.0f;///////////////
        // 将最优力向量投影到轮子当前实际角度上
        // 注意：此处投影必须使用底盘坐标系下的角度 current_theta_chassis
        float F_drive = F_ix * cosf(current_theta_chassis) + F_iy * sinf(current_theta_chassis);
        // 换算为电流原始控制值并存入输出数组
        ff_out[i] = speed_dir * (F_drive * cfg->r) * M3508_NM_TO_RAW;
    }
}