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

static float normalize_to_pi(float angle) {
    angle = fmodf(angle, 2.0f * PI);
    if (angle > PI)  angle -= 2.0f * PI;
    if (angle < -PI) angle += 2.0f * PI;
    return angle;
}

// ===================== 初始化函数（配置完全不变，新增状态清零） =====================
uint8_t Swerve_Init(Swerve_Cfg_t *cfg, Swerve_State_t *state) {
    cfg->m = 10.5f;
    cfg->J = 1.0f;
    cfg->R = 0.24f;
    cfg->r = 0.06f;
    cfg->gear_d = 15.76f;


    cfg->Swerve_offset[0] = (float)(-8.5f * 2*PI / 360.0f);
    cfg->Swerve_offset[1] = (float)(83.5f * 2*PI / 360.0f);
    cfg->Swerve_offset[2] = (float)(94.5f * 2*PI / 360.0f);
    cfg->Swerve_offset[3] = (float)(127.5f * 2*PI / 360.0f);

    cfg->drive_dir[0] = 1;  cfg->drive_dir[1] = -1;
    cfg->drive_dir[2] = 1;  cfg->drive_dir[3] = -1;

    cfg->phi[0] = 0.25f * PI;  cfg->phi[1] = 0.75f * PI;
    cfg->phi[2] = 1.25f * PI;  cfg->phi[3] = 1.75f * PI;

    if (state != NULL) {
        __builtin_memset(state, 0, sizeof(Swerve_State_t));
    }
    return 0;
}

void Swerve_Forward_Calc(Swerve_State_t *now, MOTOR_Typdef *motor, float gyro_vw, Swerve_Cfg_t *cfg) {
    float b_x = 0, b_y = 0;

    for (int i = 0; i < 4; i++) {
        now->wheel[i].v_wheel_now =
            (motor->DJI_3508_Chassis[i].DATA.Speed_now * RPM_TO_RADS / cfg->gear_d)
            * cfg->r;

        float theta_chassis = (motor->DJI_6020_Steer[i].DATA.Angle_now * ENCODER_TO_RAD) - cfg->Swerve_offset[i];
        now->wheel[i].theta_now = normalize_to_pi(theta_chassis);

        float vix = now->wheel[i].v_wheel_now * cosf(theta_chassis);
        float viy = now->wheel[i].v_wheel_now * sinf(theta_chassis);

        b_x += vix + gyro_vw * cfg->R * sinf(cfg->phi[i]);
        b_y += viy - gyro_vw * cfg->R * cosf(cfg->phi[i]);
    }

    now->vx = b_x / 4.0f;
    now->vy = b_y / 4.0f;
    now->vw = gyro_vw;
}

void Swerve_Inverse_Calc(float *ff_out, MOTOR_Typdef *motor,
                        float ax, float ay, float aw,
                        float vx, float vy, float vw, Swerve_Cfg_t *cfg, Swerve_State_t *state)
{
    state->ax_target = ax;
    state->ay_target = ay;
    state->aw_target = aw;
    state->vx_target = vx;
    state->vy_target = vy;
    state->vw_target = vw;

    for (int i = 0; i < 4; i++) {
        float vix = vx - cfg->R * vw * sinf(cfg->phi[i]) * cfg->drive_dir[i];
        float viy = vy + cfg->R * vw * cosf(cfg->phi[i]) * cfg->drive_dir[i];
        float v_mag = sqrtf(vix * vix + viy * viy);

        float current_theta_motor = motor->DJI_6020_Steer[i].DATA.Angle_Infinite * ENCODER_TO_RAD;
        float current_theta_chassis = current_theta_motor - cfg->Swerve_offset[i];

        float target_theta_raw;
        if (fabsf(v_mag) < 0.005f) {
            target_theta_raw = current_theta_chassis;
        } else {
            target_theta_raw = atan2f(viy, vix);
        }

        float diff = target_theta_raw - fmodf(current_theta_chassis, 2.0f * PI);
        while (diff >  PI) diff -= 2.0f * PI;
        while (diff < -PI) diff += 2.0f * PI;

        float speed_dir = 1.0f;
        if (fabsf(diff) > PI / 2.0f) {
            diff = (diff > 0) ? diff - PI : diff + PI;
            speed_dir = -1.0f;
        }

        state->wheel[i].theta_target = normalize_to_pi(current_theta_chassis + diff);
        state->wheel[i].v_wheel_target = speed_dir * v_mag;

        motor->DJI_6020_Steer[i].PID_P.Ref = current_theta_motor + diff;
        motor->DJI_3508_Chassis[i].PID_S.Ref = state->wheel[i].v_wheel_target / cfg->r * cfg->gear_d / RPM_TO_RADS;

        float F_ix = (cfg->m * ax - cfg->drive_dir[i] * (cfg->J * aw / cfg->R) * sinf(cfg->phi[i])) / 4.0f;
        float F_iy = (cfg->m * ay + cfg->drive_dir[i] * (cfg->J * aw / cfg->R) * cosf(cfg->phi[i])) / 4.0f;
        float F_drive = F_ix * cosf(current_theta_chassis) + F_iy * sinf(current_theta_chassis);
        state->wheel[i].ff_out = speed_dir * (F_drive * cfg->r) * M3508_NM_TO_RAW * cfg->drive_dir[i];
        ff_out[i] = state->wheel[i].ff_out;
    }
}