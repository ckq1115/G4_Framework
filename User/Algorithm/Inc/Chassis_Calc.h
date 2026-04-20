//
// Created by CaoKangqi on 2026/2/23.
//

#ifndef G4_FRAMEWORK_CHASSIS_CALC_H
#define G4_FRAMEWORK_CHASSIS_CALC_H
#include <stdint.h>

#include "All_Motor.h"

typedef struct
{
    float wheel_perimeter;    /* 轮的周长（mm）*/
    float wheeltrack;         /* 轮距（mm）*/
    float wheelbase;          /* 轴距（mm）*/
    float rotate_x_offset;    /* 相对于底盘中心的x轴旋转偏移量(mm) */
    float rotate_y_offset;    /* 相对于底盘中心的y轴旋转偏移量(mm) */
    float deceleration_ratio; /* 电机减速比 */
    int max_vx_speed;         /* 底盘的x轴的最大速度(mm/s) */
    int max_vy_speed;         /* 底盘的y轴的最大速度(mm/s) */
    int max_vw_speed;         /* 底盘的自转的最大速度(degree/s) */
    int max_wheel_ramp;       /* 3508最大转速 */
    // 每一个轮子的旋转比率//与旋转中心点相关
    float raid_fr;            // 右前
    float raid_fl;            // 左前
    float raid_bl;            // 左后
    float raid_br;            // 右后
    float wheel_rpm_ratio;    // 用来将速度转化成转每分钟
} mecanumInit_typdef;

typedef struct
{
    float wheel_perimeter;    /* 轮的周长（mm）*/
    float CHASSIS_DECELE_RATIO; /* 电机减速比 */
    float LENGTH_A;         /* 底盘长的一半（mm）*/
    float LENGTH_B;          /* 底盘宽的一半（mm）*/
    float rotate_radius;     /* (A+B)/57.3, 将角速度(deg/s)转换为线速度(mm/s) */
    float wheel_rpm_ratio;   /* 线速度(mm/s) -> 轮速(rpm) */
    int max_vx_speed;        /* 底盘x轴最大速度(mm/s) */
    int max_vy_speed;        /* 底盘y轴最大速度(mm/s) */
    int max_vw_speed;        /* 底盘自转最大速度(deg/s) */
    int max_wheel_ramp;      /* 3508最大转速 */
} OmniInit_typdef;

extern OmniInit_typdef OmniInit_t;

uint8_t MecanumInit(mecanumInit_typdef *mecanumInitT);
void MecanumResolve(float *wheel_rpm, float vx_temp, float vy_temp, float vr, mecanumInit_typdef *mecanumInit_t);

uint8_t OmniInit(OmniInit_typdef *OmniInitT);
void Omni_calc(float *wheel_rpm, float vx_temp, float vy_temp, float vr, OmniInit_typdef *OmniInit_t);

// 舵轮物理常量
typedef struct {
    float m;         // 底盘质量 (kg)
    float J;         // 转动惯量 (kg*m^2)
    float R;         // 旋转半径 (m)
    float r;         // 轮子半径 (m)
    float Swerve_offset[4]; // 舵轮零点偏角
    float drive_dir[4]; // 轮子正转方向 (1或-1)
    float phi[4];    // 轮子安装方位角 (rad)
    float gear_d;    // 驱动电机减速比 (M3508通常19:1)
} Swerve_Cfg_t;

// 实时物理状态
typedef struct {
    float theta_now;       // 归一化±π的舵轮实际角度 (rad)
    float theta_target;    // 归一化±π的舵轮目标角度 (rad)
    float v_wheel_now;     // 单轮实际线速度 (m/s)
    float v_wheel_target;  // 单轮目标线速度 (m/s)
    float ff_out;          // 单轮前馈输出原始值
} Swerve_Wheel_Debug_t;

typedef struct {
    // 底盘核心速度
    float vx;        // 底盘x轴实际速度 (m/s)
    float vy;        // 底盘y轴实际速度 (m/s)
    float vw;        // 底盘实际角速度 (rad/s)
    // 底盘目标指令
    float vx_target; // 底盘x轴目标速度 (m/s)
    float vy_target; // 底盘y轴目标速度 (m/s)
    float vw_target; // 底盘目标角速度 (rad/s)
    float ax_target; // 底盘x轴目标加速度 (m/s²)
    float ay_target; // 底盘y轴目标加速度 (m/s²)
    float aw_target; // 底盘目标角加速度 (rad/s²)
    Swerve_Wheel_Debug_t wheel[4];
} Swerve_State_t;

extern Swerve_Cfg_t S_Cfg;
extern Swerve_State_t S_Now;

uint8_t Swerve_Init(Swerve_Cfg_t *cfg, Swerve_State_t *state);

/**
 * @brief 舵轮正解算 (Forward Kinematics)
 * @param now 输出物理状态
 * @param motor 输入电机结构体指针
 * @param gyro_vw IMU角速度反馈
 */
void Swerve_Forward_Calc(Swerve_State_t *now, MOTOR_Typdef *motor, float gyro_vw, Swerve_Cfg_t *cfg);

/**
 * @brief 舵轮力控逆解算 (Inverse Kinematics with Feedforward)
 * @param motor 输出目标到电机结构体
 * @param ax/ay/aw 输入目标加速度 (由外环PID产生)
 * @param vx/vy/vw 输入目标速度
 */
void Swerve_Inverse_Calc(float *ff_out, MOTOR_Typdef *motor,
                        float ax, float ay, float aw,
                        float vx, float vy, float vw, Swerve_Cfg_t *cfg, Swerve_State_t *state);

#endif //G4_FRAMEWORK_CHASSIS_CALC_H