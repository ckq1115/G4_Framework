#ifndef __REFEREE_H
#define __REFEREE_H

#include "usart.h"
#include "string.h"
#include "stdint.h"
#include "CRC_DJI.h"

/* 帧长度 */
#define REFEREE_RXFRAME_LENGTH 136 //frame_header 5bytes , cmd_id 2bytes , data_max 127bytes , crc16 2bytes = 136bytes
#define FrameHeader_Length 5U /*!< the length of frame header */
#define CMDID_Length 2U /*!< the length of CMD ID */
#define CRC16_Length 2U /*!< the length of CRC ID */

/* ==================== CMD ID ==================== */
enum Read_Cmd_ID_Typdef
{
    game_state      = 0x0001,
    Match_results   = 0x0002,
    Robot_HP        = 0x0003,
    Venue_Events    = 0x0101,
    Referee_warning = 0x0104,
    Dart_fire       = 0x0105,
    Robot_performan = 0x0201,
    time_power      = 0x0202,
    Robot_location  = 0x0203,
    Robot_buff      = 0x0204,
    Damage_status   = 0x0206,
    time_shooting   = 0x0207,
    Allowable_ammo  = 0x0208,
    RFID            = 0x0209,
    Dart_directives = 0x020A,
    Ground_location = 0x020B,
    Radar_Marking   = 0x020C,
    Route_Informat  = 0x020D,
    Radar_Informat  = 0x020E,

    Robot_Interaction = 0x0301,
    Minimap           = 0x0303,
};

/* ==================== 帧头 ==================== */
typedef struct __packed
{
    uint8_t  SOF;
    uint16_t DataLenth;
    uint8_t  Seq;
    uint8_t  CRC8;
} frame_header_R_Typdef;

/* ==================== 基础数据 ==================== */

typedef struct __packed
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;

typedef struct __packed
{
    uint8_t winner;
} game_result_t;

typedef struct __packed
{
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
    uint16_t reserved;
    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;
} game_robot_HP_t;

/* ==================== 状态数据 ==================== */

typedef struct __packed
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} referee_warning_t;

typedef struct __packed
{
    uint8_t dart_remaining_time;
    uint16_t bit0_2 : 3;
    uint16_t bit3_5 : 3;
    uint16_t bit6_8 : 3;
    uint16_t bit9_15 : 7;
} dart_info_t;

typedef struct __packed
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} robot_status_t;

typedef struct __packed
{
    uint16_t chassis_voltage;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

typedef struct __packed
{
    float x;
    float y;
    float angle;
} robot_pos_t;

typedef struct __packed
{
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
} buff_t;

typedef struct __packed
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
} hurt_data_t;

typedef struct __packed
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
} shoot_data_t;

typedef struct __packed
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} projectile_allowance_t;

/* ==================== 位置 / 雷达 ==================== */

typedef struct __packed
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved_5_x;
    float reserved_5_y;
} ground_robot_position_t;

typedef struct __packed
{
    uint16_t mark_hero_progress:1;
    uint16_t mark_engineer_progress:1;
    uint16_t mark_standard_3_progress:1;
    uint16_t mark_standard_4_progress:1;
    uint16_t mark_air_progress:1;
    uint16_t mark_sentry_progress:1;
    uint16_t bit6:1;
    uint16_t bit7:1;
    uint16_t bit8:1;
    uint16_t bit9:1;
    uint16_t bit10:1;
    uint16_t bit11:1;
    uint16_t bit12_15:4;
} radar_mark_data_t;

typedef struct __packed
{
    uint8_t vulnerable_begin : 2;
    uint8_t vulnerable_now : 1;
    uint8_t bit3_4 : 2;
    uint8_t bit5 :1;
    uint8_t bit6_7 : 2;
} radar_info_t;

/* ==================== 交互数据（重点） ==================== */

typedef struct __packed
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[112];
} robot_interaction_data_t;

/* 图形结构 */
typedef struct __packed
{
    uint8_t figure_name[3];
    uint32_t operate_tpye:3;
    uint32_t figure_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t details_a:9;
    uint32_t details_b:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t details_c:10;
    uint32_t details_d:11;
    uint32_t details_e:11;
} interaction_figure_t;

/* UI字符 */
typedef struct __packed
{
    uint16_t data_id;
    uint16_t tx_id;
    uint16_t rx_id;
    uint8_t Character_configuration[15];
    uint8_t Character[30];
} graphic_data_struct_t;

/* ==================== 小地图 ==================== */

typedef struct __packed
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint8_t cmd_source;
} map_command_t;

/* ==================== 发送用结构体（补齐旧工程） ==================== */

typedef struct
{
    uint32_t bit0       :1;
    uint32_t bit1       :1;
    uint32_t bit2_12    :11;
    uint32_t bit13_16   :4;
    uint32_t bit17_20   :4;
    uint32_t bit21_22   :2;
    uint32_t bit23      :1;
    uint32_t bit24_31   :8;
} sentry_cmd_t;

typedef struct __packed
{
    uint8_t cmd;
    uint8_t robot_id;
    float x;
    float y;
} radar_cmd_t;

typedef struct __packed
{
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
} map_robot_data_t;

typedef struct __packed
{
    uint8_t intention;
    float start_position_x;
    float start_position_y;
    float delta_x[49];
    float delta_y[49];
} map_data_t;

typedef struct __packed
{
    uint8_t data[30];
} custom_info_t;

typedef struct
{
    uint32_t bit0       :1;
    uint32_t bit1       :1;
    uint32_t bit2       :1;
    uint32_t bit3_22    :20;
    uint32_t bit23_24   :2;
    uint32_t bit25_31   :7;
} event_data_t;
/* ==================== 用户数据 ==================== */

typedef struct __packed
{
    game_status_t game_status;
    game_result_t game_result;
    game_robot_HP_t game_robot_HP;

    referee_warning_t referee_warning;
    dart_info_t dart_info;
    robot_status_t robot_status;
    power_heat_data_t power_heat_data;
    robot_pos_t robot_pos;
    buff_t buff;
    hurt_data_t hurt_data;
    shoot_data_t shoot_data;
    projectile_allowance_t projectile_allowance;

    ground_robot_position_t ground_robot_position;
    radar_mark_data_t radar_mark_data;
    radar_info_t radar_info;

    map_command_t map_command;
    event_data_t event_data;
    custom_info_t custom_info;

} User_Data_T;

/* ==================== DMA解析用 ==================== */

typedef union
{
    struct __packed
    {
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
    } RX_Data_head;

    struct __packed
    {
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        game_status_t game_status;
        uint16_t frame_tail;
    } RX_Data_game_status;

    struct __packed
    {
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        robot_status_t robot_status;
        uint16_t frame_tail;
    } RX_Data_robot_status;

    uint8_t Data[255];

} ALL_RX_Data_T;

/* ==================== 外部变量 ==================== */

extern uint8_t Referee_Rx_Buf[2][REFEREE_RXFRAME_LENGTH];
extern User_Data_T User_data;

/* ==================== 接口 ==================== */

void Referee_System_Frame_Update(uint8_t *Buff);

#endif