#ifndef __REFEREE_H
#define __REFEREE_H

#include "usart.h"
#include "string.h"
#include "stdint.h"
#include "CRC_DJI.h"

/* 帧长度 */
#define REFEREE_RXFRAME_LENGTH 256
#define FrameHeader_Length 5U
#define CMDID_Length 2U
#define CRC16_Length 2U

/* ==================== CMD ID ==================== */
enum Read_Cmd_ID_Typdef
{
    game_state                 = 0x0001, /* 0x0001: 比赛状态数据，1Hz 周期发送 */
    Match_results              = 0x0002, /* 0x0002: 比赛结果数据，比赛结束时触发发送 */
    Robot_HP                   = 0x0003, /* 0x0003: 比赛机器人血量数据，3Hz 周期发送 */
    Venue_Events               = 0x0101, /* 0x0101: 场地事件数据，1Hz 周期发送或状态改变时触发发送 */
    Referee_warning            = 0x0104, /* 0x0104: 裁判系统警告数据，判罚发生时触发发送 */
    Dart_fire                  = 0x0105, /* 0x0105: 飞镖发射相关数据，飞镖发射或击中目标时触发发送 */
    Robot_performan            = 0x0201, /* 0x0201: 机器人性能体系数据，10Hz 周期发送 */
    time_power                 = 0x0202, /* 0x0202: 实时功率热量数据，50Hz 周期发送 */
    Robot_location             = 0x0203, /* 0x0203: 机器人位置数据，10Hz 周期发送 */
    Robot_buff                 = 0x0204, /* 0x0204: 机器人增益和底盘能量数据，3Hz 周期发送 */
    Damage_status              = 0x0206, /* 0x0206: 伤害状态数据，受到伤害时触发发送 */
    time_shooting              = 0x0207, /* 0x0207: 实时射击数据，弹丸发射后触发发送 */
    Allowable_ammo             = 0x0208, /* 0x0208: 允许发弹量/弹丸剩余量数据，10Hz 周期发送 */
    RFID_status                = 0x0209, /* 0x0209: 机器人 RFID 模块状态数据，3Hz 周期发送 */
    Dart_directives            = 0x020A, /* 0x020A: 飞镖机器人客户端指令数据，10Hz 周期发送 */
    Ground_location            = 0x020B, /* 0x020B: 地面机器人位置数据，1Hz 周期发送（仅供空中/雷达机器人接收） */
    Radar_Marking              = 0x020C, /* 0x020C: 雷达标记进度数据，1Hz 周期发送（仅供雷达机器人接收） */
    Route_Informat             = 0x020D, /* 0x020D: 哨兵自主决策信息同步数据，1Hz 周期发送 */
    Radar_Informat             = 0x020E, /* 0x020E: 雷达自主决策信息同步数据，1Hz 周期发送 */

    Robot_Interaction          = 0x0301, /* 0x0301: 机器人交互数据（多机战术通信/UI图形绘制），上限 30Hz 触发发送 */
    Custom_controller_to_robot = 0x0302, /* 0x0302: 自定义控制器向机器人发送交互数据，上限 30Hz 触发发送 */
    Minimap                    = 0x0303, /* 0x0303: 选手端小地图交互数据（双击/点击下发），选手端触发发送 */
    Map_receive_radar          = 0x0305, /* 0x0305: 选手端小地图接收雷达/哨兵下发的目标点/标定坐标，上限 10Hz 触发发送 */
    Custom_controller_to_map   = 0x0306, /* 0x0306: 自定义控制器键鼠操作数据（发给选手端小地图/自定义客户端），上限 30Hz */
    Map_receive_path           = 0x0307, /* 0x0307: 选手端接收路径数据（通常由云台/底盘自主导航下发），上限 10Hz */
    Map_receive_robot_info     = 0x0308, /* 0x0308: 选手端接收选手及机器人专属信息数据，上限 1Hz 触发发送 */
    Robot_to_custom_controller = 0x0309, /* 0x0309: 机器人向自定义控制器发送交互数据，上限 30Hz 触发发送 */
    Robot_to_custom_client     = 0x0310, /* 0x0310: 机器人向自定义客户端（选手端）发送交互数据，上限 50Hz 发送 */
    Custom_client_to_robot     = 0x0311  /* 0x0311: 自定义客户端（选手端）向机器人发送交互数据，上限 20Hz 发送 */
};

typedef struct __packed
{
    uint8_t  SOF;         /* 帧头标志，固定为 0xA5 */
    uint16_t DataLenth;   /* 数据帧中 data 段的长度 */
    uint8_t  Seq;         /* 包序号 */
    uint8_t  CRC8;        /* 帧头部分（前 4 字节）的 CRC8 校验值 */
} frame_header_R_Typdef;

typedef struct __packed
{
    uint8_t game_type : 4;         /* 比赛类型：1-RMUC, 2-RMUT, 3-RMUA, 4-RMUL */
    uint8_t game_progress : 4;     /* 比赛阶段：0-未开始, 1-准备阶段, 2-自检阶段, 3-5s倒计时, 4-比赛中, 5-比赛结束 */
    uint16_t stage_remain_time;    /* 当前比赛阶段剩余时间（单位：秒） */
    uint64_t SyncTimeStamp;        /* 机器人时间同步时间戳（单位：毫秒） */
} game_status_t;

typedef struct __packed
{
    uint8_t winner;                /* 获胜队伍：0-平局, 1-红方胜, 2-蓝方胜 */
} game_result_t;

typedef struct __packed
{
    uint16_t ally_1_robot_HP;      /* 己方 1 号英雄机器人血量 */
    uint16_t ally_2_robot_HP;      /* 己方 2 号工程机器人血量 */
    uint16_t ally_3_robot_HP;      /* 己方 3 号步兵机器人血量 */
    uint16_t ally_4_robot_HP;      /* 己方 4 号步兵机器人血量 */
    uint16_t reserved;             /* 保留未定义字段 */
    uint16_t ally_7_robot_HP;      /* 己方 7 号哨兵机器人血量 */
    uint16_t ally_outpost_HP;      /* 己方前哨站血量 */
    uint16_t ally_base_HP;         /* 己方基地血量 */
} game_robot_HP_t;

/* 0x0101 场地事件数据 */
typedef struct __packed
{
    uint32_t supply_zone_status : 1;                 // bit 0: 己方补给区的占领状态（0:未占领/无增益, 1:已占领/有增益）
    uint32_t reserved_1 : 1;                         // bit 1: 保留位
    uint32_t supply_zone_status_rmul : 1;            // bit 2: 己方补给区的占领状态 RMUL
    uint32_t small_energy_mechanism_status : 2;      // bit 3-4: 己方小能量机关的激活状态（0:未激活, 1:正处于激活中, 2:已全部激活完成）
    uint32_t big_energy_mechanism_status : 2;        // bit 5-6: 己方大能量机关的激活状态（0:未激活, 1:正处于激活中, 2:已全部激活完成）
    uint32_t central_highland_status : 2;            // bit 7-8: 己方中央高地的占领状态（0:未占领, 1:已占领, 2:被抢占中）
    uint32_t trapezoidal_highland_status : 2;        // bit 9-10: 己方梯形高地的占领状态
    uint32_t dart_hit_time : 9;                      // bit 11-19: 对方飞镖最后击中己方前哨站或基地的时间（倒计时秒数）
    uint32_t dart_hit_target : 3;                    // bit 20-22: 对方飞镖最后击中的具体目标（0:未击中, 1:前哨站, 2:基地固定目标, 3:基地随机目标）
    uint32_t center_buff_status : 2;                 // bit 23-24: 中心增益点的占领状态 RMUL
    uint32_t fortress_buff_status : 2;               // bit 25-26: 己方堡垒增益点的占领状态（0:未占领, 1:已占领）
    uint32_t outpost_buff_status : 2;                // bit 27-28: 己方前哨站增益点的占领状态
    uint32_t base_buff_status : 1;                   // bit 29: 己方基地增益点的占领状态
    uint32_t reserved_2 : 2;                         // bit 30-31: 保留位
} event_data_t;

typedef struct __packed
{
    uint8_t level;                  /* 判罚警告等级：1-黄牌, 2-红牌, 3-直接判负 */
    uint8_t offending_robot_id;     /* 违规犯规机器人的 ID，无目标机器人时为 0 */
    uint8_t count;                  /* 触发同一违规等级判罚的累计次数 */
} referee_warning_t;

/* 0x0105 飞镖发射相关数据 */
typedef struct __packed
{
    uint8_t dart_remaining_time;        /* 飞镖发射引导剩余倒计时（单位：秒） */
    uint16_t dart_hit_target : 3;       // bit 0-2: 最近一次己方飞镖击中的目标
    uint16_t dart_hit_count : 3;        // bit 3-5: 对方最近被击中的目标累计被击中总计次数
    uint16_t dart_selected_target : 3;  // bit 6-8: 飞镖此时在系统端选定的击打目标
    uint16_t reserved : 7;              // bit 9-15: 保留位
} dart_info_t;

/* 0x0201 机器人性能体系数据 */
typedef struct __packed
{
    uint8_t robot_id;                            /* 本机器人的专属 ID（如红1为1，蓝1为101） */
    uint8_t robot_level;                         /* 机器人当前等级：1级, 2级, 3级等等 */
    uint16_t current_HP;                         /* 机器人当前实时血量 */
    uint16_t maximum_HP;                         /* 机器人当前等级对应的最大血量上限 */
    uint16_t shooter_barrel_cooling_value;       /* 机器人发射机构枪口每秒冷却值 */
    uint16_t shooter_barrel_heat_limit;          /* 机器人当前枪口热量上限值 */
    uint16_t chassis_power_limit;                /* 机器人当前底盘限制的最大功率上限值（W） */
    uint8_t power_management_gimbal_output : 1;  /* 云台电源输出状态：0-关闭, 1-开启 */
    uint8_t power_management_chassis_output : 1; /* 底盘电源输出状态：0-关闭, 1-开启 */
    uint8_t power_management_shooter_output : 1; /* 发射机构电源输出状态：0-关闭, 1-开启 */
    uint8_t reserved_power_management : 5;       /* 补齐一个字节，防止错位 */
} robot_status_t;

typedef struct __packed
{
    uint16_t reserved_1;                         /* 历史底盘电压保留位 */
    uint16_t reserved_2;                         /* 历史底盘电流保留位 */
    float reserved_3;                            /* 历史底盘功率保留位 */
    uint16_t buffer_energy;                      /* 机器人当前底盘缓冲能量（焦耳 J） */
    uint16_t shooter_17mm_barrel_heat;           /* 机器人 17mm 枪口实时热量值 */
    uint16_t shooter_42mm_barrel_heat;           /* 机器人 42mm 枪口实时热量值 */
} power_heat_data_t;

typedef struct __packed
{
    float x;                                     /* 机器人当前的全局定位坐标 X 轴位置（单位：米） */
    float y;                                     /* 机器人当前的全局定位坐标 Y 轴位置（单位：米） */
    float angle;                                 /* 机器人当前的枪口/底盘朝向偏航角（单位：度，范围0~360°） */
} robot_pos_t;

/* 0x0204 机器人增益和底盘能量数据 */
typedef struct __packed
{
    uint8_t recovery_buff;                       /* 机器人实时回血增益（数值代表每秒回血百分比） */
    uint16_t cooling_buff;                       /* 枪口冷却增益（数值代表枪口冷却倍数） */
    uint8_t defence_buff;                        /* 防御增益（百分比，减少受到的外部伤害数值） */
    uint8_t vulnerability_buff;                  /* 负面易伤增益状态（增加受到的外部伤害数值） */
    uint16_t attack_buff;                        /* 攻击力增益（百分比） */
    uint8_t energy_125 : 1;                      // bit 0: 底盘剩余能量是否 >=125%
    uint8_t energy_100 : 1;                      // bit 1: 底盘剩余能量是否 >=100%
    uint8_t energy_50 : 1;                       // bit 2: 底盘剩余能量是否 >=50%
    uint8_t energy_30 : 1;                       // bit 3: 底盘剩余能量是否 >=30%
    uint8_t energy_15 : 1;                       // bit 4: 底盘剩余能量是否 >=15%
    uint8_t energy_5 : 1;                        // bit 5: 底盘剩余能量是否 >=5%
    uint8_t energy_1 : 1;                        // bit 6: 底盘剩余能量是否 >=1%
    uint8_t energy_reserved : 1;                 // bit 7: 底盘能量状态位保留
} buff_t;

typedef struct __packed
{
    uint8_t armor_id : 4;                        /* 受击装甲板 ID：0~4 代表对应位置装甲板编号 */
    uint8_t HP_deduction_reason : 4;             /* 扣血原因：0-装甲受击, 1-模块掉线, 2-超功率, 3-超初速, 4-超热量 */
} hurt_data_t;

/* 0x0207 实时射击数据 */
typedef struct __packed
{
    uint8_t reserved_1 : 1;                      // bit 0: 保留位
    uint8_t bullet_type_17mm : 1;                // bit 1: 发射弹丸为 17mm 弹丸
    uint8_t bullet_type_42mm : 1;                // bit 2: 发射弹丸为 42mm 弹丸
    uint8_t reserved_2 : 5;                      // bit 3-7: 保留位
    uint8_t shooter_number;                      /* 发射机构编号：1-17mm发射机构1, 2-17mm发射机构2, 3-42mm发射机构 */
    uint8_t launching_frequency;                 /* 实时射频：当前射击弹丸的发射频率（Hz） */
    float initial_speed;                         /* 弹丸发射初速度（单位：m/s） */
} shoot_data_t;

typedef struct __packed
{
    uint16_t projectile_allowance_17mm;          /* 17mm 弹丸的剩余允许发弹量 */
    uint16_t projectile_allowance_42mm;          /* 42mm 弹丸的剩余允许发弹量 */
    uint16_t remaining_gold_coin;                /* 剩余金币数量 */
    uint16_t projectile_allowance_fortress;      /* 堡垒增益点提供的储备17mm弹丸允许发弹量 */
} projectile_allowance_t;

/* 0x0209 机器人 RFID 模块状态 */
typedef struct __packed
{
    // 32-bit (uint32_t rfid_status 展开)
    uint32_t ally_base : 1;                      /* bit 0: 是否处于己方基地 RFID 增益点 */
    uint32_t ally_central_highland : 1;          /* bit 1: 是否处于己方中央高地 RFID 增益点 */
    uint32_t enemy_central_highland : 1;         /* bit 2: 是否处于对方中央高地 RFID 增益点 */
    uint32_t ally_trapezoidal_highland : 1;      /* bit 3: 是否处于己方梯形高地 RFID 增益点 */
    uint32_t enemy_trapezoidal_highland : 1;     /* bit 4: 是否处于对方梯形高地 RFID 增益点 */
    uint32_t ally_fly_ramp_front : 1;            /* bit 5: 是否处于己方飞坡前方 RFID 点 */
    uint32_t ally_fly_ramp_back : 1;             /* bit 6: 是否处于己方飞坡后方 RFID 点 */
    uint32_t enemy_fly_ramp_front : 1;           /* bit 7: 是否处于对方飞坡前方 RFID 点 */
    uint32_t enemy_fly_ramp_back : 1;            /* bit 8: 是否处于对方飞坡后方 RFID 点 */
    uint32_t ally_central_highland_lower : 1;    /* bit 9: 是否处于己方中央高地低区 RFID 点 */
    uint32_t ally_central_highland_upper : 1;    /* bit 10: 是否处于己方中央高地高区 RFID 点 */
    uint32_t enemy_central_highland_lower : 1;   /* bit 11: 是否处于对方中央高地低区 RFID 点 */
    uint32_t enemy_central_highland_upper : 1;   /* bit 12: 是否处于对方中央高地高区 RFID 点 */
    uint32_t ally_highway_lower : 1;             /* bit 13: 是否处于己方公路低区 RFID 点 */
    uint32_t ally_highway_upper : 1;             /* bit 14: 是否处于己方公路高区 RFID 点 */
    uint32_t enemy_highway_lower : 1;            /* bit 15: 是否处于对方公路低区 RFID 点 */
    uint32_t enemy_highway_upper : 1;            /* bit 16: 是否处于对方公路高区 RFID 点 */
    uint32_t ally_fortress : 1;                  /* bit 17: 是否处于己方堡垒增益点 RFID */
    uint32_t ally_outpost : 1;                   /* bit 18: 是否处于己方前哨站增益点 RFID */
    uint32_t ally_supply_zone_not_overlapping : 1;/* bit 19: 是否处于己方补给区非重叠区域 RFID */
    uint32_t ally_supply_zone_overlapping : 1;   /* bit 20: 是否处于己方补给区重叠区域 RFID */
    uint32_t ally_assembly_zone : 1;             /* bit 21: 是否处于己方入场复活/集结区 RFID */
    uint32_t enemy_assembly_zone : 1;            /* bit 22: 是否处于对方入场复活/集结区 RFID */
    uint32_t center_buff_zone : 1;               /* bit 23: 是否处于中心增益区 RFID RMUL */
    uint32_t enemy_fortress : 1;                 /* bit 24: 是否处于对方堡垒增益点 RFID */
    uint32_t enemy_outpost : 1;                  /* bit 25: 是否处于对方前哨站增益点 RFID */
    uint32_t ally_tunnel_highway_lower : 1;      /* bit 26: 是否处于己方隧道公路低区 RFID */
    uint32_t ally_tunnel_highway_middle : 1;     /* bit 27: 是否处于己方隧道公路中区 RFID */
    uint32_t ally_tunnel_highway_upper : 1;      /* bit 28: 是否处于己方隧道公路高区 RFID */
    uint32_t ally_tunnel_trapezoidal_lower : 1;  /* bit 29: 是否处于己方隧道梯形低区 RFID */
    uint32_t ally_tunnel_trapezoidal_middle : 1; /* bit 30: 是否处于己方隧道梯形中区 RFID */
    uint32_t ally_tunnel_trapezoidal_upper : 1;  /* bit 31: 是否处于己方隧道梯形高区 RFID */

    // 8-bit (uint8_t rfid_status_2 展开)
    uint8_t enemy_tunnel_highway_lower : 1;      /* bit 0: 是否处于对方隧道公路低区 RFID */
    uint8_t enemy_tunnel_highway_middle : 1;     /* bit 1: 是否处于对方隧道公路中区 RFID */
    uint8_t enemy_tunnel_highway_upper : 1;      /* bit 2: 是否处于对方隧道公路高区 RFID */
    uint8_t enemy_tunnel_trapezoidal_lower : 1;  /* bit 3: 是否处于对方隧道梯形低区 RFID */
    uint8_t enemy_tunnel_trapezoidal_middle : 1; /* bit 4: 是否处于对方隧道梯形中区 RFID */
    uint8_t enemy_tunnel_trapezoidal_upper : 1;  /* bit 5: 是否处于对方隧道梯形高区 RFID */
    uint8_t reserved_tunnel : 2;                 /* bit 6-7: 隧道位置标志保留位 */
} rfid_status_t;

typedef struct __packed
{
    uint8_t dart_launch_opening_status;          /* 飞镖发射站闸门的开启状态（0:关闭, 1:开启） */
    uint8_t reserved;                            /* 保留未定义字段 */
    uint16_t target_change_time;                 /* 切换飞镖打击目标对应的比赛剩余时间（单位：秒） */
    uint16_t latest_launch_cmd_time;             /* 最近一次接收到发射飞镖指令对应的比赛剩余时间 */
} dart_client_cmd_t;

typedef struct __packed
{
    float hero_x;                                /* 己方 1 号英雄机器人当前的地图横坐标位置 */
    float hero_y;                                /* 己方 1 号英雄机器人当前的地图纵坐标位置 */
    float engineer_x;                            /* 己方 2 号工程机器人当前的地图横坐标位置 */
    float engineer_y;                            /* 己方 2 号工程机器人当前的地图纵坐标位置 */
    float standard_3_x;                          /* 己方 3 号步兵机器人当前的地图横坐标位置 */
    float standard_3_y;                          /* 己方 3 号步兵机器人当前的地图纵坐标位置 */
    float standard_4_x;                          /* 己方 4 号步兵机器人当前的地图横坐标位置 */
    float standard_4_y;                          /* 己方 4 号步兵机器人当前的地图纵坐标位置 */
    float reserved_1;                            /* 位置同步预留变量 1 */
    float reserved_2;                            /* 位置同步预留变量 2 */
} ground_robot_position_t;

/* 0x020C 雷达标记进度数据 */
typedef struct __packed
{
    uint16_t enemy_hero_mark : 1;          // bit 0: 对方英雄机器人被我方标记的进度状态
    uint16_t enemy_engineer_mark : 1;      // bit 1: 对方工程机器人被我方标记的进度状态
    uint16_t enemy_infantry_3_mark : 1;    // bit 2: 对方 3 号步兵机器人被我方标记的进度状态
    uint16_t enemy_infantry_4_mark : 1;    // bit 3: 对方 4 号步兵机器人被我方标记的进度状态
    uint16_t enemy_aerial_mark : 1;        // bit 4: 对方空中机器人被我方标记的进度状态
    uint16_t enemy_sentry_mark : 1;        // bit 5: 对方哨兵机器人被我方标记的进度状态
    uint16_t ally_hero_mark : 1;           // bit 6: 己方英雄机器人被敌方标记的进度状态
    uint16_t ally_engineer_mark : 1;       // bit 7: 己方工程机器人被敌方标记的进度状态
    uint16_t ally_infantry_3_mark : 1;     // bit 8: 己方 3 号步兵机器人被敌方标记的进度状态
    uint16_t ally_infantry_4_mark : 1;     // bit 9: 己方 4 号步兵机器人被敌方标记的进度状态
    uint16_t ally_aerial_mark : 1;         // bit 10: 己方空中机器人被敌方标记的进度状态
    uint16_t ally_sentry_mark : 1;         // bit 11: 己方哨兵机器人被敌方标记的进度状态
    uint16_t reserved : 4;                 // bit 12-15: 雷达标记进度保留位
} radar_mark_data_t;

/* 0x020D 哨兵自主决策信息同步 */
typedef struct __packed
{
    // uint32_t sentry_info 展开
    uint32_t sentry_redeemed_projectile_allowance : 11; /* bit 0-10: 哨兵已经远程兑换发弹量的总计数量 */
    uint32_t sentry_remote_redeemed_projectile_times : 4;/* bit 11-14: 哨兵执行远程兑换发弹量的累计次数 */
    uint32_t sentry_remote_redeemed_HP_times : 4;       /* bit 15-18: 哨兵执行远程兑换血量的累计次数 */
    uint32_t can_confirm_free_revive : 1;               /* bit 19: 哨兵当前是否能进行免费复活的确认 */
    uint32_t can_redeem_immediate_revive : 1;           /* bit 20: 哨兵当前是否符合消耗金币立即复活的条件 */
    uint32_t cost_for_immediate_revive : 10;            /* bit 21-30: 哨兵当前若要立即复活所需消耗的金币代币数量 */
    uint32_t reserved_1 : 1;                            /* bit 31: 决策同步信息保留位 1 */

    // uint16_t sentry_info_2 展开
    uint16_t is_out_of_combat : 1;                       /* bit 0: 哨兵机器人当前是否处于脱离战斗状态 */
    uint16_t remaining_redeemable_projectile_allowance : 11;/* bit 1-11: 哨兵当前阶段还允许购买/兑换的发弹量剩余余额 */
    uint16_t sentry_posture : 2;                         /* bit 12-13: 哨兵当前战术姿态（如 0-常规开火, 1-防御等） */
    uint16_t can_enter_activating_state : 1;             /* bit 14: 哨兵当前是否能切入能量机关激活准备状态 */
    uint16_t reserved_2 : 1;                             /* bit 15: 决策同步信息保留位 2 */
} sentry_info_t;

/* 0x020E 雷达自主决策信息同步 */
typedef struct __packed
{
    uint8_t radar_double_damage_chance : 2;   // bit 0-1: 雷达当前可用的“双倍易伤”触发机会剩余次数
    uint8_t enemy_is_double_damaged : 1;      // bit 2: 对方当前是否正处于受我方雷达影响的“双倍易伤”状态中
    uint8_t ally_encryption_level : 2;        // bit 3-4: 己方雷达当前信道数据的加密等级
    uint8_t can_modify_password : 1;          // bit 5: 雷达当前是否允许选手修改系统密钥/密码
    uint8_t reserved : 2;                     // bit 6-7: 雷达自主决策信息预留
} radar_info_t;

typedef struct __packed
{
    uint16_t data_cmd_id;           /* 机器人子交互数据的指令子 ID (例如自定义图形 UI 等对应的代码) */
    uint16_t sender_id;             /* 发送数据方机器人的通信 ID */
    uint16_t receiver_id;           /* 接收数据方机器人或客户端的通信 ID */
    uint8_t user_data[112];         /* 包含在交互协议中的主要用户自定义数据负载（上限 112 字节） */
} robot_interaction_data_t;

/* 图形结构 */
typedef struct __packed
{
    uint8_t figure_name[3];         /* 自定义图形在客户端系统中的名字标签，由三个 ASCII 字节表示 */
    uint32_t operate_tpye:3;        // 操作类型：0-空操作, 1-添加图形, 2-修改图形, 3-删除图形
    uint32_t figure_tpye:3;         // 图形类型：0-直线, 1-矩形, 2-整圆, 3-椭圆, 4-圆弧, 5-浮点数, 6-整数, 7-字符
    uint32_t layer:4;               // 图形渲染所在的图层级（范围0~9）
    uint32_t color:4;               // 图形颜色：0-红蓝主色, 1-黄, 2-绿, 3-橙, 4-紫, 5-粉, 6-青, 7-黑, 8-白
    uint32_t details_a:9;           // 图形核心细节参数 A（如圆弧起始角度，字体大小等）
    uint32_t details_b:9;           // 图形核心细节参数 B（如圆弧终止角度等）
    uint32_t width:10;              // 线宽/图形线粗细（像素单位）
    uint32_t start_x:11;            // 图形绘制起点坐标 X 位置（像素横坐标）
    uint32_t start_y:11;            // 图形绘制起点坐标 Y 位置（像素纵坐标）
    uint32_t details_c:10;          // 图形核心细节参数 C（如文字浮点数的精度控制等）
    uint32_t details_d:11;          // 图形核心细节参数 D（如矩形对角线终点坐标 X 等）
    uint32_t details_e:11;          // 图形核心细节参数 E（如矩形对角线终点坐标 Y 等）
} interaction_figure_t;

/* UI字符 */
typedef struct __packed
{
    uint16_t data_id;                            /* 交互数据的具体操作特征数据码 */
    uint16_t tx_id;                              /* 发送端的机器人通信链路 ID */
    uint16_t rx_id;                              /* 接收端的选手操作端自定义客户端链路 ID */
    uint8_t Character_configuration[15];         /* 针对在屏幕渲染输出的字符串配置参数（如图形基本属性信息） */
    uint8_t Character[30];                       /* 实际需要在选手端屏幕交互绘制显示的 ASCII 字符串内容（上限 30 字节） */
} graphic_data_struct_t;

typedef struct __packed
{
    float target_position_x;        /* 选手在小地图端双击或点击标定的目标攻击/防守点横坐标 */
    float target_position_y;        /* 选手在小地图端双击或点击标定的目标攻击/防守点纵坐标 */
    uint8_t cmd_keyboard;           /* 选手在触发鼠标操作时，同时按下的辅助控制键盘键值（如 Shift, Ctrl） */
    uint8_t target_robot_id;        /* 若点击的是特定小地图目标，对应的目标机器人专属 ID */
    uint16_t cmd_source;            /* 产生该交互小地图指令的信息发送源头标识 */
} map_command_t;

/* 0x0120 哨兵自主决策指令 */
typedef struct __packed
{
    uint32_t confirm_resurrection : 1;                      // bit 0: 哨兵确认处于可免费复活状态下的复活请求信号
    uint32_t confirm_immediate_resurrection : 1;            // bit 1: 哨兵确认需要消耗金币进行“立即复活”的控制指令
    uint32_t redeem_projectile_allowance : 11;              // bit 2-12: 哨兵本次自主决策申请远程购买的允许发弹量数量
    uint32_t remote_redeem_projectile_request_times : 4;    // bit 13-16: 哨兵发出远程发弹量购买请求的增量阶梯次数
    uint32_t remote_redeem_HP_request_times : 4;            // bit 17-20: 哨兵发出远程血量补给购买请求的增量阶梯次数
    uint32_t sentry_posture_cmd : 2;                        // bit 21-22: 哨兵向系统上报或变更当前运行战术姿态控制指令
    uint32_t confirm_activate_energy_mechanism : 1;         // bit 23: 哨兵触发请求配合队友激活己方大/小能量机关标志
    uint32_t reserved : 8;                                  // bit 24-31: 哨兵自主决策控制命令保留段
} sentry_cmd_t;

/* 0x0121 雷达自主决策指令 */
typedef struct __packed
{
    uint8_t radar_double_damage_cmd;  /* 雷达下达的触发“对方全队双倍易伤”控制开关（1代表下达） */
    uint8_t password_cmd;             /* 雷达修改、检验及尝试破解系统加密密码对应的控制指令码 */
    uint8_t password_1;               /* 动态配置的新密码/密钥第 1 个字节 */
    uint8_t password_2;               /* 动态配置的新密码/密钥第 2 个字节 */
    uint8_t password_3;               /* 动态配置的新密码/密钥第 3 个字节 */
    uint8_t password_4;               /* 动态配置的新密码/密钥第 4 个字节 */
    uint8_t password_5;               /* 动态配置的新密码/密钥第 5 个字节 */
    uint8_t password_6;               /* 动态配置的新密码/密钥第 6 个字节 */
} radar_cmd_t;

typedef struct __packed
{
    uint16_t hero_position_x;         /* 地图层级下英雄机器人的精细缩放横坐标（毫米级单位） */
    uint16_t hero_position_y;         /* 地图层级下英雄机器人的精细缩放纵坐标（毫米级单位） */
    uint16_t engineer_position_x;     /* 地图层级下工程机器人的精细缩放横坐标 */
    uint16_t engineer_position_y;     /* 地图层级下工程机器人的精细缩放纵坐标 */
    uint16_t infantry_3_position_x;   /* 地图层级下 3 号步兵机器人的精细缩放横坐标 */
    uint16_t infantry_3_position_y;   /* 地图层级下 3 号步兵机器人的精细缩放纵坐标 */
    uint16_t infantry_4_position_x;   /* 地图层级下 4 号步兵机器人的精细缩放横坐标 */
    uint16_t infantry_4_position_y;   /* 地图层级下 4 号步兵机器人的精细缩放纵坐标 */
    uint16_t reserved_1;              /* 地面地图位置阵列保留变量 1 */
    uint16_t reserved_2;              /* 地面地图位置阵列保留变量 2 */
    uint16_t sentry_position_x;       /* 地图层级下哨兵机器人的精细缩放横坐标 */
    uint16_t sentry_position_y;       /* 地图层级下哨兵机器人的精细缩放纵坐标 */
} map_robot_data_t;

typedef struct __packed
{
    uint8_t intention;                /* 本次下发的路径战术意图（如 0-常规巡航, 1-紧急增援, 2-伏击防守） */
    uint16_t start_position_x;        /* 选手端地图导航规划路径的起点横坐标 */
    uint16_t start_position_y;        /* 选手端地图导航规划路径的起点纵坐标 */
    int8_t delta_x[49];               /* 路径规划中后续 49 个离散航点的 X 轴相对偏移差值阵列 */
    int8_t delta_y[49];               /* 路径规划中后续 49 个离散航点的 Y 轴相对偏移差值阵列 */
    uint16_t sender_id;               /* 下发路径规划任务的源机器人/发送端节点 ID */
} map_data_t;

typedef struct __packed
{
    uint16_t sender_id;               /* 该帧自定义专属非标配置信息的发送源节点 ID */
    uint16_t receiver_id;             /* 该帧自定义专属非标配置信息的接收目标节点 ID */
    uint8_t user_data[30];            /* 自定义非结构化用户信息缓存数据区（上限 30 字节） */
} custom_info_t;

/* 0x0306 自定义控制器键鼠操作 */
typedef struct __packed
{
    uint16_t key_value;               /* 自定义控制器上传的物理键盘复合按键键值（按位映射各个按键状态） */
    uint16_t x_position:12;           // bit 0-11: 选手鼠标当前的虚拟屏幕绝对横坐标位置值
    uint16_t mouse_left:4;            // bit 12-15: 鼠标左键点击/持续按下状态深度标志
    uint16_t y_position:12;           // bit 16-27: 选手鼠标当前的虚拟屏幕绝对纵坐标位置值
    uint16_t mouse_right:4;           // bit 28-31: 鼠标右键点击/持续按下状态深度标志
    uint16_t reserved;                /* 键鼠控制器交互数据保留对齐位 */
} custom_client_data_t;

typedef struct __packed
{
    game_status_t game_status;                           /* 内部包含：0x0001 比赛状态数据段 */
    game_result_t game_result;                           /* 内部包含：0x0002 比赛结果数据段 */
    game_robot_HP_t game_robot_HP;                       /* 内部包含：0x0003 所有机器人实时血量数据段 */

    referee_warning_t referee_warning;                   /* 内部包含：0x0104 系统违规与警告数据段 */
    dart_info_t dart_info;                               /* 内部包含：0x0105 飞镖发射及目标打击统计数据段 */
    robot_status_t robot_status;                         /* 内部包含：0x0201 本机基本性能与功率管理控制段 */
    power_heat_data_t power_heat_data;                   /* 内部包含：0x0202 本机实时缓冲能量与枪口双路热量段 */
    robot_pos_t robot_pos;                               /* 内部包含：0x0203 本机全局平面位置与朝向姿态数据段 */
    buff_t buff;                                         /* 内部包含：0x0204 机器人回血/冷却/防御增益及超级电容标志 */
    hurt_data_t hurt_data;                               /* 内部包含：0x0206 本机受击装甲板方位及扣血原因数据段 */
    shoot_data_t shoot_data;                             /* 内部包含：0x0207 本机射击弹丸类型、射频及初速度检测段 */
    projectile_allowance_t projectile_allowance;         /* 内部包含：0x0208 机器人17/42mm枪口可用弹量及金币代币余额 */
    rfid_status_t rfid_status;                           /* 内部包含：0x0209 机器人当前覆盖踩中的所有 RFID 增益点状态 */
    dart_client_cmd_t dart_client_cmd;                   /* 内部包含：0x020A 飞镖闸门开启状态与选定目标时间戳指令段 */

    ground_robot_position_t ground_robot_position;       /* 内部包含：0x020B 地面多机器人全局位置信息阵列（针对空中/雷达） */
    radar_mark_data_t radar_mark_data;                   /* 内部包含：0x020C 雷达对红蓝两队各机型目标的双向标记进度段 */
    sentry_info_t sentry_info;                           /* 内部包含：0x020D 哨兵机器人专属的远程兑换、姿态及自主决策状态 */
    radar_info_t radar_info;                             /* 内部包含：0x020E 雷达机器人专属的易伤机会与信道加密决策状态 */

    map_command_t map_command;                           /* 内部包含：0x0303 选手端点击地图下发的目标标定控制指令 */
    event_data_t event_data;                             /* 内部包含：0x0101 全场机关、高地占领及飞镖受击点关键场地事件 */
    custom_info_t custom_info;                           /* 内部包含：自定义非标小数据块数据段 */
} User_Data_T;

typedef union
{
    struct __packed
    {
        frame_header_R_Typdef frame_header;              /* 统一封装的数据包通用 5 字节帧头 */
        uint16_t read_cmd_id;                            /* 当前数据包对应的 2 字节核心命令字识别码 */
    } RX_Data_head;

    struct __packed
    {
        frame_header_R_Typdef frame_header;              /* 5 字节帧头 */
        uint16_t read_cmd_id;                            /* 2 字节命令字 (0x0001) */
        game_status_t game_status;                       /* 实际搭载的数据载荷：比赛状态结构体 */
        uint16_t frame_tail;                             /* 2 字节尾部：整帧数据的 CRC16 校验值 */
    } RX_Data_game_status;

    struct __packed
    {
        frame_header_R_Typdef frame_header;              /* 5 字节帧头 */
        uint16_t read_cmd_id;                            /* 2 字节命令字 (0x0201) */
        robot_status_t robot_status;                     /* 实际搭载的数据载荷：机器人性能体系结构体 */
        uint16_t frame_tail;                             /* 2 字节尾部：整帧数据的 CRC16 校验值 */
    } RX_Data_robot_status;

    uint8_t Data[255];                                   /* 字节数组形态：用于底层串口 DMA 接收或遍历校验的通用缓冲区 */

} ALL_RX_Data_T;

extern uint8_t Referee_Rx_Buf[2][REFEREE_RXFRAME_LENGTH];
extern User_Data_T User_data;

void Referee_System_Frame_Update(uint8_t *Buff, uint16_t Size);

void Referee_Send_KeyMouse(custom_client_data_t *control_data);

void Referee_Send_Data(uint16_t cmd_id, uint8_t *p_data, uint16_t len);

#endif