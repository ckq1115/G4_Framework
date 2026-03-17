//
// Created by CaoKangqi on 2026/2/23.
//

#ifndef G4_FRAMEWORK_REFEREE_H
#define G4_FRAMEWORK_REFEREE_H

#include "usart.h"
#include "string.h"
#include "CRC_DJI.h"

enum Read_Cmd_ID_Typdef
{
    game_state      = 0x0001 ,  //比赛状态数据，固定以 1Hz 频率发送
    Match_results   = 0x0002 ,  //比赛结果数据，比赛结束触发发送
    Robot_HP        = 0x0003 ,  //机器人血量数据，固定以 3Hz 频率发送
    Venue_Events    = 0x0101 ,  //场地事件数据，固定以 1Hz 频率发送
    Referee_warning = 0x0104 ,  //裁判警告数据，己方判罚/判负时触发发送，其余时间以 1Hz 频率发送
    Dart_fire       = 0x0105 ,  //飞镖发射相关数据，固定以 1Hz频率发送
    Robot_performan = 0x0201 ,  //机器人性能体系数据，固定以10Hz 频率发送
    time_power      = 0x0202 ,  //实时底盘缓冲能量和射击热量数据，固定以 10Hz 频率发送
    Robot_location  = 0x0203 ,  //机器人位置数据，固定以 1Hz 频率发送，其余机器人的发送频率为 1Hz
    Robot_buff      = 0x0204 ,  //机器人增益和底盘能量数据，固定以 3Hz 频率发送
    Air_support     = 0x0205 ,  ////空中支援时间数据，固定 1Hz 频率发送
    Damage_status   = 0x0206 ,  //伤害状态数据，伤害发生后发送
    time_shooting   = 0x0207 ,  //实时射击数据，弹丸发射后发送
    Allowable_ammo  = 0x0208 ,  //允许发弹量，固定以 10Hz 频率发送
    RFID            = 0x0209 ,  //机器人 RFID 模块状态，固定以3Hz 频率发送
    Dart_directives = 0x020A ,  //飞镖选手端指令数据，固定以3Hz 频率发送
    Ground_location = 0x020B ,  //地面机器人位置数据，固定以1Hz 频率发送
    Radar_Marking   = 0x020C ,  //雷达标记进度数据，固定以 1Hz频率发送
    Route_Informat  = 0x020D ,  //哨兵自主决策信息同步，固定以1Hz 频率发送
    Radar_Informat  = 0x020E ,  //雷达自主决策信息同步，固定以1Hz 频率发送
		Robot_Interaction = 0x301 ,  //机器人交互数据，发送方触发发送，频率上限为 30Hz
    Custom_controller_robot = 0x302 ,  //自定义控制器与机器人交互数据，发送方触发发送，频率上限为 30Hz
		Minimap         = 0x0303 ,  //选手端小地图交互数据，选手端触发发送
    Minimap_radar          = 0x0305 ,  //选手端小地图接收雷达数据，频率上限为 5Hz
		Custom_controller_master = 0x0306 ,  //自定义控制器与选手端交互数据，发送方触发发送，频率上限为 30Hz
		Minimap_road = 0x0307 ,  //选手端小地图接收路径数据，频率上限为 1Hz
		Minimap_robot = 0x0308 ,  //选手端小地图接收机器人数据，频率上限为 3Hz
		Custom_controller_rx_robot = 0x0309 ,  //自定义控制器接收机器人数据，频率上限为 10Hz
		Robot_tx_custom_controlapp = 0x0310 ,  //机器人发送给自定义客户端的数据，频率上限为 50Hz
		Custom_controlapp_tx_robot = 0x0311 ,  //自定义客户端发送给机器人的自定义指令，频率上限为 75Hz
		Enemy_position = 0x0A01 ,  //对方机器人的位置坐标，频率上限为 10Hz
		Enemy_health = 0x0A02 ,  //对方机器人的血量信息，频率上限为 10Hz
		Enemy_ammunition = 0x0A03 ,  //对方机器人的剩余发弹量信息，频率上限为 10Hz
		Enemy_status = 0x0A04 ,  //对方队伍的宏观状态信息，频率上限为 10Hz
		Enemy_buffs = 0x0A05 ,  //对方各机器人当前增益效果，频率上限为 10Hz
		Enemy_Key = 0x0A06 ,  //对方干扰波密钥，频率上限为10Hz
};

/*0x0001*/
typedef struct __packed
{
    uint8_t game_type : 4;      //比赛类型
    uint8_t game_progress : 4;  //当前比赛阶段
    uint16_t stage_remain_time; //当前阶段剩余时间，单位：秒
    uint64_t SyncTimeStamp;     //UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效

}game_status_t;

/*0x0002*/
typedef struct __packed
{
    uint8_t winner;     //获胜方

}game_result_t;

/*0x0003*/
typedef struct __packed
{

		uint16_t ally_1_robot_HP; //己方 1 号英雄机器人血量，若该机器人未上场或者被罚下，则血量为 0，下文同理
		uint16_t ally_2_robot_HP; //己方 2 号工程机器人血量
		uint16_t ally_3_robot_HP; //己方 3 号步兵机器人血量
		uint16_t ally_4_robot_HP; //己方 4 号步兵机器人血量
		uint16_t reserved; 				//保留位
		uint16_t ally_7_robot_HP; //己方 7 号哨兵机器人血量
		uint16_t ally_outpost_HP; //己方前哨站血量
		uint16_t ally_base_HP;  	//己方基地血量

}game_robot_HP_t;

/*0x0101*/
typedef struct __packed
{
    uint32_t bit0 : 1;           //己方补给站前补血点的占领状态，1 为已占领
    uint32_t bit1 : 1;              //己方与兑换区重叠的补给区占领状态，1为占领
    uint32_t bit2 : 1;           //己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
    uint32_t bit3_4 : 2;            //己方小能量机关的激活状态，0 为未激活，1 为已激活，2 为正在激活
    uint32_t bit5_6 : 2;                //己方大能量机关的激活状态，0 为未激活，1 为已激活，2 为正在激活
    uint32_t bit7_8 : 2;                  //己方中央高地的占领状态，1为被己方占领，2为被对方占领
    uint32_t bit9_10 : 2;          //己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领
    uint32_t bit11_19 : 9;         //对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0）
    uint32_t bit20_22 : 3;      //对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机固定目标，4 为击中基地随机移动目标，5 为击中基地末端移动目标
    uint32_t bit23_24 : 2;            //中心增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领。（仅 RMUL 适用）
    uint32_t bit25_26 : 2;             //己方堡垒增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领
		uint32_t bit27_28 : 2;             //己方前哨站增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领
		uint32_t bit29 : 1; 								//己方基地增益点的占领状态，1 为已占领
	  uint32_t bit30_31 : 2; 				//保留位
}event_data_t;


/*0x0104*/
typedef struct __packed
{
    uint8_t level;                  //己方最后一次受到判罚的等级
    uint8_t offending_robot_id;     //己方最后一次受到判罚的违规机器人 ID
    uint8_t count;                  //己方最后一次受到判罚的违规机器人对应判罚等级的违规次数

}referee_warning_t;

/*0x0105*/
typedef struct __packed
{
    uint8_t dart_remaining_time;    //己方飞镖发射剩余时间，单位：秒
    uint16_t bit0_2 : 3;         //最近一次己方飞镖击中的目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机目标 ,4为击中基地随机移动目标
    uint16_t bit3_5 : 3;       //对方最近被击中的目标累计被击中计数，开局默认为 0，至多为 4
    uint16_t bit6_8 : 3;        //飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为 0，选中基地固定目标为 1，选中基地随机固定目标为 2，选中基地随机移动目标为 3，选中基地末端移动目标为 4
    uint16_t bit9_15 : 7;       //保留

}dart_info_t;

/*0x0201*/
typedef struct __packed
{
    uint8_t robot_id;                               //本机器人 ID
    uint8_t robot_level;                            //机器人等级
    uint16_t current_HP;                            //机器人当前血量
    uint16_t maximum_HP;                            //机器人血量上限
    uint16_t shooter_barrel_cooling_value;          //机器人枪口热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;             //机器人枪口热量上限
    uint16_t chassis_power_limit;                   //机器人底盘功率上限
    uint8_t power_management_gimbal_output : 1;     //gimbal 口输出：0 为无输出，1 为 24V 输出
    uint8_t power_management_chassis_output : 1;    //chassis 口输出：0 为无输出，1 为 24V 输出
    uint8_t power_management_shooter_output : 1;    //shooter 口输出：0 为无输出，1 为 24V 输出

}robot_status_t;

/*0x0202*/
typedef struct __packed
{
    uint16_t chassis_voltage;               //保留位
    uint16_t chassis_current;               //保留位
    float chassis_power;                    //保留位
    uint16_t buffer_energy;                 //缓冲能量（单位：J）
    uint16_t shooter_17mm_1_barrel_heat;    //第 1 个 17mm 发射机构的枪口热量
    uint16_t shooter_17mm_2_barrel_heat;    //第 2 个 17mm 发射机构的枪口热量
    uint16_t shooter_42mm_barrel_heat;      //42mm 发射机构的枪口热量

}power_heat_data_t;

/*0x0203*/
typedef struct __packed
{
    float x;        //本机器人位置 x 坐标，单位：m
    float y;        //本机器人位置 y 坐标，单位：m
    float angle;    //本机器人测速模块的朝向，单位：度。正北为 0 度

}robot_pos_t;

/*0x0204*/
typedef struct __packed
{
    uint8_t recovery_buff;      //机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
    uint16_t cooling_buff;       //机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s）
    uint8_t defence_buff;       //机器人防御增益（百分比，值为 50 表示 50%防御增益）
    uint8_t vulnerability_buff; //机器人负防御增益（百分比，值为 30 表示-30%防御增益）
    uint16_t attack_buff;       //机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
		uint8_t bit0 :1;						//在剩余能量≥125%时为 1，其余情况为 0
	  uint8_t bit1 :1;						//在剩余能量≥100%时为 1，其余情况为 0
		uint8_t bit2 :1;						//在剩余能量≥50%时为 1，其余情况为 0
		uint8_t bit3 :1;						//在剩余能量≥30%时为 1，其余情况为 0
		uint8_t bit4 :1;						//在剩余能量≥15%时为 1，其余情况为 0
		uint8_t bit5 :1;						//在剩余能量≥5%时为 1，其余情况为 0
		uint8_t bit6 :1;						//在剩余能量≥1%时为 1，其余情况为 0

}buff_t;

///*0x0205*/
//typedef struct __packed
//{
//    uint8_t airforce_status;    //空中机器人状态（0 为正在冷却，1 为冷却完毕，2 为正在空中支援）
//    uint8_t time_remain;        //此状态的剩余时间

//}air_support_data_t;

/*0x0206*/
typedef struct __packed
{
    uint8_t armor_id : 4;               //当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0
	uint8_t HP_deduction_reason : 4;    //血量变化类型： 0：装甲模块被弹丸攻击导致扣血 1：装甲模块或超级电容管理模块离线导致扣血 5：装甲模块受到撞击导致扣血

}hurt_data_t;

/*0x0207*/
typedef struct __packed
{
    uint8_t bullet_type;            //弹丸类型bit 1：17mm 弹丸 ,bit 2：42mm 弹丸
    uint8_t shooter_number;         //发射机构 ID：1：17mm 发射机构 2：保留位 3：42mm 发射机构
    uint8_t launching_frequency;    //弹丸射速（单位：Hz）
    float initial_speed;            //弹丸初速度（单位：m/s）

}shoot_data_t;

/*0x0208*/
typedef struct __packed
{
    uint16_t projectile_allowance_17mm;     //17mm 弹丸允许发弹量
    uint16_t projectile_allowance_42mm;     //42mm 弹丸允许发弹量
    uint16_t remaining_gold_coin;           //剩余金币数量
		uint16_t projectile_allowance_fortress; //堡垒增益点提供的储备 17mm 弹丸允许发弹量； 该值与机器人是否实际占领堡垒无关

}projectile_allowance_t;

/*0x0209*/
typedef struct __packed
{
    uint32_t bit0 : 1;      //己方基地增益点
    uint32_t bit1 : 1;      //己方中央高地增益点
    uint32_t bit2 : 1;      //对方中央高地增益点
    uint32_t bit3 : 1;      //己方梯形高地增益点
    uint32_t bit4 : 1;      //对方梯形高地增益点
    uint32_t bit5 : 1;      //己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
    uint32_t bit6 : 1;      //己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
    uint32_t bit7 : 1;      //对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
    uint32_t bit8 : 1;      //对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
    uint32_t bit9 : 1;      //己方地形跨越增益点（中央高地下方）
    uint32_t bit10 : 1;     //己方地形跨越增益点（中央高地上方）
    uint32_t bit11 : 1;     //对方地形跨越增益点（中央高地下方）
    uint32_t bit12 : 1;     //对方地形跨越增益点（中央高地上方）
    uint32_t bit13 : 1;     //己方地形跨越增益点（公路下方）
	  uint32_t bit14 : 1;     //己方地形跨越增益点（公路上方）
    uint32_t bit15 : 1;     //对方地形跨越增益点（公路下方）
		uint32_t bit16 : 1;     //对方地形跨越增益点（公路上方）
    uint32_t bit17 : 1;     //己方堡垒增益点
    uint32_t bit18 : 1;     //己方前哨站增益点
    uint32_t bit19 : 1;     //己方与资源区不重叠的补给区/RMUL补给区
		uint32_t bit20 : 1;     //己方与资源区重叠的补给区
    uint32_t bit21 : 1;     //己方装配增益点
    uint32_t bit22 : 1;     //对方装配增益点
    uint32_t bit23 : 1;     //中心增益点（仅 RMUL 适用）
	  uint32_t bit24 : 1;     //对方堡垒增益点
    uint32_t bit25 : 1;     //对方前哨站增益点
		uint32_t bit26 : 1;     //己方地形跨越增益点（隧道）（靠近己方一侧公路区下方）
    uint32_t bit27 : 1;     //己方地形跨越增益点（隧道）（靠近己方一侧公路区中间）
    uint32_t bit28 : 1;     //己方地形跨越增益点（隧道）（靠近己方一侧公路区上方）
    uint32_t bit29 : 1;     //己方地形跨越增益点（隧道）（靠近己方梯形高地较低处）
		uint32_t bit30 : 1;     //己方地形跨越增益点（隧道）（靠近己方梯形高地较中间）
    uint32_t bit31 : 1;     //己方地形跨越增益点（隧道）（靠近己方梯形高地较高处）

		uint8_t bit_two_0 : 1;  //对方地形跨越增益点（隧道）（靠近对方公路一侧下方）
		uint8_t bit_two_1 : 1;  //己方地形跨越增益点（隧道）（靠近己方一侧公路区中间）
    uint8_t bit_two_2 : 1;  //己方地形跨越增益点（隧道）（靠近己方一侧公路区上方）
    uint8_t bit_two_3 : 1;  //己方地形跨越增益点（隧道）（靠近己方梯形高地较低处）
		uint8_t bit_two_4 : 1;  //己方地形跨越增益点（隧道）（靠近己方梯形高地较中间）
    uint8_t bit_two_5 : 1;  //己方地形跨越增益点（隧道）（靠近己方梯形高地较高处）

}rfid_status_t;

/*0x020A*/
typedef struct __packed
{
    uint8_t dart_launch_opening_status;     //当前飞镖发射站的状态：1：关闭 2：正在开启或者关闭中 0：已经开启
    uint8_t reserved;                       //保留位
    uint16_t target_change_time;            //切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0。
    uint16_t latest_launch_cmd_time;        //最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为 0。

}dart_client_cmd_t;

/*0x020B*/
typedef struct __packed
{
    float hero_x;           //己方英雄机器人位置 x 轴坐标，单位：m
    float hero_y;           //己方英雄机器人位置 y 轴坐标，单位：m
    float engineer_x;       //己方工程机器人位置 x 轴坐标，单位：m
    float engineer_y;       //己方工程机器人位置 y 轴坐标，单位：m
    float standard_3_x;     //己方 3 号步兵机器人位置 x 轴坐标，单位：m
    float standard_3_y;     //己方 3 号步兵机器人位置 y 轴坐标，单位：m
    float standard_4_x;     //己方 4 号步兵机器人位置 x 轴坐标，单位：m
    float standard_4_y;     //己方 4 号步兵机器人位置 y 轴坐标，单位：m
    float standard_5_x;     //保留位
    float standard_5_y;     //保留位

}ground_robot_position_t;

/*0x020C*/
typedef struct __packed
{
		uint16_t mark_hero_progress:1;         //对方英雄机器人被标记进度：0-120
    uint16_t mark_engineer_progress:1;     //对方工程机器人被标记进度：0-120
    uint16_t mark_standard_3_progress:1;   //对方 3 号步兵机器人被标记进度：0-120
    uint16_t mark_standard_4_progress:1;   //对方 4 号步兵机器人被标记进度：0-120
    uint16_t mark_air_progress:1;   //对方空中机器人被标记进度：0-120
    uint16_t mark_sentry_progress:1;       //对方哨兵机器人被标记进度：0-120
	  uint16_t bit6:1;         //己方 1 号英雄机器人特殊标识情况
    uint16_t bit7:1;     //己方 2 号工程机器人特殊标识情况
    uint16_t bit8:1;   //己方 3 号步兵机器人特殊标识情况
    uint16_t bit9:1;   //己方 4 号步兵机器人特殊标识情况
    uint16_t bit10:1;   //己方空中机器人特殊标识情况
    uint16_t bit11:1;       //己方哨兵机器人特殊标识情况
	uint16_t bit12_15:4; //保留位
}radar_mark_data_t;

/*0x020D*/
typedef struct __packed
{
    uint32_t successfully_redeemed : 11;        //除远程兑换外，哨兵成功兑换的发弹量，开局为 0，在哨兵成功兑换一定发弹量后，该值将变为哨兵成功兑换的发弹量值。
    uint32_t remote_successfully_redeemed : 4;  //哨兵成功远程兑换发弹量的次数，开局为 0，在哨兵成功远程兑换发弹量后，该值将变为哨兵成功远程兑换发弹量的次数。
    uint32_t remote_HP : 4;                     //哨兵成功远程兑换血量的次数，开局为 0，在哨兵成功远程兑换血量后，该值将变为哨兵成功远程兑换血量的次数。
    uint32_t remote_free_revival: 1;            //哨兵当前是否可以确认免费复活，可以确认免费复活时值为1，否则为0
    uint32_t remote_exchange_revival : 1;       //哨兵当前是否兑换立即复活，可以兑换立即复活时值为1，否则为0
    uint32_t cost_gold_coin : 10;               //哨兵当前若兑换立即复活花费的金币数
    uint32_t Reserved_bits : 1;                 //保留

    uint16_t sentry_fighting : 1;                 //哨兵是否处于战斗状态，处于脱战状态为1，否则为0
    uint16_t shoot_17_exchange : 11;              //队伍17mm允许发弹量的剩余可兑换数
    uint16_t sentry_form : 2;                     //哨兵当前姿态，1 为进攻姿态，2 为防御姿态，3 为移动姿态
	  uint16_t Energy_mechanism_state :1; 					//己方能量机关是否能够进入正在激活状态，1 为当前可激活
		uint16_t bit15 :1;                            //保留位
}sentry_info_t;

/*0x020E*/
typedef struct __packed
{
    uint8_t vulnerable_begin : 2;   //雷达是否拥有触发双倍易伤的机会，开局为 0，数值为雷达拥有触发双倍易伤的机会，至多为 2
    uint8_t vulnerable_now : 1;     //对方是否正在被触发双，0：对方未被触发双倍易伤 1：对方正在被触发双倍易伤
	  uint8_t bit3_4 : 2;							//己方加密等级（即对方干扰波难度等级），开局为 1，最高为 3
	  uint8_t bit5 :1;								//当前是否可以修改密钥，1 为可修改
		uint8_t bit6_7 : 2;             //保留位
}radar_info_t;


/*0x0301*/
typedef  struct __packed
{
  uint16_t data_cmd_id;  //需要开放的子内容ID
  uint16_t sender_id; //需与自身ID匹配
  uint16_t receiver_id; //接收者ID
  uint8_t user_data[112];
}robot_interaction_data_t;

/*0x0100*/
typedef struct __packed
{
uint8_t delete_type;
uint8_t layer;
}interaction_layer_delete_t;

/*0x0101*/
typedef  struct __packed
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
}interaction_figure_t;

/*0x0102*/
typedef struct __packed
{
 interaction_figure_t interaction_figure[2];
}interaction_figure_2_t;

/*0x0103*/
typedef struct __packed
{
interaction_figure_t interaction_figure[5];
}interaction_figure_3_t;

/*0x0104*/
typedef struct __packed
{
interaction_figure_t interaction_figure[7];
}interaction_figure_4_t;

typedef struct __packed
{
	uint16_t data_id;
	uint16_t tx_id;
	uint16_t rx_id;
	uint8_t Character_configuration[15];
	uint8_t Character[30];
}graphic_data_struct_t;

/*0x0110*/
typedef struct __packed
{
graphic_data_struct_t grapic_data_struct;
uint8_t data[30];
} ext_client_custom_character_t;

/*0x0120*/
typedef struct __packed
{
	uint32_t bit0:1;				//哨兵机器人是否确认复活
	uint32_t bit1:1;				//哨兵机器人是否确认兑换立即复活
	uint32_t bit2_12:11;		//哨兵将要兑换的发弹量值，开局为 0，修改此值后，哨兵在补血点即可兑换允许发弹量
	uint32_t bit13_16:4;		//哨兵远程兑换发弹量的请求次数，开局为 0，修改此值即可请求远程兑换发弹量
	uint32_t bit17_20:4;		//哨兵远程兑换血量的请求次数，开局为 0，修改此值即可请求远程兑换血量
	uint32_t bit21_22:2;		//哨兵修改当前姿态指令，1 为进攻姿态，2 为防御姿态，3 为移动姿态，默认为 3；修改此值即可改变哨兵姿态。
	uint32_t bit23:1;				//哨兵机器人是否确认使能量机关进入正在激活状态，1 为确认。默认为 0。
	uint32_t bit24_31:8;    //保留位
}sentry_cmd_t;

/*0x0121*/
typedef struct __packed
{
 uint8_t radar_cmd;
 uint8_t password_cmd;
 uint8_t password_1;
 uint8_t password_2;
 uint8_t password_3;
 uint8_t password_4;
 uint8_t password_5;
 uint8_t password_6;
} radar_cmd_t;

/*0x0303*/
typedef struct __packed
{
    float target_position_x;    //目标位置 x 轴坐标，单位 m  当发送目标机器人 ID 时，该值为 0
    float target_position_y;    //目标位置 y 轴坐标，单位 m  当发送目标机器人 ID 时，该值为 0
    uint8_t cmd_keyboard;       //云台手按下的键盘按键通用键值
    uint8_t target_robot_id;    //对方机器人 ID
    uint8_t cmd_source;         //信息来源 ID

}map_command_t;

/*0x0305*/   //(雷达)
typedef struct __packed
{
uint16_t hero_position_x; //英雄机器人 x 位置坐标，单位：cm
 uint16_t hero_position_y; //英雄机器人 y 位置坐标，单位：cm
 uint16_t engineer_position_x; //工程机器人 x 位置坐标，单位：cm
 uint16_t engineer_position_y; //工程机器人 y 位置坐标，单位：cm
 uint16_t infantry_3_position_x; //3 号步兵机器人 x 位置坐标，单位：cm
 uint16_t infantry_3_position_y; //3 号步兵机器人 y 位置坐标，单位：cm
 uint16_t infantry_4_position_x; //4 号步兵机器人 x 位置坐标，单位：cm
 uint16_t infantry_4_position_y; //4 号步兵机器人 y 位置坐标，单位：cm
 uint16_t reserved1; 						 //保留位
 uint16_t reserved2; 						 //保留位
 uint16_t sentry_position_x; 		 //哨兵机器人 x 位置坐标，单位：cm
 uint16_t sentry_position_y; 		 //哨兵机器人 y 位置坐标，单位：cm
} map_robot_data_t;

/*0x0307*/
typedef struct __packed
{
uint8_t intention; 								//1：到目标点攻击 2：到目标点防守 3：移动到目标点
uint16_t start_position_x; 				//路径起点 x 轴坐标，单位：dm
uint16_t start_position_y; 				//路径起点 y 轴坐标，单位：dm
int8_t delta_x[49]; 							//路径点 x 轴增量数组，单位：dm
int8_t delta_y[49]; 							//路径点 y 轴增量数组，单位：dm
uint16_t sender_id; 							//发送者 ID
}map_data_t;

/*0x0308*/
typedef struct  __packed
{
uint16_t sender_id;
uint16_t receiver_id;
uint8_t user_data[30];
} custom_info_t;

typedef struct __packed
{
    /* data */
    game_status_t game_status;
    game_result_t game_result;
    game_robot_HP_t game_robot_HP;
    event_data_t event_data;

    referee_warning_t referee_warning;
    dart_info_t dart_info;
    robot_status_t robot_status;
    power_heat_data_t power_heat_data;
    robot_pos_t robot_pos;
    buff_t buff;
    hurt_data_t hurt_data;
    shoot_data_t shoot_data;
    projectile_allowance_t projectile_allowance;
    rfid_status_t rfid_status;
    dart_client_cmd_t dart_client_cmd;
    ground_robot_position_t ground_robot_position;
    radar_mark_data_t radar_mark_data;
    sentry_info_t sentry_info;
    radar_info_t radar_info;
    map_command_t map_command;
//    remote_control_t remote_control;

}User_Data_T;

typedef struct __packed
{
    uint8_t   SOF;				//0xA5
    uint16_t  DataLenth;	    //数据位长度
	uint8_t   Seq;				//包序号
    uint8_t   CRC8;				//crc8位校验
}frame_header_R_Typdef;

typedef union
{
	struct __packed
    {
        /* data head*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
    }RX_Data_head;

    struct __packed
    {
        /* data game_status*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        game_status_t game_status;
        uint16_t frame_tail;
    }RX_Data_game_status;

	struct __packed
    {
        /* data game_result*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        game_result_t game_result;
        uint16_t frame_tail;
    }RX_Data_game_result;

	struct __packed
    {
        /* data game_robot_HP*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        game_robot_HP_t game_robot_HP;
        uint16_t frame_tail;
    }RX_Data_game_robot_HP;

	struct __packed
    {
        /* data event_data*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        event_data_t event_data;
        uint16_t frame_tail;
    }RX_Data_event_data;

	struct __packed
    {
        /* data ext_supply_projectile_action*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        //ext_supply_projectile_action_t ext_supply_projectile_action;
        uint16_t frame_tail;
    }RX_Data_ext_supply_projectile_action;

	struct __packed
    {
        /* data referee_warning*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        referee_warning_t referee_warning;
        uint16_t frame_tail;
    }RX_Data_referee_warning;

	struct __packed
    {
        /* data dart_info*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        dart_info_t dart_info;
        uint16_t frame_tail;
    }RX_Data_dart_info;

	struct __packed
    {
        /* data robot_status*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        robot_status_t robot_status;
        uint16_t frame_tail;
    }RX_Data_robot_status;

	struct __packed
    {
        /* data power_heat_data*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        power_heat_data_t power_heat_data;
        uint16_t frame_tail;
    }RX_Data_power_heat_data;

	struct __packed
    {
        /* data robot_pos*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        robot_pos_t robot_pos;
        uint16_t frame_tail;
    }RX_Data_robot_pos;

	struct __packed
    {
        /* data buff*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        buff_t buff;
        uint16_t frame_tail;
    }RX_Data_buff;

	struct __packed
    {
        /* data air_support_data*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        //air_support_data_t air_support_data;
        uint16_t frame_tail;
    }RX_Data_air_support_data;

	struct __packed
    {
        /* data hurt_data*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        hurt_data_t hurt_data;
        uint16_t frame_tail;
    }RX_Data_hurt_data;

	struct __packed
    {
        /* data shoot_data*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        shoot_data_t shoot_data;
        uint16_t frame_tail;
    }RX_Data_shoot_data;

	struct __packed
    {
        /* data projectile_allowance*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        projectile_allowance_t projectile_allowance;
        uint16_t frame_tail;
    }RX_Data_projectile_allowance;

	struct __packed
    {
        /* data rfid_status*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        rfid_status_t rfid_status;
        uint16_t frame_tail;
    }RX_Data_rfid_status;

	struct __packed
    {
        /* data dart_client_cmd*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        dart_client_cmd_t dart_client_cmd;
        uint16_t frame_tail;
    }RX_Data_dart_client_cmd;

	struct __packed
    {
        /* data ground_robot_position*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        ground_robot_position_t ground_robot_position;
        uint16_t frame_tail;
    }RX_Data_ground_robot_position;

	struct __packed
    {
        /* data radar_mark_data*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        radar_mark_data_t radar_mark_data;
        uint16_t frame_tail;
    }RX_Data_radar_mark_data;

	struct __packed
    {
        /* data sentry_info*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        sentry_info_t sentry_info;
        uint16_t frame_tail;
    }RX_Data_sentry_info;

	struct __packed
    {
        /* data radar_info*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        radar_info_t radar_info;
        uint16_t frame_tail;
    }RX_Data_radar_info;

	struct __packed
    {
        /* data map_command*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        map_command_t map_command;
        uint16_t frame_tail;
    }RX_Data_map_command;

	struct __packed
    {
        /* data remote_control*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        //remote_control_t remote_control;
        uint16_t frame_tail;
    }RX_Data_remote_control;

			struct __packed
    {
        /* data remote_control*/
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        projectile_allowance_t robot_shoot;
        uint16_t frame_tail;
    }RX_Data_shoot_control;//发弹量

    uint8_t Data[500];

}ALL_RX_Data_T;

extern User_Data_T User_data;
void Read_Data_first(ALL_RX_Data_T *ALL_RX_Data , User_Data_T *user_data , uint16_t length);
uint16_t Read_Data_system(ALL_RX_Data_T *ALL_RX_Data , User_Data_T *user_data);

#endif //G4_FRAMEWORK_REFEREE_H