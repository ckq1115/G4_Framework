#include "Referee.h"
#include <stdbool.h>

static void Referee_System_Info_Update(uint16_t cmd_id, uint8_t *data_ptr, User_Data_T *usr_data);

/**
  * @brief  裁判系统数据解析
  */
void Referee_System_Frame_Update(uint8_t *Buff, uint16_t Size)
{
    uint16_t i = 0;
    uint16_t data_length = 0;
    uint16_t cmd_id = 0;
    uint8_t *data_ptr;
    // 遍历整个接收到的缓冲区，寻找帧头 0xA5
    for (i = 0; i < Size - FrameHeader_Length; )
    {
        if (Buff[i] == 0xA5)
        {
            // CRC8 校验帧头
            if (Verify_CRC8_Check_Sum(&Buff[i], FrameHeader_Length) == true)
            {
                // 获取该帧的总长度
                data_length = (uint16_t)(Buff[i+2] << 8 | Buff[i+1])
                            + FrameHeader_Length
                            + CMDID_Length
                            + CRC16_Length;
                // 防护：检查解析出的长度是否合法，防止内存越界
                if (i + data_length <= Size)
                {
                    // CRC16 校验整帧
                    if (Verify_CRC16_Check_Sum(&Buff[i], data_length) == true)
                    {
                        // 解析命令码
                        cmd_id = (uint16_t)(Buff[i + FrameHeader_Length + 1] << 8
                                          | Buff[i + FrameHeader_Length]);

                        // 指向负载数据起始地址
                        data_ptr = &Buff[i + FrameHeader_Length + CMDID_Length];
                        // 更新数据结构体
                        Referee_System_Info_Update(cmd_id, data_ptr, &User_data);
                        // 校验成功，跳过当前整帧长度，继续寻找下一帧
                        i += data_length;
                        continue;
                    }
                }
            }
        }
        // 如果不是 0xA5，或者校验失败，则只向后挪动 1 字节寻找下一个起始符
        i++;
    }
}

static void Referee_System_Info_Update(uint16_t cmd_id, uint8_t *data_ptr, User_Data_T *usr_data)
{
    switch (cmd_id)
    {
        case game_state:
            memcpy(&usr_data->game_status, data_ptr, sizeof(game_status_t));
            break;

        case Match_results:
            memcpy(&usr_data->game_result, data_ptr, sizeof(game_result_t));
            break;

        case Robot_HP:
            memcpy(&usr_data->game_robot_HP, data_ptr, sizeof(game_robot_HP_t));
            break;

        case Venue_Events:
            memcpy(&usr_data->event_data, data_ptr, sizeof(event_data_t));
            break;

        case Referee_warning:
            memcpy(&usr_data->referee_warning, data_ptr, sizeof(referee_warning_t));
            break;

        case Dart_fire:
            memcpy(&usr_data->dart_info, data_ptr, sizeof(dart_info_t));
            break;

        case Robot_performan:
            memcpy(&usr_data->robot_status, data_ptr, sizeof(robot_status_t));
            break;

        case time_power:
            memcpy(&usr_data->power_heat_data, data_ptr, sizeof(power_heat_data_t));
            break;

        case Robot_location:
            memcpy(&usr_data->robot_pos, data_ptr, sizeof(robot_pos_t));
            break;

        case Robot_buff:
            memcpy(&usr_data->buff, data_ptr, sizeof(buff_t));
            break;

        case Damage_status:
            memcpy(&usr_data->hurt_data, data_ptr, sizeof(hurt_data_t));
            break;

        case time_shooting:
            memcpy(&usr_data->shoot_data, data_ptr, sizeof(shoot_data_t));
            break;

        case Allowable_ammo:
            memcpy(&usr_data->projectile_allowance, data_ptr, sizeof(projectile_allowance_t));
            break;

        case RFID_status:
            memcpy(&usr_data->rfid_status, data_ptr, sizeof(rfid_status_t));
            break;

        case Dart_directives:
            memcpy(&usr_data->dart_client_cmd, data_ptr, sizeof(dart_client_cmd_t));
            break;

        case Ground_location:
            memcpy(&usr_data->ground_robot_position, data_ptr, sizeof(ground_robot_position_t));
            break;

        case Radar_Marking:
            memcpy(&usr_data->radar_mark_data, data_ptr, sizeof(radar_mark_data_t));
            break;

        case Route_Informat:
            memcpy(&usr_data->sentry_info, data_ptr, sizeof(sentry_info_t));
            break;

        case Radar_Informat:
            memcpy(&usr_data->radar_info, data_ptr, sizeof(radar_info_t));
            break;

        case Minimap:
            memcpy(&usr_data->map_command, data_ptr, sizeof(map_command_t));
            break;

        default:
            break;
    }
}