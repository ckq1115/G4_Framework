#include "Referee.h"
#include <stdbool.h>

uint8_t Referee_System_Info_MultiRx_Buf[2][REFEREE_RXFRAME_LENGTH];

static void Referee_System_Info_Update(uint16_t cmd_id, uint8_t *data_ptr, User_Data_T *usr_data);

/**
  * @brief  裁判系统数据解析（支持黏包）
  */
void Referee_System_Frame_Update(uint8_t *Buff)
{
    uint16_t index = 0;
    uint16_t data_length = 0;
    uint16_t cmd_id = 0;
    uint8_t *data_ptr;

    while (Buff[index] == 0xA5)
    {
        /* CRC8 校验帧头 */
        if (Verify_CRC8_Check_Sum(&Buff[index], FrameHeader_Length) == true)
        {
            data_length = (uint16_t)(Buff[index+2]<<8 | Buff[index+1])
                        + FrameHeader_Length
                        + CMDID_Length
                        + CRC16_Length;
            /* CRC16 校验整帧 */
            if (Verify_CRC16_Check_Sum(&Buff[index], data_length) == true)
            {
                cmd_id = (uint16_t)(Buff[index + FrameHeader_Length + 1] << 8
                                  | Buff[index + FrameHeader_Length]);

                data_ptr = &Buff[index + FrameHeader_Length + CMDID_Length];

                Referee_System_Info_Update(cmd_id, data_ptr, &User_data);
            }
            else
            {
                break;
            }
        }
        else
        {
            break;
        }
        index += data_length;
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