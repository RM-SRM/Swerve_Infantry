#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP;
ext_dart_status_t dart_status;
ext_ICRA_buff_debuff_zone_status_t ICRA_buff_debuff_zone_status;

ext_event_data_t event_data;
ext_supply_projectile_action_t supply_projectile_action;
ext_referee_warning_t referee_warning;
ext_dart_remaining_time_t dart_remaining_time;

ext_game_robot_status_t game_robot_status;
ext_power_heat_data_t power_heat_data;
ext_game_robot_pos_t game_robot_pos;
ext_buff_t buff_t;
aerial_robot_energy_t robot_energy;
ext_robot_hurt_t robot_hurt;

ext_shoot_data_t shoot_data;
ext_bullet_remaining_t bullet_remaining;
ext_rfid_status_t rfid_status;
ext_dart_client_cmd_t dart_client_cmd;

ext_student_interactive_header_data_t student_interactive_header_data;
robot_interactive_data_t robot_interactive_data;

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP, 0, sizeof(ext_game_robot_HP_t));
    memset(&dart_status,0,sizeof(ext_dart_status_t));
    memset(&ICRA_buff_debuff_zone_status,0,sizeof(ext_ICRA_buff_debuff_zone_status_t));

    memset(&event_data, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action, 0, sizeof(ext_supply_projectile_action_t));
    memset(&referee_warning, 0, sizeof(ext_referee_warning_t));
    memset(&dart_remaining_time,0,sizeof(ext_dart_remaining_time_t));
    memset(&game_robot_status, 0, sizeof(ext_game_robot_status_t));
    
    memset(&power_heat_data, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_t, 0, sizeof(ext_buff_t));
    memset(&robot_energy, 0, sizeof(aerial_robot_energy_t));
    memset(&robot_hurt, 0, sizeof(ext_robot_hurt_t));

    memset(&shoot_data, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining, 0, sizeof(ext_bullet_remaining_t));
    memset(&rfid_status,0,sizeof(ext_rfid_status_t));
    memset(&dart_client_cmd,0,sizeof(ext_dart_client_cmd_t));

    memset(&student_interactive_header_data, 0, sizeof(ext_student_interactive_header_data_t));
    memset(&robot_interactive_data,0,sizeof(robot_interactive_data_t));
}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        //0x00
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;
        case DART_STATUS_CMD_ID:
        {
            memcpy(&dart_status, frame + index, sizeof(ext_dart_status_t));
        }break;
        case ICRA_BUFF_DEBUFF_ZONE_STATUS_CMD_ID:
        {
            memcpy(&ICRA_buff_debuff_zone_status, frame + index, sizeof(ICRA_buff_debuff_zone_status));
        }break;
        //0x01
        case EVENT_DATA_CMD_ID:
        {
            memcpy(&event_data, frame + index, sizeof(event_data));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action, frame + index, sizeof(supply_projectile_action));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning, frame + index, sizeof(referee_warning));
        }
        break;
        case DART_REMAINING_TIME_CMD_ID:
        {
            memcpy(&dart_remaining_time, frame + index, sizeof(dart_remaining_time));
        }break;

        //0x02
        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&game_robot_status, frame + index, sizeof(game_robot_status));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos, frame + index, sizeof(game_robot_pos));
        }
        break;
        case BUFF_CMD_ID:
        {
            memcpy(&buff_t, frame + index, sizeof(buff_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy, frame + index, sizeof(robot_energy));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt, frame + index, sizeof(robot_hurt));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data, frame + index, sizeof(shoot_data));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining, frame + index, sizeof(bullet_remaining));
        }
        break;
        case RFID_STATUS_CMD_ID:
        {
            memcpy(&rfid_status, frame + index, sizeof(rfid_status));
        }break;
        case DART_CLIENT_CMD_ID:
        {
            memcpy(&dart_client_cmd, frame + index, sizeof(dart_client_cmd));
        }break;
        //0x03
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_header_data, frame + index, sizeof(student_interactive_header_data));
        }break;
        case ROBOT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&robot_interactive_data, frame + index, sizeof(robot_interactive_data));
        }break;
        default:
        {
            break;
        }
    }
}

void get_chassis_power_and_buffer(fp32 *power, uint16_t *buffer)
{
    *power = power_heat_data.chassis_power;
    *buffer = power_heat_data.chassis_power_buffer;
}


void get_chassis_maxpower(uint16_t *maxpower)
{
    *maxpower = game_robot_status.chassis_power_limit;
}



uint8_t get_robot_id(void)
{
    return game_robot_status.robot_id;
}

uint8_t get_robot_level(void)
{
    return game_robot_status.robot_level;
}

uint8_t get_robot_HP(void)
{
    return game_robot_status.remain_HP;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = game_robot_status.shooter_id1_17mm_cooling_limit;
    *heat0 = power_heat_data.shooter_id1_17mm_cooling_heat;
}

void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = game_robot_status.shooter_id2_17mm_cooling_limit;
    *heat1 = power_heat_data.shooter_id2_17mm_cooling_heat;
}

void get_robot_shoot_speed_limit(uint16_t *shoot_speed_limit)
{
    *shoot_speed_limit = game_robot_status.shooter_id1_17mm_speed_limit;
}

void get_robot_shoot_cooling(uint16_t *cooling_rate, uint16_t *cooling_limit, uint16_t *cooling_data_now)
{
    *cooling_rate = game_robot_status.shooter_id1_17mm_cooling_rate;
    *cooling_limit = game_robot_status.shooter_id1_17mm_cooling_limit;
    *cooling_data_now = power_heat_data.shooter_id1_17mm_cooling_heat;
}

float get_robot_shoot_speed(void)
{
 return shoot_data.bullet_speed;
}



