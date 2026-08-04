/** @file
 *    @brief MAVLink comm protocol testsuite generated from siyi.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef SIYI_TESTSUITE_H
#define SIYI_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_siyi(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_siyi(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_siyi_smart_battery_use_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_siyi_smart_battery_use_info_t packet_in = {
        963497464,17443,17547
    };
    mavlink_siyi_smart_battery_use_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.remaining_time = packet_in.remaining_time;
        packet1.recommend_return_pct = packet_in.recommend_return_pct;
        packet1.force_return_pct = packet_in.force_return_pct;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_siyi_smart_battery_use_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_siyi_smart_battery_use_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_siyi_smart_battery_use_info_pack(system_id, component_id, &msg , packet1.recommend_return_pct , packet1.force_return_pct , packet1.remaining_time );
    mavlink_msg_siyi_smart_battery_use_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_siyi_smart_battery_use_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.recommend_return_pct , packet1.force_return_pct , packet1.remaining_time );
    mavlink_msg_siyi_smart_battery_use_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_siyi_smart_battery_use_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_siyi_smart_battery_use_info_send(MAVLINK_COMM_1 , packet1.recommend_return_pct , packet1.force_return_pct , packet1.remaining_time );
    mavlink_msg_siyi_smart_battery_use_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SIYI_SMART_BATTERY_USE_INFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO) != NULL);
#endif
}

static void mavlink_test_siyi_esc_fault_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_siyi_esc_fault_status_t packet_in = {
        963497464,17443,151
    };
    mavlink_siyi_esc_fault_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.fault_flags = packet_in.fault_flags;
        packet1.esc_id = packet_in.esc_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_siyi_esc_fault_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_siyi_esc_fault_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_siyi_esc_fault_status_pack(system_id, component_id, &msg , packet1.esc_id , packet1.fault_flags , packet1.timestamp );
    mavlink_msg_siyi_esc_fault_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_siyi_esc_fault_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.esc_id , packet1.fault_flags , packet1.timestamp );
    mavlink_msg_siyi_esc_fault_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_siyi_esc_fault_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_siyi_esc_fault_status_send(MAVLINK_COMM_1 , packet1.esc_id , packet1.fault_flags , packet1.timestamp );
    mavlink_msg_siyi_esc_fault_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SIYI_ESC_FAULT_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS) != NULL);
#endif
}

static void mavlink_test_siyi_meteorological_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_siyi_meteorological_data_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,20355,20459,20563,75
    };
    mavlink_siyi_meteorological_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.air_temperature_c = packet_in.air_temperature_c;
        packet1.relative_humidity_pct = packet_in.relative_humidity_pct;
        packet1.pressure_abs_hpa = packet_in.pressure_abs_hpa;
        packet1.relative_wind_speed_m_s = packet_in.relative_wind_speed_m_s;
        packet1.visibility_m = packet_in.visibility_m;
        packet1.true_wind_speed_m_s = packet_in.true_wind_speed_m_s;
        packet1.baro_altitude_m = packet_in.baro_altitude_m;
        packet1.dust_concentration_ug_m3 = packet_in.dust_concentration_ug_m3;
        packet1.pm1_ug_m3 = packet_in.pm1_ug_m3;
        packet1.pm10_ug_m3 = packet_in.pm10_ug_m3;
        packet1.uv_w_m2 = packet_in.uv_w_m2;
        packet1.illuminance_lux = packet_in.illuminance_lux;
        packet1.daily_radiation_kj = packet_in.daily_radiation_kj;
        packet1.solar_radiation_w_m2 = packet_in.solar_radiation_w_m2;
        packet1.relative_wind_dir_deg = packet_in.relative_wind_dir_deg;
        packet1.compass_hdg_deg = packet_in.compass_hdg_deg;
        packet1.true_wind_dir_deg = packet_in.true_wind_dir_deg;
        packet1.uv_index = packet_in.uv_index;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_siyi_meteorological_data_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_siyi_meteorological_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_siyi_meteorological_data_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.air_temperature_c , packet1.relative_humidity_pct , packet1.pressure_abs_hpa , packet1.relative_wind_dir_deg , packet1.relative_wind_speed_m_s , packet1.compass_hdg_deg , packet1.visibility_m , packet1.true_wind_speed_m_s , packet1.true_wind_dir_deg , packet1.baro_altitude_m , packet1.dust_concentration_ug_m3 , packet1.pm1_ug_m3 , packet1.pm10_ug_m3 , packet1.uv_w_m2 , packet1.uv_index , packet1.illuminance_lux , packet1.daily_radiation_kj , packet1.solar_radiation_w_m2 );
    mavlink_msg_siyi_meteorological_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_siyi_meteorological_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.air_temperature_c , packet1.relative_humidity_pct , packet1.pressure_abs_hpa , packet1.relative_wind_dir_deg , packet1.relative_wind_speed_m_s , packet1.compass_hdg_deg , packet1.visibility_m , packet1.true_wind_speed_m_s , packet1.true_wind_dir_deg , packet1.baro_altitude_m , packet1.dust_concentration_ug_m3 , packet1.pm1_ug_m3 , packet1.pm10_ug_m3 , packet1.uv_w_m2 , packet1.uv_index , packet1.illuminance_lux , packet1.daily_radiation_kj , packet1.solar_radiation_w_m2 );
    mavlink_msg_siyi_meteorological_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_siyi_meteorological_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_siyi_meteorological_data_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.air_temperature_c , packet1.relative_humidity_pct , packet1.pressure_abs_hpa , packet1.relative_wind_dir_deg , packet1.relative_wind_speed_m_s , packet1.compass_hdg_deg , packet1.visibility_m , packet1.true_wind_speed_m_s , packet1.true_wind_dir_deg , packet1.baro_altitude_m , packet1.dust_concentration_ug_m3 , packet1.pm1_ug_m3 , packet1.pm10_ug_m3 , packet1.uv_w_m2 , packet1.uv_index , packet1.illuminance_lux , packet1.daily_radiation_kj , packet1.solar_radiation_w_m2 );
    mavlink_msg_siyi_meteorological_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SIYI_METEOROLOGICAL_DATA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA) != NULL);
#endif
}

static void mavlink_test_siyi(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_siyi_smart_battery_use_info(system_id, component_id, last_msg);
    mavlink_test_siyi_esc_fault_status(system_id, component_id, last_msg);
    mavlink_test_siyi_meteorological_data(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SIYI_TESTSUITE_H
