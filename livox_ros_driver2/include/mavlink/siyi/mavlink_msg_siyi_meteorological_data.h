#pragma once
// MESSAGE SIYI_METEOROLOGICAL_DATA PACKING

#define MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA 14801


typedef struct __mavlink_siyi_meteorological_data_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float air_temperature_c; /*< [degC] Air temperature; NaN if unknown.*/
 float relative_humidity_pct; /*< [%] Relative humidity 0-100; NaN if unknown.*/
 float pressure_abs_hpa; /*< [hPa] Absolute air pressure; NaN if unknown.*/
 float relative_wind_speed_m_s; /*< [m/s] Relative wind speed.*/
 float visibility_m; /*< [m] Meteorological visibility.*/
 float true_wind_speed_m_s; /*< [m/s] True wind speed; NaN if unknown.*/
 float baro_altitude_m; /*< [m] Barometric / pressure altitude if not same as GPS MSL; NaN if unknown.*/
 float dust_concentration_ug_m3; /*< [ug/m3] Dust concentration.*/
 float pm1_ug_m3; /*< [ug/m3] PM1.0.*/
 float pm10_ug_m3; /*< [ug/m3] PM10.*/
 float uv_w_m2; /*< [W/m2] UV irradiance.*/
 float illuminance_lux; /*< [lux] Illuminance.*/
 float daily_radiation_kj; /*< [kJ] Daily accumulated solar radiation energy.*/
 float solar_radiation_w_m2; /*< [W/m2] Solar radiation power density.*/
 uint16_t relative_wind_dir_deg; /*< [deg] Relative wind direction (0-359).*/
 uint16_t compass_hdg_deg; /*< [cdeg] Electronic compass heading 0-35999 (NOT COG); 65535 if unknown.*/
 uint16_t true_wind_dir_deg; /*< [deg] True wind direction (0-359).*/
 uint8_t uv_index; /*<  UV index (0-255); 255=invalid.*/
} mavlink_siyi_meteorological_data_t;

#define MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN 67
#define MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_MIN_LEN 67
#define MAVLINK_MSG_ID_14801_LEN 67
#define MAVLINK_MSG_ID_14801_MIN_LEN 67

#define MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_CRC 163
#define MAVLINK_MSG_ID_14801_CRC 163



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SIYI_METEOROLOGICAL_DATA { \
    14801, \
    "SIYI_METEOROLOGICAL_DATA", \
    19, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_siyi_meteorological_data_t, time_boot_ms) }, \
         { "air_temperature_c", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_siyi_meteorological_data_t, air_temperature_c) }, \
         { "relative_humidity_pct", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_siyi_meteorological_data_t, relative_humidity_pct) }, \
         { "pressure_abs_hpa", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_siyi_meteorological_data_t, pressure_abs_hpa) }, \
         { "relative_wind_dir_deg", NULL, MAVLINK_TYPE_UINT16_T, 0, 60, offsetof(mavlink_siyi_meteorological_data_t, relative_wind_dir_deg) }, \
         { "relative_wind_speed_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_siyi_meteorological_data_t, relative_wind_speed_m_s) }, \
         { "compass_hdg_deg", NULL, MAVLINK_TYPE_UINT16_T, 0, 62, offsetof(mavlink_siyi_meteorological_data_t, compass_hdg_deg) }, \
         { "visibility_m", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_siyi_meteorological_data_t, visibility_m) }, \
         { "true_wind_speed_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_siyi_meteorological_data_t, true_wind_speed_m_s) }, \
         { "true_wind_dir_deg", NULL, MAVLINK_TYPE_UINT16_T, 0, 64, offsetof(mavlink_siyi_meteorological_data_t, true_wind_dir_deg) }, \
         { "baro_altitude_m", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_siyi_meteorological_data_t, baro_altitude_m) }, \
         { "dust_concentration_ug_m3", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_siyi_meteorological_data_t, dust_concentration_ug_m3) }, \
         { "pm1_ug_m3", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_siyi_meteorological_data_t, pm1_ug_m3) }, \
         { "pm10_ug_m3", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_siyi_meteorological_data_t, pm10_ug_m3) }, \
         { "uv_w_m2", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_siyi_meteorological_data_t, uv_w_m2) }, \
         { "uv_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 66, offsetof(mavlink_siyi_meteorological_data_t, uv_index) }, \
         { "illuminance_lux", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_siyi_meteorological_data_t, illuminance_lux) }, \
         { "daily_radiation_kj", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_siyi_meteorological_data_t, daily_radiation_kj) }, \
         { "solar_radiation_w_m2", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_siyi_meteorological_data_t, solar_radiation_w_m2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SIYI_METEOROLOGICAL_DATA { \
    "SIYI_METEOROLOGICAL_DATA", \
    19, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_siyi_meteorological_data_t, time_boot_ms) }, \
         { "air_temperature_c", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_siyi_meteorological_data_t, air_temperature_c) }, \
         { "relative_humidity_pct", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_siyi_meteorological_data_t, relative_humidity_pct) }, \
         { "pressure_abs_hpa", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_siyi_meteorological_data_t, pressure_abs_hpa) }, \
         { "relative_wind_dir_deg", NULL, MAVLINK_TYPE_UINT16_T, 0, 60, offsetof(mavlink_siyi_meteorological_data_t, relative_wind_dir_deg) }, \
         { "relative_wind_speed_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_siyi_meteorological_data_t, relative_wind_speed_m_s) }, \
         { "compass_hdg_deg", NULL, MAVLINK_TYPE_UINT16_T, 0, 62, offsetof(mavlink_siyi_meteorological_data_t, compass_hdg_deg) }, \
         { "visibility_m", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_siyi_meteorological_data_t, visibility_m) }, \
         { "true_wind_speed_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_siyi_meteorological_data_t, true_wind_speed_m_s) }, \
         { "true_wind_dir_deg", NULL, MAVLINK_TYPE_UINT16_T, 0, 64, offsetof(mavlink_siyi_meteorological_data_t, true_wind_dir_deg) }, \
         { "baro_altitude_m", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_siyi_meteorological_data_t, baro_altitude_m) }, \
         { "dust_concentration_ug_m3", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_siyi_meteorological_data_t, dust_concentration_ug_m3) }, \
         { "pm1_ug_m3", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_siyi_meteorological_data_t, pm1_ug_m3) }, \
         { "pm10_ug_m3", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_siyi_meteorological_data_t, pm10_ug_m3) }, \
         { "uv_w_m2", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_siyi_meteorological_data_t, uv_w_m2) }, \
         { "uv_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 66, offsetof(mavlink_siyi_meteorological_data_t, uv_index) }, \
         { "illuminance_lux", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_siyi_meteorological_data_t, illuminance_lux) }, \
         { "daily_radiation_kj", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_siyi_meteorological_data_t, daily_radiation_kj) }, \
         { "solar_radiation_w_m2", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_siyi_meteorological_data_t, solar_radiation_w_m2) }, \
         } \
}
#endif

/**
 * @brief Pack a siyi_meteorological_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param air_temperature_c [degC] Air temperature; NaN if unknown.
 * @param relative_humidity_pct [%] Relative humidity 0-100; NaN if unknown.
 * @param pressure_abs_hpa [hPa] Absolute air pressure; NaN if unknown.
 * @param relative_wind_dir_deg [deg] Relative wind direction (0-359).
 * @param relative_wind_speed_m_s [m/s] Relative wind speed.
 * @param compass_hdg_deg [cdeg] Electronic compass heading 0-35999 (NOT COG); 65535 if unknown.
 * @param visibility_m [m] Meteorological visibility.
 * @param true_wind_speed_m_s [m/s] True wind speed; NaN if unknown.
 * @param true_wind_dir_deg [deg] True wind direction (0-359).
 * @param baro_altitude_m [m] Barometric / pressure altitude if not same as GPS MSL; NaN if unknown.
 * @param dust_concentration_ug_m3 [ug/m3] Dust concentration.
 * @param pm1_ug_m3 [ug/m3] PM1.0.
 * @param pm10_ug_m3 [ug/m3] PM10.
 * @param uv_w_m2 [W/m2] UV irradiance.
 * @param uv_index  UV index (0-255); 255=invalid.
 * @param illuminance_lux [lux] Illuminance.
 * @param daily_radiation_kj [kJ] Daily accumulated solar radiation energy.
 * @param solar_radiation_w_m2 [W/m2] Solar radiation power density.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_siyi_meteorological_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float air_temperature_c, float relative_humidity_pct, float pressure_abs_hpa, uint16_t relative_wind_dir_deg, float relative_wind_speed_m_s, uint16_t compass_hdg_deg, float visibility_m, float true_wind_speed_m_s, uint16_t true_wind_dir_deg, float baro_altitude_m, float dust_concentration_ug_m3, float pm1_ug_m3, float pm10_ug_m3, float uv_w_m2, uint8_t uv_index, float illuminance_lux, float daily_radiation_kj, float solar_radiation_w_m2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, air_temperature_c);
    _mav_put_float(buf, 8, relative_humidity_pct);
    _mav_put_float(buf, 12, pressure_abs_hpa);
    _mav_put_float(buf, 16, relative_wind_speed_m_s);
    _mav_put_float(buf, 20, visibility_m);
    _mav_put_float(buf, 24, true_wind_speed_m_s);
    _mav_put_float(buf, 28, baro_altitude_m);
    _mav_put_float(buf, 32, dust_concentration_ug_m3);
    _mav_put_float(buf, 36, pm1_ug_m3);
    _mav_put_float(buf, 40, pm10_ug_m3);
    _mav_put_float(buf, 44, uv_w_m2);
    _mav_put_float(buf, 48, illuminance_lux);
    _mav_put_float(buf, 52, daily_radiation_kj);
    _mav_put_float(buf, 56, solar_radiation_w_m2);
    _mav_put_uint16_t(buf, 60, relative_wind_dir_deg);
    _mav_put_uint16_t(buf, 62, compass_hdg_deg);
    _mav_put_uint16_t(buf, 64, true_wind_dir_deg);
    _mav_put_uint8_t(buf, 66, uv_index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN);
#else
    mavlink_siyi_meteorological_data_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.air_temperature_c = air_temperature_c;
    packet.relative_humidity_pct = relative_humidity_pct;
    packet.pressure_abs_hpa = pressure_abs_hpa;
    packet.relative_wind_speed_m_s = relative_wind_speed_m_s;
    packet.visibility_m = visibility_m;
    packet.true_wind_speed_m_s = true_wind_speed_m_s;
    packet.baro_altitude_m = baro_altitude_m;
    packet.dust_concentration_ug_m3 = dust_concentration_ug_m3;
    packet.pm1_ug_m3 = pm1_ug_m3;
    packet.pm10_ug_m3 = pm10_ug_m3;
    packet.uv_w_m2 = uv_w_m2;
    packet.illuminance_lux = illuminance_lux;
    packet.daily_radiation_kj = daily_radiation_kj;
    packet.solar_radiation_w_m2 = solar_radiation_w_m2;
    packet.relative_wind_dir_deg = relative_wind_dir_deg;
    packet.compass_hdg_deg = compass_hdg_deg;
    packet.true_wind_dir_deg = true_wind_dir_deg;
    packet.uv_index = uv_index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_MIN_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_CRC);
}

/**
 * @brief Pack a siyi_meteorological_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param air_temperature_c [degC] Air temperature; NaN if unknown.
 * @param relative_humidity_pct [%] Relative humidity 0-100; NaN if unknown.
 * @param pressure_abs_hpa [hPa] Absolute air pressure; NaN if unknown.
 * @param relative_wind_dir_deg [deg] Relative wind direction (0-359).
 * @param relative_wind_speed_m_s [m/s] Relative wind speed.
 * @param compass_hdg_deg [cdeg] Electronic compass heading 0-35999 (NOT COG); 65535 if unknown.
 * @param visibility_m [m] Meteorological visibility.
 * @param true_wind_speed_m_s [m/s] True wind speed; NaN if unknown.
 * @param true_wind_dir_deg [deg] True wind direction (0-359).
 * @param baro_altitude_m [m] Barometric / pressure altitude if not same as GPS MSL; NaN if unknown.
 * @param dust_concentration_ug_m3 [ug/m3] Dust concentration.
 * @param pm1_ug_m3 [ug/m3] PM1.0.
 * @param pm10_ug_m3 [ug/m3] PM10.
 * @param uv_w_m2 [W/m2] UV irradiance.
 * @param uv_index  UV index (0-255); 255=invalid.
 * @param illuminance_lux [lux] Illuminance.
 * @param daily_radiation_kj [kJ] Daily accumulated solar radiation energy.
 * @param solar_radiation_w_m2 [W/m2] Solar radiation power density.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_siyi_meteorological_data_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float air_temperature_c, float relative_humidity_pct, float pressure_abs_hpa, uint16_t relative_wind_dir_deg, float relative_wind_speed_m_s, uint16_t compass_hdg_deg, float visibility_m, float true_wind_speed_m_s, uint16_t true_wind_dir_deg, float baro_altitude_m, float dust_concentration_ug_m3, float pm1_ug_m3, float pm10_ug_m3, float uv_w_m2, uint8_t uv_index, float illuminance_lux, float daily_radiation_kj, float solar_radiation_w_m2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, air_temperature_c);
    _mav_put_float(buf, 8, relative_humidity_pct);
    _mav_put_float(buf, 12, pressure_abs_hpa);
    _mav_put_float(buf, 16, relative_wind_speed_m_s);
    _mav_put_float(buf, 20, visibility_m);
    _mav_put_float(buf, 24, true_wind_speed_m_s);
    _mav_put_float(buf, 28, baro_altitude_m);
    _mav_put_float(buf, 32, dust_concentration_ug_m3);
    _mav_put_float(buf, 36, pm1_ug_m3);
    _mav_put_float(buf, 40, pm10_ug_m3);
    _mav_put_float(buf, 44, uv_w_m2);
    _mav_put_float(buf, 48, illuminance_lux);
    _mav_put_float(buf, 52, daily_radiation_kj);
    _mav_put_float(buf, 56, solar_radiation_w_m2);
    _mav_put_uint16_t(buf, 60, relative_wind_dir_deg);
    _mav_put_uint16_t(buf, 62, compass_hdg_deg);
    _mav_put_uint16_t(buf, 64, true_wind_dir_deg);
    _mav_put_uint8_t(buf, 66, uv_index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN);
#else
    mavlink_siyi_meteorological_data_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.air_temperature_c = air_temperature_c;
    packet.relative_humidity_pct = relative_humidity_pct;
    packet.pressure_abs_hpa = pressure_abs_hpa;
    packet.relative_wind_speed_m_s = relative_wind_speed_m_s;
    packet.visibility_m = visibility_m;
    packet.true_wind_speed_m_s = true_wind_speed_m_s;
    packet.baro_altitude_m = baro_altitude_m;
    packet.dust_concentration_ug_m3 = dust_concentration_ug_m3;
    packet.pm1_ug_m3 = pm1_ug_m3;
    packet.pm10_ug_m3 = pm10_ug_m3;
    packet.uv_w_m2 = uv_w_m2;
    packet.illuminance_lux = illuminance_lux;
    packet.daily_radiation_kj = daily_radiation_kj;
    packet.solar_radiation_w_m2 = solar_radiation_w_m2;
    packet.relative_wind_dir_deg = relative_wind_dir_deg;
    packet.compass_hdg_deg = compass_hdg_deg;
    packet.true_wind_dir_deg = true_wind_dir_deg;
    packet.uv_index = uv_index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_MIN_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_MIN_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN);
#endif
}

/**
 * @brief Pack a siyi_meteorological_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param air_temperature_c [degC] Air temperature; NaN if unknown.
 * @param relative_humidity_pct [%] Relative humidity 0-100; NaN if unknown.
 * @param pressure_abs_hpa [hPa] Absolute air pressure; NaN if unknown.
 * @param relative_wind_dir_deg [deg] Relative wind direction (0-359).
 * @param relative_wind_speed_m_s [m/s] Relative wind speed.
 * @param compass_hdg_deg [cdeg] Electronic compass heading 0-35999 (NOT COG); 65535 if unknown.
 * @param visibility_m [m] Meteorological visibility.
 * @param true_wind_speed_m_s [m/s] True wind speed; NaN if unknown.
 * @param true_wind_dir_deg [deg] True wind direction (0-359).
 * @param baro_altitude_m [m] Barometric / pressure altitude if not same as GPS MSL; NaN if unknown.
 * @param dust_concentration_ug_m3 [ug/m3] Dust concentration.
 * @param pm1_ug_m3 [ug/m3] PM1.0.
 * @param pm10_ug_m3 [ug/m3] PM10.
 * @param uv_w_m2 [W/m2] UV irradiance.
 * @param uv_index  UV index (0-255); 255=invalid.
 * @param illuminance_lux [lux] Illuminance.
 * @param daily_radiation_kj [kJ] Daily accumulated solar radiation energy.
 * @param solar_radiation_w_m2 [W/m2] Solar radiation power density.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_siyi_meteorological_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float air_temperature_c,float relative_humidity_pct,float pressure_abs_hpa,uint16_t relative_wind_dir_deg,float relative_wind_speed_m_s,uint16_t compass_hdg_deg,float visibility_m,float true_wind_speed_m_s,uint16_t true_wind_dir_deg,float baro_altitude_m,float dust_concentration_ug_m3,float pm1_ug_m3,float pm10_ug_m3,float uv_w_m2,uint8_t uv_index,float illuminance_lux,float daily_radiation_kj,float solar_radiation_w_m2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, air_temperature_c);
    _mav_put_float(buf, 8, relative_humidity_pct);
    _mav_put_float(buf, 12, pressure_abs_hpa);
    _mav_put_float(buf, 16, relative_wind_speed_m_s);
    _mav_put_float(buf, 20, visibility_m);
    _mav_put_float(buf, 24, true_wind_speed_m_s);
    _mav_put_float(buf, 28, baro_altitude_m);
    _mav_put_float(buf, 32, dust_concentration_ug_m3);
    _mav_put_float(buf, 36, pm1_ug_m3);
    _mav_put_float(buf, 40, pm10_ug_m3);
    _mav_put_float(buf, 44, uv_w_m2);
    _mav_put_float(buf, 48, illuminance_lux);
    _mav_put_float(buf, 52, daily_radiation_kj);
    _mav_put_float(buf, 56, solar_radiation_w_m2);
    _mav_put_uint16_t(buf, 60, relative_wind_dir_deg);
    _mav_put_uint16_t(buf, 62, compass_hdg_deg);
    _mav_put_uint16_t(buf, 64, true_wind_dir_deg);
    _mav_put_uint8_t(buf, 66, uv_index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN);
#else
    mavlink_siyi_meteorological_data_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.air_temperature_c = air_temperature_c;
    packet.relative_humidity_pct = relative_humidity_pct;
    packet.pressure_abs_hpa = pressure_abs_hpa;
    packet.relative_wind_speed_m_s = relative_wind_speed_m_s;
    packet.visibility_m = visibility_m;
    packet.true_wind_speed_m_s = true_wind_speed_m_s;
    packet.baro_altitude_m = baro_altitude_m;
    packet.dust_concentration_ug_m3 = dust_concentration_ug_m3;
    packet.pm1_ug_m3 = pm1_ug_m3;
    packet.pm10_ug_m3 = pm10_ug_m3;
    packet.uv_w_m2 = uv_w_m2;
    packet.illuminance_lux = illuminance_lux;
    packet.daily_radiation_kj = daily_radiation_kj;
    packet.solar_radiation_w_m2 = solar_radiation_w_m2;
    packet.relative_wind_dir_deg = relative_wind_dir_deg;
    packet.compass_hdg_deg = compass_hdg_deg;
    packet.true_wind_dir_deg = true_wind_dir_deg;
    packet.uv_index = uv_index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_MIN_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_CRC);
}

/**
 * @brief Encode a siyi_meteorological_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param siyi_meteorological_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_siyi_meteorological_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_siyi_meteorological_data_t* siyi_meteorological_data)
{
    return mavlink_msg_siyi_meteorological_data_pack(system_id, component_id, msg, siyi_meteorological_data->time_boot_ms, siyi_meteorological_data->air_temperature_c, siyi_meteorological_data->relative_humidity_pct, siyi_meteorological_data->pressure_abs_hpa, siyi_meteorological_data->relative_wind_dir_deg, siyi_meteorological_data->relative_wind_speed_m_s, siyi_meteorological_data->compass_hdg_deg, siyi_meteorological_data->visibility_m, siyi_meteorological_data->true_wind_speed_m_s, siyi_meteorological_data->true_wind_dir_deg, siyi_meteorological_data->baro_altitude_m, siyi_meteorological_data->dust_concentration_ug_m3, siyi_meteorological_data->pm1_ug_m3, siyi_meteorological_data->pm10_ug_m3, siyi_meteorological_data->uv_w_m2, siyi_meteorological_data->uv_index, siyi_meteorological_data->illuminance_lux, siyi_meteorological_data->daily_radiation_kj, siyi_meteorological_data->solar_radiation_w_m2);
}

/**
 * @brief Encode a siyi_meteorological_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param siyi_meteorological_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_siyi_meteorological_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_siyi_meteorological_data_t* siyi_meteorological_data)
{
    return mavlink_msg_siyi_meteorological_data_pack_chan(system_id, component_id, chan, msg, siyi_meteorological_data->time_boot_ms, siyi_meteorological_data->air_temperature_c, siyi_meteorological_data->relative_humidity_pct, siyi_meteorological_data->pressure_abs_hpa, siyi_meteorological_data->relative_wind_dir_deg, siyi_meteorological_data->relative_wind_speed_m_s, siyi_meteorological_data->compass_hdg_deg, siyi_meteorological_data->visibility_m, siyi_meteorological_data->true_wind_speed_m_s, siyi_meteorological_data->true_wind_dir_deg, siyi_meteorological_data->baro_altitude_m, siyi_meteorological_data->dust_concentration_ug_m3, siyi_meteorological_data->pm1_ug_m3, siyi_meteorological_data->pm10_ug_m3, siyi_meteorological_data->uv_w_m2, siyi_meteorological_data->uv_index, siyi_meteorological_data->illuminance_lux, siyi_meteorological_data->daily_radiation_kj, siyi_meteorological_data->solar_radiation_w_m2);
}

/**
 * @brief Encode a siyi_meteorological_data struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param siyi_meteorological_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_siyi_meteorological_data_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_siyi_meteorological_data_t* siyi_meteorological_data)
{
    return mavlink_msg_siyi_meteorological_data_pack_status(system_id, component_id, _status, msg,  siyi_meteorological_data->time_boot_ms, siyi_meteorological_data->air_temperature_c, siyi_meteorological_data->relative_humidity_pct, siyi_meteorological_data->pressure_abs_hpa, siyi_meteorological_data->relative_wind_dir_deg, siyi_meteorological_data->relative_wind_speed_m_s, siyi_meteorological_data->compass_hdg_deg, siyi_meteorological_data->visibility_m, siyi_meteorological_data->true_wind_speed_m_s, siyi_meteorological_data->true_wind_dir_deg, siyi_meteorological_data->baro_altitude_m, siyi_meteorological_data->dust_concentration_ug_m3, siyi_meteorological_data->pm1_ug_m3, siyi_meteorological_data->pm10_ug_m3, siyi_meteorological_data->uv_w_m2, siyi_meteorological_data->uv_index, siyi_meteorological_data->illuminance_lux, siyi_meteorological_data->daily_radiation_kj, siyi_meteorological_data->solar_radiation_w_m2);
}

/**
 * @brief Send a siyi_meteorological_data message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param air_temperature_c [degC] Air temperature; NaN if unknown.
 * @param relative_humidity_pct [%] Relative humidity 0-100; NaN if unknown.
 * @param pressure_abs_hpa [hPa] Absolute air pressure; NaN if unknown.
 * @param relative_wind_dir_deg [deg] Relative wind direction (0-359).
 * @param relative_wind_speed_m_s [m/s] Relative wind speed.
 * @param compass_hdg_deg [cdeg] Electronic compass heading 0-35999 (NOT COG); 65535 if unknown.
 * @param visibility_m [m] Meteorological visibility.
 * @param true_wind_speed_m_s [m/s] True wind speed; NaN if unknown.
 * @param true_wind_dir_deg [deg] True wind direction (0-359).
 * @param baro_altitude_m [m] Barometric / pressure altitude if not same as GPS MSL; NaN if unknown.
 * @param dust_concentration_ug_m3 [ug/m3] Dust concentration.
 * @param pm1_ug_m3 [ug/m3] PM1.0.
 * @param pm10_ug_m3 [ug/m3] PM10.
 * @param uv_w_m2 [W/m2] UV irradiance.
 * @param uv_index  UV index (0-255); 255=invalid.
 * @param illuminance_lux [lux] Illuminance.
 * @param daily_radiation_kj [kJ] Daily accumulated solar radiation energy.
 * @param solar_radiation_w_m2 [W/m2] Solar radiation power density.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_siyi_meteorological_data_send(mavlink_channel_t chan, uint32_t time_boot_ms, float air_temperature_c, float relative_humidity_pct, float pressure_abs_hpa, uint16_t relative_wind_dir_deg, float relative_wind_speed_m_s, uint16_t compass_hdg_deg, float visibility_m, float true_wind_speed_m_s, uint16_t true_wind_dir_deg, float baro_altitude_m, float dust_concentration_ug_m3, float pm1_ug_m3, float pm10_ug_m3, float uv_w_m2, uint8_t uv_index, float illuminance_lux, float daily_radiation_kj, float solar_radiation_w_m2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, air_temperature_c);
    _mav_put_float(buf, 8, relative_humidity_pct);
    _mav_put_float(buf, 12, pressure_abs_hpa);
    _mav_put_float(buf, 16, relative_wind_speed_m_s);
    _mav_put_float(buf, 20, visibility_m);
    _mav_put_float(buf, 24, true_wind_speed_m_s);
    _mav_put_float(buf, 28, baro_altitude_m);
    _mav_put_float(buf, 32, dust_concentration_ug_m3);
    _mav_put_float(buf, 36, pm1_ug_m3);
    _mav_put_float(buf, 40, pm10_ug_m3);
    _mav_put_float(buf, 44, uv_w_m2);
    _mav_put_float(buf, 48, illuminance_lux);
    _mav_put_float(buf, 52, daily_radiation_kj);
    _mav_put_float(buf, 56, solar_radiation_w_m2);
    _mav_put_uint16_t(buf, 60, relative_wind_dir_deg);
    _mav_put_uint16_t(buf, 62, compass_hdg_deg);
    _mav_put_uint16_t(buf, 64, true_wind_dir_deg);
    _mav_put_uint8_t(buf, 66, uv_index);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA, buf, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_MIN_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_CRC);
#else
    mavlink_siyi_meteorological_data_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.air_temperature_c = air_temperature_c;
    packet.relative_humidity_pct = relative_humidity_pct;
    packet.pressure_abs_hpa = pressure_abs_hpa;
    packet.relative_wind_speed_m_s = relative_wind_speed_m_s;
    packet.visibility_m = visibility_m;
    packet.true_wind_speed_m_s = true_wind_speed_m_s;
    packet.baro_altitude_m = baro_altitude_m;
    packet.dust_concentration_ug_m3 = dust_concentration_ug_m3;
    packet.pm1_ug_m3 = pm1_ug_m3;
    packet.pm10_ug_m3 = pm10_ug_m3;
    packet.uv_w_m2 = uv_w_m2;
    packet.illuminance_lux = illuminance_lux;
    packet.daily_radiation_kj = daily_radiation_kj;
    packet.solar_radiation_w_m2 = solar_radiation_w_m2;
    packet.relative_wind_dir_deg = relative_wind_dir_deg;
    packet.compass_hdg_deg = compass_hdg_deg;
    packet.true_wind_dir_deg = true_wind_dir_deg;
    packet.uv_index = uv_index;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA, (const char *)&packet, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_MIN_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_CRC);
#endif
}

/**
 * @brief Send a siyi_meteorological_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_siyi_meteorological_data_send_struct(mavlink_channel_t chan, const mavlink_siyi_meteorological_data_t* siyi_meteorological_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_siyi_meteorological_data_send(chan, siyi_meteorological_data->time_boot_ms, siyi_meteorological_data->air_temperature_c, siyi_meteorological_data->relative_humidity_pct, siyi_meteorological_data->pressure_abs_hpa, siyi_meteorological_data->relative_wind_dir_deg, siyi_meteorological_data->relative_wind_speed_m_s, siyi_meteorological_data->compass_hdg_deg, siyi_meteorological_data->visibility_m, siyi_meteorological_data->true_wind_speed_m_s, siyi_meteorological_data->true_wind_dir_deg, siyi_meteorological_data->baro_altitude_m, siyi_meteorological_data->dust_concentration_ug_m3, siyi_meteorological_data->pm1_ug_m3, siyi_meteorological_data->pm10_ug_m3, siyi_meteorological_data->uv_w_m2, siyi_meteorological_data->uv_index, siyi_meteorological_data->illuminance_lux, siyi_meteorological_data->daily_radiation_kj, siyi_meteorological_data->solar_radiation_w_m2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA, (const char *)siyi_meteorological_data, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_MIN_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_siyi_meteorological_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float air_temperature_c, float relative_humidity_pct, float pressure_abs_hpa, uint16_t relative_wind_dir_deg, float relative_wind_speed_m_s, uint16_t compass_hdg_deg, float visibility_m, float true_wind_speed_m_s, uint16_t true_wind_dir_deg, float baro_altitude_m, float dust_concentration_ug_m3, float pm1_ug_m3, float pm10_ug_m3, float uv_w_m2, uint8_t uv_index, float illuminance_lux, float daily_radiation_kj, float solar_radiation_w_m2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, air_temperature_c);
    _mav_put_float(buf, 8, relative_humidity_pct);
    _mav_put_float(buf, 12, pressure_abs_hpa);
    _mav_put_float(buf, 16, relative_wind_speed_m_s);
    _mav_put_float(buf, 20, visibility_m);
    _mav_put_float(buf, 24, true_wind_speed_m_s);
    _mav_put_float(buf, 28, baro_altitude_m);
    _mav_put_float(buf, 32, dust_concentration_ug_m3);
    _mav_put_float(buf, 36, pm1_ug_m3);
    _mav_put_float(buf, 40, pm10_ug_m3);
    _mav_put_float(buf, 44, uv_w_m2);
    _mav_put_float(buf, 48, illuminance_lux);
    _mav_put_float(buf, 52, daily_radiation_kj);
    _mav_put_float(buf, 56, solar_radiation_w_m2);
    _mav_put_uint16_t(buf, 60, relative_wind_dir_deg);
    _mav_put_uint16_t(buf, 62, compass_hdg_deg);
    _mav_put_uint16_t(buf, 64, true_wind_dir_deg);
    _mav_put_uint8_t(buf, 66, uv_index);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA, buf, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_MIN_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_CRC);
#else
    mavlink_siyi_meteorological_data_t *packet = (mavlink_siyi_meteorological_data_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->air_temperature_c = air_temperature_c;
    packet->relative_humidity_pct = relative_humidity_pct;
    packet->pressure_abs_hpa = pressure_abs_hpa;
    packet->relative_wind_speed_m_s = relative_wind_speed_m_s;
    packet->visibility_m = visibility_m;
    packet->true_wind_speed_m_s = true_wind_speed_m_s;
    packet->baro_altitude_m = baro_altitude_m;
    packet->dust_concentration_ug_m3 = dust_concentration_ug_m3;
    packet->pm1_ug_m3 = pm1_ug_m3;
    packet->pm10_ug_m3 = pm10_ug_m3;
    packet->uv_w_m2 = uv_w_m2;
    packet->illuminance_lux = illuminance_lux;
    packet->daily_radiation_kj = daily_radiation_kj;
    packet->solar_radiation_w_m2 = solar_radiation_w_m2;
    packet->relative_wind_dir_deg = relative_wind_dir_deg;
    packet->compass_hdg_deg = compass_hdg_deg;
    packet->true_wind_dir_deg = true_wind_dir_deg;
    packet->uv_index = uv_index;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA, (const char *)packet, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_MIN_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE SIYI_METEOROLOGICAL_DATA UNPACKING


/**
 * @brief Get field time_boot_ms from siyi_meteorological_data message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_siyi_meteorological_data_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field air_temperature_c from siyi_meteorological_data message
 *
 * @return [degC] Air temperature; NaN if unknown.
 */
static inline float mavlink_msg_siyi_meteorological_data_get_air_temperature_c(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field relative_humidity_pct from siyi_meteorological_data message
 *
 * @return [%] Relative humidity 0-100; NaN if unknown.
 */
static inline float mavlink_msg_siyi_meteorological_data_get_relative_humidity_pct(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pressure_abs_hpa from siyi_meteorological_data message
 *
 * @return [hPa] Absolute air pressure; NaN if unknown.
 */
static inline float mavlink_msg_siyi_meteorological_data_get_pressure_abs_hpa(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field relative_wind_dir_deg from siyi_meteorological_data message
 *
 * @return [deg] Relative wind direction (0-359).
 */
static inline uint16_t mavlink_msg_siyi_meteorological_data_get_relative_wind_dir_deg(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  60);
}

/**
 * @brief Get field relative_wind_speed_m_s from siyi_meteorological_data message
 *
 * @return [m/s] Relative wind speed.
 */
static inline float mavlink_msg_siyi_meteorological_data_get_relative_wind_speed_m_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field compass_hdg_deg from siyi_meteorological_data message
 *
 * @return [cdeg] Electronic compass heading 0-35999 (NOT COG); 65535 if unknown.
 */
static inline uint16_t mavlink_msg_siyi_meteorological_data_get_compass_hdg_deg(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  62);
}

/**
 * @brief Get field visibility_m from siyi_meteorological_data message
 *
 * @return [m] Meteorological visibility.
 */
static inline float mavlink_msg_siyi_meteorological_data_get_visibility_m(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field true_wind_speed_m_s from siyi_meteorological_data message
 *
 * @return [m/s] True wind speed; NaN if unknown.
 */
static inline float mavlink_msg_siyi_meteorological_data_get_true_wind_speed_m_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field true_wind_dir_deg from siyi_meteorological_data message
 *
 * @return [deg] True wind direction (0-359).
 */
static inline uint16_t mavlink_msg_siyi_meteorological_data_get_true_wind_dir_deg(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  64);
}

/**
 * @brief Get field baro_altitude_m from siyi_meteorological_data message
 *
 * @return [m] Barometric / pressure altitude if not same as GPS MSL; NaN if unknown.
 */
static inline float mavlink_msg_siyi_meteorological_data_get_baro_altitude_m(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field dust_concentration_ug_m3 from siyi_meteorological_data message
 *
 * @return [ug/m3] Dust concentration.
 */
static inline float mavlink_msg_siyi_meteorological_data_get_dust_concentration_ug_m3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field pm1_ug_m3 from siyi_meteorological_data message
 *
 * @return [ug/m3] PM1.0.
 */
static inline float mavlink_msg_siyi_meteorological_data_get_pm1_ug_m3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field pm10_ug_m3 from siyi_meteorological_data message
 *
 * @return [ug/m3] PM10.
 */
static inline float mavlink_msg_siyi_meteorological_data_get_pm10_ug_m3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field uv_w_m2 from siyi_meteorological_data message
 *
 * @return [W/m2] UV irradiance.
 */
static inline float mavlink_msg_siyi_meteorological_data_get_uv_w_m2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field uv_index from siyi_meteorological_data message
 *
 * @return  UV index (0-255); 255=invalid.
 */
static inline uint8_t mavlink_msg_siyi_meteorological_data_get_uv_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  66);
}

/**
 * @brief Get field illuminance_lux from siyi_meteorological_data message
 *
 * @return [lux] Illuminance.
 */
static inline float mavlink_msg_siyi_meteorological_data_get_illuminance_lux(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field daily_radiation_kj from siyi_meteorological_data message
 *
 * @return [kJ] Daily accumulated solar radiation energy.
 */
static inline float mavlink_msg_siyi_meteorological_data_get_daily_radiation_kj(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field solar_radiation_w_m2 from siyi_meteorological_data message
 *
 * @return [W/m2] Solar radiation power density.
 */
static inline float mavlink_msg_siyi_meteorological_data_get_solar_radiation_w_m2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Decode a siyi_meteorological_data message into a struct
 *
 * @param msg The message to decode
 * @param siyi_meteorological_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_siyi_meteorological_data_decode(const mavlink_message_t* msg, mavlink_siyi_meteorological_data_t* siyi_meteorological_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    siyi_meteorological_data->time_boot_ms = mavlink_msg_siyi_meteorological_data_get_time_boot_ms(msg);
    siyi_meteorological_data->air_temperature_c = mavlink_msg_siyi_meteorological_data_get_air_temperature_c(msg);
    siyi_meteorological_data->relative_humidity_pct = mavlink_msg_siyi_meteorological_data_get_relative_humidity_pct(msg);
    siyi_meteorological_data->pressure_abs_hpa = mavlink_msg_siyi_meteorological_data_get_pressure_abs_hpa(msg);
    siyi_meteorological_data->relative_wind_speed_m_s = mavlink_msg_siyi_meteorological_data_get_relative_wind_speed_m_s(msg);
    siyi_meteorological_data->visibility_m = mavlink_msg_siyi_meteorological_data_get_visibility_m(msg);
    siyi_meteorological_data->true_wind_speed_m_s = mavlink_msg_siyi_meteorological_data_get_true_wind_speed_m_s(msg);
    siyi_meteorological_data->baro_altitude_m = mavlink_msg_siyi_meteorological_data_get_baro_altitude_m(msg);
    siyi_meteorological_data->dust_concentration_ug_m3 = mavlink_msg_siyi_meteorological_data_get_dust_concentration_ug_m3(msg);
    siyi_meteorological_data->pm1_ug_m3 = mavlink_msg_siyi_meteorological_data_get_pm1_ug_m3(msg);
    siyi_meteorological_data->pm10_ug_m3 = mavlink_msg_siyi_meteorological_data_get_pm10_ug_m3(msg);
    siyi_meteorological_data->uv_w_m2 = mavlink_msg_siyi_meteorological_data_get_uv_w_m2(msg);
    siyi_meteorological_data->illuminance_lux = mavlink_msg_siyi_meteorological_data_get_illuminance_lux(msg);
    siyi_meteorological_data->daily_radiation_kj = mavlink_msg_siyi_meteorological_data_get_daily_radiation_kj(msg);
    siyi_meteorological_data->solar_radiation_w_m2 = mavlink_msg_siyi_meteorological_data_get_solar_radiation_w_m2(msg);
    siyi_meteorological_data->relative_wind_dir_deg = mavlink_msg_siyi_meteorological_data_get_relative_wind_dir_deg(msg);
    siyi_meteorological_data->compass_hdg_deg = mavlink_msg_siyi_meteorological_data_get_compass_hdg_deg(msg);
    siyi_meteorological_data->true_wind_dir_deg = mavlink_msg_siyi_meteorological_data_get_true_wind_dir_deg(msg);
    siyi_meteorological_data->uv_index = mavlink_msg_siyi_meteorological_data_get_uv_index(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN? msg->len : MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN;
        memset(siyi_meteorological_data, 0, MAVLINK_MSG_ID_SIYI_METEOROLOGICAL_DATA_LEN);
    memcpy(siyi_meteorological_data, _MAV_PAYLOAD(msg), len);
#endif
}
