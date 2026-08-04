#pragma once
// MESSAGE SIYI_SMART_BATTERY_USE_INFO PACKING

#define MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO 14601


typedef struct __mavlink_siyi_smart_battery_use_info_t {
 uint32_t remaining_time; /*< [s] Estimated remaining flight time in seconds.*/
 uint16_t recommend_return_pct; /*< [%] Recommended battery percentage to start return-home.*/
 uint16_t force_return_pct; /*< [%] Force-return-home battery percentage (0 disables).*/
} mavlink_siyi_smart_battery_use_info_t;

#define MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN 8
#define MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_MIN_LEN 8
#define MAVLINK_MSG_ID_14601_LEN 8
#define MAVLINK_MSG_ID_14601_MIN_LEN 8

#define MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_CRC 33
#define MAVLINK_MSG_ID_14601_CRC 33



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SIYI_SMART_BATTERY_USE_INFO { \
    14601, \
    "SIYI_SMART_BATTERY_USE_INFO", \
    3, \
    {  { "recommend_return_pct", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_siyi_smart_battery_use_info_t, recommend_return_pct) }, \
         { "force_return_pct", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_siyi_smart_battery_use_info_t, force_return_pct) }, \
         { "remaining_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_siyi_smart_battery_use_info_t, remaining_time) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SIYI_SMART_BATTERY_USE_INFO { \
    "SIYI_SMART_BATTERY_USE_INFO", \
    3, \
    {  { "recommend_return_pct", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_siyi_smart_battery_use_info_t, recommend_return_pct) }, \
         { "force_return_pct", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_siyi_smart_battery_use_info_t, force_return_pct) }, \
         { "remaining_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_siyi_smart_battery_use_info_t, remaining_time) }, \
         } \
}
#endif

/**
 * @brief Pack a siyi_smart_battery_use_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param recommend_return_pct [%] Recommended battery percentage to start return-home.
 * @param force_return_pct [%] Force-return-home battery percentage (0 disables).
 * @param remaining_time [s] Estimated remaining flight time in seconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_siyi_smart_battery_use_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t recommend_return_pct, uint16_t force_return_pct, uint32_t remaining_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN];
    _mav_put_uint32_t(buf, 0, remaining_time);
    _mav_put_uint16_t(buf, 4, recommend_return_pct);
    _mav_put_uint16_t(buf, 6, force_return_pct);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN);
#else
    mavlink_siyi_smart_battery_use_info_t packet;
    packet.remaining_time = remaining_time;
    packet.recommend_return_pct = recommend_return_pct;
    packet.force_return_pct = force_return_pct;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_MIN_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_CRC);
}

/**
 * @brief Pack a siyi_smart_battery_use_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param recommend_return_pct [%] Recommended battery percentage to start return-home.
 * @param force_return_pct [%] Force-return-home battery percentage (0 disables).
 * @param remaining_time [s] Estimated remaining flight time in seconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_siyi_smart_battery_use_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t recommend_return_pct, uint16_t force_return_pct, uint32_t remaining_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN];
    _mav_put_uint32_t(buf, 0, remaining_time);
    _mav_put_uint16_t(buf, 4, recommend_return_pct);
    _mav_put_uint16_t(buf, 6, force_return_pct);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN);
#else
    mavlink_siyi_smart_battery_use_info_t packet;
    packet.remaining_time = remaining_time;
    packet.recommend_return_pct = recommend_return_pct;
    packet.force_return_pct = force_return_pct;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_MIN_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_MIN_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN);
#endif
}

/**
 * @brief Pack a siyi_smart_battery_use_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param recommend_return_pct [%] Recommended battery percentage to start return-home.
 * @param force_return_pct [%] Force-return-home battery percentage (0 disables).
 * @param remaining_time [s] Estimated remaining flight time in seconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_siyi_smart_battery_use_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t recommend_return_pct,uint16_t force_return_pct,uint32_t remaining_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN];
    _mav_put_uint32_t(buf, 0, remaining_time);
    _mav_put_uint16_t(buf, 4, recommend_return_pct);
    _mav_put_uint16_t(buf, 6, force_return_pct);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN);
#else
    mavlink_siyi_smart_battery_use_info_t packet;
    packet.remaining_time = remaining_time;
    packet.recommend_return_pct = recommend_return_pct;
    packet.force_return_pct = force_return_pct;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_MIN_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_CRC);
}

/**
 * @brief Encode a siyi_smart_battery_use_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param siyi_smart_battery_use_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_siyi_smart_battery_use_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_siyi_smart_battery_use_info_t* siyi_smart_battery_use_info)
{
    return mavlink_msg_siyi_smart_battery_use_info_pack(system_id, component_id, msg, siyi_smart_battery_use_info->recommend_return_pct, siyi_smart_battery_use_info->force_return_pct, siyi_smart_battery_use_info->remaining_time);
}

/**
 * @brief Encode a siyi_smart_battery_use_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param siyi_smart_battery_use_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_siyi_smart_battery_use_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_siyi_smart_battery_use_info_t* siyi_smart_battery_use_info)
{
    return mavlink_msg_siyi_smart_battery_use_info_pack_chan(system_id, component_id, chan, msg, siyi_smart_battery_use_info->recommend_return_pct, siyi_smart_battery_use_info->force_return_pct, siyi_smart_battery_use_info->remaining_time);
}

/**
 * @brief Encode a siyi_smart_battery_use_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param siyi_smart_battery_use_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_siyi_smart_battery_use_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_siyi_smart_battery_use_info_t* siyi_smart_battery_use_info)
{
    return mavlink_msg_siyi_smart_battery_use_info_pack_status(system_id, component_id, _status, msg,  siyi_smart_battery_use_info->recommend_return_pct, siyi_smart_battery_use_info->force_return_pct, siyi_smart_battery_use_info->remaining_time);
}

/**
 * @brief Send a siyi_smart_battery_use_info message
 * @param chan MAVLink channel to send the message
 *
 * @param recommend_return_pct [%] Recommended battery percentage to start return-home.
 * @param force_return_pct [%] Force-return-home battery percentage (0 disables).
 * @param remaining_time [s] Estimated remaining flight time in seconds.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_siyi_smart_battery_use_info_send(mavlink_channel_t chan, uint16_t recommend_return_pct, uint16_t force_return_pct, uint32_t remaining_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN];
    _mav_put_uint32_t(buf, 0, remaining_time);
    _mav_put_uint16_t(buf, 4, recommend_return_pct);
    _mav_put_uint16_t(buf, 6, force_return_pct);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO, buf, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_MIN_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_CRC);
#else
    mavlink_siyi_smart_battery_use_info_t packet;
    packet.remaining_time = remaining_time;
    packet.recommend_return_pct = recommend_return_pct;
    packet.force_return_pct = force_return_pct;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO, (const char *)&packet, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_MIN_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_CRC);
#endif
}

/**
 * @brief Send a siyi_smart_battery_use_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_siyi_smart_battery_use_info_send_struct(mavlink_channel_t chan, const mavlink_siyi_smart_battery_use_info_t* siyi_smart_battery_use_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_siyi_smart_battery_use_info_send(chan, siyi_smart_battery_use_info->recommend_return_pct, siyi_smart_battery_use_info->force_return_pct, siyi_smart_battery_use_info->remaining_time);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO, (const char *)siyi_smart_battery_use_info, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_MIN_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_siyi_smart_battery_use_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t recommend_return_pct, uint16_t force_return_pct, uint32_t remaining_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, remaining_time);
    _mav_put_uint16_t(buf, 4, recommend_return_pct);
    _mav_put_uint16_t(buf, 6, force_return_pct);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO, buf, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_MIN_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_CRC);
#else
    mavlink_siyi_smart_battery_use_info_t *packet = (mavlink_siyi_smart_battery_use_info_t *)msgbuf;
    packet->remaining_time = remaining_time;
    packet->recommend_return_pct = recommend_return_pct;
    packet->force_return_pct = force_return_pct;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO, (const char *)packet, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_MIN_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE SIYI_SMART_BATTERY_USE_INFO UNPACKING


/**
 * @brief Get field recommend_return_pct from siyi_smart_battery_use_info message
 *
 * @return [%] Recommended battery percentage to start return-home.
 */
static inline uint16_t mavlink_msg_siyi_smart_battery_use_info_get_recommend_return_pct(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field force_return_pct from siyi_smart_battery_use_info message
 *
 * @return [%] Force-return-home battery percentage (0 disables).
 */
static inline uint16_t mavlink_msg_siyi_smart_battery_use_info_get_force_return_pct(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field remaining_time from siyi_smart_battery_use_info message
 *
 * @return [s] Estimated remaining flight time in seconds.
 */
static inline uint32_t mavlink_msg_siyi_smart_battery_use_info_get_remaining_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a siyi_smart_battery_use_info message into a struct
 *
 * @param msg The message to decode
 * @param siyi_smart_battery_use_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_siyi_smart_battery_use_info_decode(const mavlink_message_t* msg, mavlink_siyi_smart_battery_use_info_t* siyi_smart_battery_use_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    siyi_smart_battery_use_info->remaining_time = mavlink_msg_siyi_smart_battery_use_info_get_remaining_time(msg);
    siyi_smart_battery_use_info->recommend_return_pct = mavlink_msg_siyi_smart_battery_use_info_get_recommend_return_pct(msg);
    siyi_smart_battery_use_info->force_return_pct = mavlink_msg_siyi_smart_battery_use_info_get_force_return_pct(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN? msg->len : MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN;
        memset(siyi_smart_battery_use_info, 0, MAVLINK_MSG_ID_SIYI_SMART_BATTERY_USE_INFO_LEN);
    memcpy(siyi_smart_battery_use_info, _MAV_PAYLOAD(msg), len);
#endif
}
