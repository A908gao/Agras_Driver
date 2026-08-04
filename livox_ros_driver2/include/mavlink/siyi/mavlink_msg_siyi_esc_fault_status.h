#pragma once
// MESSAGE SIYI_ESC_FAULT_STATUS PACKING

#define MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS 14611


typedef struct __mavlink_siyi_esc_fault_status_t {
 uint32_t timestamp; /*< [ms] Timestamp in milliseconds.*/
 uint16_t fault_flags; /*<  Fault flags bitmask. Bit0=低压, Bit1=过压, Bit2=运放异常, Bit3=MOS短路, Bit4=缺相, Bit5=油门丢失, Bit6=油门不归零, Bit7=堵转, Bit8=电调过温, Bit9=电容过温, Bit11=过流.*/
 uint8_t esc_id; /*<  ESC ID (0-based index).*/
} mavlink_siyi_esc_fault_status_t;

#define MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN 7
#define MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_MIN_LEN 7
#define MAVLINK_MSG_ID_14611_LEN 7
#define MAVLINK_MSG_ID_14611_MIN_LEN 7

#define MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_CRC 241
#define MAVLINK_MSG_ID_14611_CRC 241



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SIYI_ESC_FAULT_STATUS { \
    14611, \
    "SIYI_ESC_FAULT_STATUS", \
    3, \
    {  { "esc_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_siyi_esc_fault_status_t, esc_id) }, \
         { "fault_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_siyi_esc_fault_status_t, fault_flags) }, \
         { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_siyi_esc_fault_status_t, timestamp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SIYI_ESC_FAULT_STATUS { \
    "SIYI_ESC_FAULT_STATUS", \
    3, \
    {  { "esc_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_siyi_esc_fault_status_t, esc_id) }, \
         { "fault_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_siyi_esc_fault_status_t, fault_flags) }, \
         { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_siyi_esc_fault_status_t, timestamp) }, \
         } \
}
#endif

/**
 * @brief Pack a siyi_esc_fault_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param esc_id  ESC ID (0-based index).
 * @param fault_flags  Fault flags bitmask. Bit0=低压, Bit1=过压, Bit2=运放异常, Bit3=MOS短路, Bit4=缺相, Bit5=油门丢失, Bit6=油门不归零, Bit7=堵转, Bit8=电调过温, Bit9=电容过温, Bit11=过流.
 * @param timestamp [ms] Timestamp in milliseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_siyi_esc_fault_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t esc_id, uint16_t fault_flags, uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_uint16_t(buf, 4, fault_flags);
    _mav_put_uint8_t(buf, 6, esc_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN);
#else
    mavlink_siyi_esc_fault_status_t packet;
    packet.timestamp = timestamp;
    packet.fault_flags = fault_flags;
    packet.esc_id = esc_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_CRC);
}

/**
 * @brief Pack a siyi_esc_fault_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param esc_id  ESC ID (0-based index).
 * @param fault_flags  Fault flags bitmask. Bit0=低压, Bit1=过压, Bit2=运放异常, Bit3=MOS短路, Bit4=缺相, Bit5=油门丢失, Bit6=油门不归零, Bit7=堵转, Bit8=电调过温, Bit9=电容过温, Bit11=过流.
 * @param timestamp [ms] Timestamp in milliseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_siyi_esc_fault_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t esc_id, uint16_t fault_flags, uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_uint16_t(buf, 4, fault_flags);
    _mav_put_uint8_t(buf, 6, esc_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN);
#else
    mavlink_siyi_esc_fault_status_t packet;
    packet.timestamp = timestamp;
    packet.fault_flags = fault_flags;
    packet.esc_id = esc_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN);
#endif
}

/**
 * @brief Pack a siyi_esc_fault_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param esc_id  ESC ID (0-based index).
 * @param fault_flags  Fault flags bitmask. Bit0=低压, Bit1=过压, Bit2=运放异常, Bit3=MOS短路, Bit4=缺相, Bit5=油门丢失, Bit6=油门不归零, Bit7=堵转, Bit8=电调过温, Bit9=电容过温, Bit11=过流.
 * @param timestamp [ms] Timestamp in milliseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_siyi_esc_fault_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t esc_id,uint16_t fault_flags,uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_uint16_t(buf, 4, fault_flags);
    _mav_put_uint8_t(buf, 6, esc_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN);
#else
    mavlink_siyi_esc_fault_status_t packet;
    packet.timestamp = timestamp;
    packet.fault_flags = fault_flags;
    packet.esc_id = esc_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_CRC);
}

/**
 * @brief Encode a siyi_esc_fault_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param siyi_esc_fault_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_siyi_esc_fault_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_siyi_esc_fault_status_t* siyi_esc_fault_status)
{
    return mavlink_msg_siyi_esc_fault_status_pack(system_id, component_id, msg, siyi_esc_fault_status->esc_id, siyi_esc_fault_status->fault_flags, siyi_esc_fault_status->timestamp);
}

/**
 * @brief Encode a siyi_esc_fault_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param siyi_esc_fault_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_siyi_esc_fault_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_siyi_esc_fault_status_t* siyi_esc_fault_status)
{
    return mavlink_msg_siyi_esc_fault_status_pack_chan(system_id, component_id, chan, msg, siyi_esc_fault_status->esc_id, siyi_esc_fault_status->fault_flags, siyi_esc_fault_status->timestamp);
}

/**
 * @brief Encode a siyi_esc_fault_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param siyi_esc_fault_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_siyi_esc_fault_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_siyi_esc_fault_status_t* siyi_esc_fault_status)
{
    return mavlink_msg_siyi_esc_fault_status_pack_status(system_id, component_id, _status, msg,  siyi_esc_fault_status->esc_id, siyi_esc_fault_status->fault_flags, siyi_esc_fault_status->timestamp);
}

/**
 * @brief Send a siyi_esc_fault_status message
 * @param chan MAVLink channel to send the message
 *
 * @param esc_id  ESC ID (0-based index).
 * @param fault_flags  Fault flags bitmask. Bit0=低压, Bit1=过压, Bit2=运放异常, Bit3=MOS短路, Bit4=缺相, Bit5=油门丢失, Bit6=油门不归零, Bit7=堵转, Bit8=电调过温, Bit9=电容过温, Bit11=过流.
 * @param timestamp [ms] Timestamp in milliseconds.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_siyi_esc_fault_status_send(mavlink_channel_t chan, uint8_t esc_id, uint16_t fault_flags, uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_uint16_t(buf, 4, fault_flags);
    _mav_put_uint8_t(buf, 6, esc_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS, buf, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_CRC);
#else
    mavlink_siyi_esc_fault_status_t packet;
    packet.timestamp = timestamp;
    packet.fault_flags = fault_flags;
    packet.esc_id = esc_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_CRC);
#endif
}

/**
 * @brief Send a siyi_esc_fault_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_siyi_esc_fault_status_send_struct(mavlink_channel_t chan, const mavlink_siyi_esc_fault_status_t* siyi_esc_fault_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_siyi_esc_fault_status_send(chan, siyi_esc_fault_status->esc_id, siyi_esc_fault_status->fault_flags, siyi_esc_fault_status->timestamp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS, (const char *)siyi_esc_fault_status, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_siyi_esc_fault_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t esc_id, uint16_t fault_flags, uint32_t timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_uint16_t(buf, 4, fault_flags);
    _mav_put_uint8_t(buf, 6, esc_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS, buf, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_CRC);
#else
    mavlink_siyi_esc_fault_status_t *packet = (mavlink_siyi_esc_fault_status_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->fault_flags = fault_flags;
    packet->esc_id = esc_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS, (const char *)packet, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE SIYI_ESC_FAULT_STATUS UNPACKING


/**
 * @brief Get field esc_id from siyi_esc_fault_status message
 *
 * @return  ESC ID (0-based index).
 */
static inline uint8_t mavlink_msg_siyi_esc_fault_status_get_esc_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field fault_flags from siyi_esc_fault_status message
 *
 * @return  Fault flags bitmask. Bit0=低压, Bit1=过压, Bit2=运放异常, Bit3=MOS短路, Bit4=缺相, Bit5=油门丢失, Bit6=油门不归零, Bit7=堵转, Bit8=电调过温, Bit9=电容过温, Bit11=过流.
 */
static inline uint16_t mavlink_msg_siyi_esc_fault_status_get_fault_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field timestamp from siyi_esc_fault_status message
 *
 * @return [ms] Timestamp in milliseconds.
 */
static inline uint32_t mavlink_msg_siyi_esc_fault_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a siyi_esc_fault_status message into a struct
 *
 * @param msg The message to decode
 * @param siyi_esc_fault_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_siyi_esc_fault_status_decode(const mavlink_message_t* msg, mavlink_siyi_esc_fault_status_t* siyi_esc_fault_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    siyi_esc_fault_status->timestamp = mavlink_msg_siyi_esc_fault_status_get_timestamp(msg);
    siyi_esc_fault_status->fault_flags = mavlink_msg_siyi_esc_fault_status_get_fault_flags(msg);
    siyi_esc_fault_status->esc_id = mavlink_msg_siyi_esc_fault_status_get_esc_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN? msg->len : MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN;
        memset(siyi_esc_fault_status, 0, MAVLINK_MSG_ID_SIYI_ESC_FAULT_STATUS_LEN);
    memcpy(siyi_esc_fault_status, _MAV_PAYLOAD(msg), len);
#endif
}
