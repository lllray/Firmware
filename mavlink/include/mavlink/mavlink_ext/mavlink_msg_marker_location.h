#pragma once
// MESSAGE MARKER_LOCATION PACKING

#define MAVLINK_MSG_ID_MARKER_LOCATION 401

MAVPACKED(
typedef struct __mavlink_marker_location_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.*/
 float x; /*< [m] X Position of the marker location on NED_FRAME*/
 float y; /*< [m] Y Position of the marker location on NED_FRAME*/
 float z; /*< [m] Z Position of the marker location on NED_FRAME*/
 float roll; /*< [rad] roll of the marker location on NED_FRAME*/
 float pitch; /*< [rad] pitch of the marker location on NED_FRAME*/
 float yaw; /*< [rad] yaw of the marker location on NED_FRAME*/
}) mavlink_marker_location_t;

#define MAVLINK_MSG_ID_MARKER_LOCATION_LEN 32
#define MAVLINK_MSG_ID_MARKER_LOCATION_MIN_LEN 32
#define MAVLINK_MSG_ID_401_LEN 32
#define MAVLINK_MSG_ID_401_MIN_LEN 32

#define MAVLINK_MSG_ID_MARKER_LOCATION_CRC 30
#define MAVLINK_MSG_ID_401_CRC 30



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MARKER_LOCATION { \
    401, \
    "MARKER_LOCATION", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_marker_location_t, time_usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_marker_location_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_marker_location_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_marker_location_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_marker_location_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_marker_location_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_marker_location_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MARKER_LOCATION { \
    "MARKER_LOCATION", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_marker_location_t, time_usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_marker_location_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_marker_location_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_marker_location_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_marker_location_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_marker_location_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_marker_location_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a marker_location message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param x [m] X Position of the marker location on NED_FRAME
 * @param y [m] Y Position of the marker location on NED_FRAME
 * @param z [m] Z Position of the marker location on NED_FRAME
 * @param roll [rad] roll of the marker location on NED_FRAME
 * @param pitch [rad] pitch of the marker location on NED_FRAME
 * @param yaw [rad] yaw of the marker location on NED_FRAME
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_marker_location_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float x, float y, float z, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MARKER_LOCATION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MARKER_LOCATION_LEN);
#else
    mavlink_marker_location_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MARKER_LOCATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MARKER_LOCATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MARKER_LOCATION_MIN_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_CRC);
}

/**
 * @brief Pack a marker_location message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param x [m] X Position of the marker location on NED_FRAME
 * @param y [m] Y Position of the marker location on NED_FRAME
 * @param z [m] Z Position of the marker location on NED_FRAME
 * @param roll [rad] roll of the marker location on NED_FRAME
 * @param pitch [rad] pitch of the marker location on NED_FRAME
 * @param yaw [rad] yaw of the marker location on NED_FRAME
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_marker_location_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float x,float y,float z,float roll,float pitch,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MARKER_LOCATION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MARKER_LOCATION_LEN);
#else
    mavlink_marker_location_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MARKER_LOCATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MARKER_LOCATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MARKER_LOCATION_MIN_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_CRC);
}

/**
 * @brief Encode a marker_location struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param marker_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_marker_location_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_marker_location_t* marker_location)
{
    return mavlink_msg_marker_location_pack(system_id, component_id, msg, marker_location->time_usec, marker_location->x, marker_location->y, marker_location->z, marker_location->roll, marker_location->pitch, marker_location->yaw);
}

/**
 * @brief Encode a marker_location struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param marker_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_marker_location_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_marker_location_t* marker_location)
{
    return mavlink_msg_marker_location_pack_chan(system_id, component_id, chan, msg, marker_location->time_usec, marker_location->x, marker_location->y, marker_location->z, marker_location->roll, marker_location->pitch, marker_location->yaw);
}

/**
 * @brief Send a marker_location message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param x [m] X Position of the marker location on NED_FRAME
 * @param y [m] Y Position of the marker location on NED_FRAME
 * @param z [m] Z Position of the marker location on NED_FRAME
 * @param roll [rad] roll of the marker location on NED_FRAME
 * @param pitch [rad] pitch of the marker location on NED_FRAME
 * @param yaw [rad] yaw of the marker location on NED_FRAME
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_marker_location_send(mavlink_channel_t chan, uint64_t time_usec, float x, float y, float z, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MARKER_LOCATION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION, buf, MAVLINK_MSG_ID_MARKER_LOCATION_MIN_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_CRC);
#else
    mavlink_marker_location_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION, (const char *)&packet, MAVLINK_MSG_ID_MARKER_LOCATION_MIN_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_CRC);
#endif
}

/**
 * @brief Send a marker_location message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_marker_location_send_struct(mavlink_channel_t chan, const mavlink_marker_location_t* marker_location)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_marker_location_send(chan, marker_location->time_usec, marker_location->x, marker_location->y, marker_location->z, marker_location->roll, marker_location->pitch, marker_location->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION, (const char *)marker_location, MAVLINK_MSG_ID_MARKER_LOCATION_MIN_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_MARKER_LOCATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_marker_location_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float x, float y, float z, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION, buf, MAVLINK_MSG_ID_MARKER_LOCATION_MIN_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_CRC);
#else
    mavlink_marker_location_t *packet = (mavlink_marker_location_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER_LOCATION, (const char *)packet, MAVLINK_MSG_ID_MARKER_LOCATION_MIN_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_LEN, MAVLINK_MSG_ID_MARKER_LOCATION_CRC);
#endif
}
#endif

#endif

// MESSAGE MARKER_LOCATION UNPACKING


/**
 * @brief Get field time_usec from marker_location message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 */
static inline uint64_t mavlink_msg_marker_location_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field x from marker_location message
 *
 * @return [m] X Position of the marker location on NED_FRAME
 */
static inline float mavlink_msg_marker_location_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field y from marker_location message
 *
 * @return [m] Y Position of the marker location on NED_FRAME
 */
static inline float mavlink_msg_marker_location_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field z from marker_location message
 *
 * @return [m] Z Position of the marker location on NED_FRAME
 */
static inline float mavlink_msg_marker_location_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field roll from marker_location message
 *
 * @return [rad] roll of the marker location on NED_FRAME
 */
static inline float mavlink_msg_marker_location_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitch from marker_location message
 *
 * @return [rad] pitch of the marker location on NED_FRAME
 */
static inline float mavlink_msg_marker_location_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yaw from marker_location message
 *
 * @return [rad] yaw of the marker location on NED_FRAME
 */
static inline float mavlink_msg_marker_location_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a marker_location message into a struct
 *
 * @param msg The message to decode
 * @param marker_location C-struct to decode the message contents into
 */
static inline void mavlink_msg_marker_location_decode(const mavlink_message_t* msg, mavlink_marker_location_t* marker_location)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    marker_location->time_usec = mavlink_msg_marker_location_get_time_usec(msg);
    marker_location->x = mavlink_msg_marker_location_get_x(msg);
    marker_location->y = mavlink_msg_marker_location_get_y(msg);
    marker_location->z = mavlink_msg_marker_location_get_z(msg);
    marker_location->roll = mavlink_msg_marker_location_get_roll(msg);
    marker_location->pitch = mavlink_msg_marker_location_get_pitch(msg);
    marker_location->yaw = mavlink_msg_marker_location_get_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MARKER_LOCATION_LEN? msg->len : MAVLINK_MSG_ID_MARKER_LOCATION_LEN;
        memset(marker_location, 0, MAVLINK_MSG_ID_MARKER_LOCATION_LEN);
    memcpy(marker_location, _MAV_PAYLOAD(msg), len);
#endif
}
