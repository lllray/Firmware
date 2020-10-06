#pragma once
// MESSAGE VISUAL_LOCATION PACKING

#define MAVLINK_MSG_ID_VISUAL_LOCATION 400

MAVPACKED(
typedef struct __mavlink_visual_location_t {
 float x; /*< [m] X Position of the landing target on NED_FRAME*/
 float y; /*< [m] Y Position of the landing target on NED_FRAME*/
 float z; /*< [m] Z Position of the landing target on NED_FRAME*/
 float roll; /*< [rad] roll of the landing target on NED_FRAME*/
 float pitch; /*< [rad] pitch of the landing target on NED_FRAME*/
 float yaw; /*< [rad] yaw of the landing target on NED_FRAME*/
}) mavlink_visual_location_t;

#define MAVLINK_MSG_ID_VISUAL_LOCATION_LEN 24
#define MAVLINK_MSG_ID_VISUAL_LOCATION_MIN_LEN 24
#define MAVLINK_MSG_ID_400_LEN 24
#define MAVLINK_MSG_ID_400_MIN_LEN 24

#define MAVLINK_MSG_ID_VISUAL_LOCATION_CRC 140
#define MAVLINK_MSG_ID_400_CRC 140



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VISUAL_LOCATION { \
    400, \
    "VISUAL_LOCATION", \
    6, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_visual_location_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_visual_location_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_visual_location_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_visual_location_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_visual_location_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_visual_location_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VISUAL_LOCATION { \
    "VISUAL_LOCATION", \
    6, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_visual_location_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_visual_location_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_visual_location_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_visual_location_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_visual_location_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_visual_location_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a visual_location message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x [m] X Position of the landing target on NED_FRAME
 * @param y [m] Y Position of the landing target on NED_FRAME
 * @param z [m] Z Position of the landing target on NED_FRAME
 * @param roll [rad] roll of the landing target on NED_FRAME
 * @param pitch [rad] pitch of the landing target on NED_FRAME
 * @param yaw [rad] yaw of the landing target on NED_FRAME
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_visual_location_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float z, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISUAL_LOCATION_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISUAL_LOCATION_LEN);
#else
    mavlink_visual_location_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISUAL_LOCATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISUAL_LOCATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISUAL_LOCATION_MIN_LEN, MAVLINK_MSG_ID_VISUAL_LOCATION_LEN, MAVLINK_MSG_ID_VISUAL_LOCATION_CRC);
}

/**
 * @brief Pack a visual_location message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x [m] X Position of the landing target on NED_FRAME
 * @param y [m] Y Position of the landing target on NED_FRAME
 * @param z [m] Z Position of the landing target on NED_FRAME
 * @param roll [rad] roll of the landing target on NED_FRAME
 * @param pitch [rad] pitch of the landing target on NED_FRAME
 * @param yaw [rad] yaw of the landing target on NED_FRAME
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_visual_location_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float z,float roll,float pitch,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISUAL_LOCATION_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISUAL_LOCATION_LEN);
#else
    mavlink_visual_location_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISUAL_LOCATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISUAL_LOCATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISUAL_LOCATION_MIN_LEN, MAVLINK_MSG_ID_VISUAL_LOCATION_LEN, MAVLINK_MSG_ID_VISUAL_LOCATION_CRC);
}

/**
 * @brief Encode a visual_location struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param visual_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_visual_location_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_visual_location_t* visual_location)
{
    return mavlink_msg_visual_location_pack(system_id, component_id, msg, visual_location->x, visual_location->y, visual_location->z, visual_location->roll, visual_location->pitch, visual_location->yaw);
}

/**
 * @brief Encode a visual_location struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param visual_location C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_visual_location_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_visual_location_t* visual_location)
{
    return mavlink_msg_visual_location_pack_chan(system_id, component_id, chan, msg, visual_location->x, visual_location->y, visual_location->z, visual_location->roll, visual_location->pitch, visual_location->yaw);
}

/**
 * @brief Send a visual_location message
 * @param chan MAVLink channel to send the message
 *
 * @param x [m] X Position of the landing target on NED_FRAME
 * @param y [m] Y Position of the landing target on NED_FRAME
 * @param z [m] Z Position of the landing target on NED_FRAME
 * @param roll [rad] roll of the landing target on NED_FRAME
 * @param pitch [rad] pitch of the landing target on NED_FRAME
 * @param yaw [rad] yaw of the landing target on NED_FRAME
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_visual_location_send(mavlink_channel_t chan, float x, float y, float z, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_VISUAL_LOCATION_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISUAL_LOCATION, buf, MAVLINK_MSG_ID_VISUAL_LOCATION_MIN_LEN, MAVLINK_MSG_ID_VISUAL_LOCATION_LEN, MAVLINK_MSG_ID_VISUAL_LOCATION_CRC);
#else
    mavlink_visual_location_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISUAL_LOCATION, (const char *)&packet, MAVLINK_MSG_ID_VISUAL_LOCATION_MIN_LEN, MAVLINK_MSG_ID_VISUAL_LOCATION_LEN, MAVLINK_MSG_ID_VISUAL_LOCATION_CRC);
#endif
}

/**
 * @brief Send a visual_location message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_visual_location_send_struct(mavlink_channel_t chan, const mavlink_visual_location_t* visual_location)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_visual_location_send(chan, visual_location->x, visual_location->y, visual_location->z, visual_location->roll, visual_location->pitch, visual_location->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISUAL_LOCATION, (const char *)visual_location, MAVLINK_MSG_ID_VISUAL_LOCATION_MIN_LEN, MAVLINK_MSG_ID_VISUAL_LOCATION_LEN, MAVLINK_MSG_ID_VISUAL_LOCATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_VISUAL_LOCATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_visual_location_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float z, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, roll);
    _mav_put_float(buf, 16, pitch);
    _mav_put_float(buf, 20, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISUAL_LOCATION, buf, MAVLINK_MSG_ID_VISUAL_LOCATION_MIN_LEN, MAVLINK_MSG_ID_VISUAL_LOCATION_LEN, MAVLINK_MSG_ID_VISUAL_LOCATION_CRC);
#else
    mavlink_visual_location_t *packet = (mavlink_visual_location_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISUAL_LOCATION, (const char *)packet, MAVLINK_MSG_ID_VISUAL_LOCATION_MIN_LEN, MAVLINK_MSG_ID_VISUAL_LOCATION_LEN, MAVLINK_MSG_ID_VISUAL_LOCATION_CRC);
#endif
}
#endif

#endif

// MESSAGE VISUAL_LOCATION UNPACKING


/**
 * @brief Get field x from visual_location message
 *
 * @return [m] X Position of the landing target on NED_FRAME
 */
static inline float mavlink_msg_visual_location_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from visual_location message
 *
 * @return [m] Y Position of the landing target on NED_FRAME
 */
static inline float mavlink_msg_visual_location_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from visual_location message
 *
 * @return [m] Z Position of the landing target on NED_FRAME
 */
static inline float mavlink_msg_visual_location_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field roll from visual_location message
 *
 * @return [rad] roll of the landing target on NED_FRAME
 */
static inline float mavlink_msg_visual_location_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pitch from visual_location message
 *
 * @return [rad] pitch of the landing target on NED_FRAME
 */
static inline float mavlink_msg_visual_location_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field yaw from visual_location message
 *
 * @return [rad] yaw of the landing target on NED_FRAME
 */
static inline float mavlink_msg_visual_location_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a visual_location message into a struct
 *
 * @param msg The message to decode
 * @param visual_location C-struct to decode the message contents into
 */
static inline void mavlink_msg_visual_location_decode(const mavlink_message_t* msg, mavlink_visual_location_t* visual_location)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    visual_location->x = mavlink_msg_visual_location_get_x(msg);
    visual_location->y = mavlink_msg_visual_location_get_y(msg);
    visual_location->z = mavlink_msg_visual_location_get_z(msg);
    visual_location->roll = mavlink_msg_visual_location_get_roll(msg);
    visual_location->pitch = mavlink_msg_visual_location_get_pitch(msg);
    visual_location->yaw = mavlink_msg_visual_location_get_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VISUAL_LOCATION_LEN? msg->len : MAVLINK_MSG_ID_VISUAL_LOCATION_LEN;
        memset(visual_location, 0, MAVLINK_MSG_ID_VISUAL_LOCATION_LEN);
    memcpy(visual_location, _MAV_PAYLOAD(msg), len);
#endif
}
