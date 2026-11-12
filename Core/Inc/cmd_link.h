/**
 * cmd_link.h - velocity commands from the Jetson over USART1.
 *
 * Frame (18 bytes, little-endian), sent by tools/jetson_cmd_sender.py:
 *
 *   0xA5 0x5A | len = 13 | seq (u8) | vx (f32, m/s) | vy (f32, m/s) | wz (f32, rad/s) | crc16 (u16)
 *
 *   crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over the len byte and the payload.
 *   Robot frame: x forward, y left, wz counter-clockwise (same as ROS cmd_vel).
 *
 * The parser resynchronises on the two header bytes, so garbage, partial frames and corrupted
 * frames are dropped without affecting the next good one.
 *
 * Failsafe: if no valid frame arrives for CMD_TIMEOUT_MS, cmd_link_get() reports online = false
 * and a zero velocity.
 */
#ifndef CMD_LINK_H
#define CMD_LINK_H

#include <stdint.h>
#include <stdbool.h>

#define CMD_PAYLOAD_LEN   13U
#define CMD_FRAME_LEN     (2U + 1U + CMD_PAYLOAD_LEN + 2U)

typedef struct {
    bool     online;           /* a valid frame arrived within CMD_TIMEOUT_MS      */
    bool     ever_received;    /* at least one valid frame since boot              */
    float    vx, vy;           /* m/s  (0 when offline)                            */
    float    wz;               /* rad/s (0 when offline)                           */
    uint8_t  seq;              /* sender's frame counter                           */
    uint32_t age_ms;           /* time since the last valid frame                  */
} cmd_vel_t;

typedef struct {
    uint32_t bytes;            /* bytes fed to the parser                          */
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t len_errors;       /* header found but length byte wrong               */
    uint32_t value_errors;     /* frame valid but a number was NaN / infinite      */
    uint32_t lost_frames;      /* gaps in the sender's sequence counter            */
} cmd_link_stats_t;

void cmd_link_init(void);
/** Feed received bytes to the parser. Called by inputTask only. */
void cmd_link_feed(const uint8_t *data, uint32_t len);
/** Latest command with the failsafe applied. Call from any task. */
void cmd_link_get(cmd_vel_t *out);
void cmd_link_get_stats(cmd_link_stats_t *out);

#endif /* CMD_LINK_H */
