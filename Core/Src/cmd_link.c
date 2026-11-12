#include "cmd_link.h"
#include "robot_config.h"
#include "util.h"
#include <math.h>
#include <string.h>

#define SOF1  0xA5U
#define SOF2  0x5AU

typedef enum { ST_SOF1, ST_SOF2, ST_LEN, ST_PAYLOAD, ST_CRC_LO, ST_CRC_HI } parse_state_t;

/* parser state: inputTask only */
static parse_state_t s_st;
static uint8_t       s_payload[CMD_PAYLOAD_LEN];
static uint32_t      s_idx;
static uint16_t      s_crc;          /* running CRC over len + payload */
static uint8_t       s_crc_lo;
static bool          s_have_seq;
static uint8_t       s_last_seq;

/* latest command: written by inputTask, read by anyone (critical section) */
static float    s_vx, s_vy, s_wz;
static uint8_t  s_seq;
static uint32_t s_last_tick;
static bool     s_valid;             /* at least one frame received */

static cmd_link_stats_t s_link_stats;

/* CRC-16/CCITT-FALSE, one byte at a time */
static uint16_t crc16_step(uint16_t crc, uint8_t b)
{
    crc ^= (uint16_t)((uint16_t)b << 8);
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void accept_frame(void)
{
    const uint8_t seq = s_payload[0];
    if (s_have_seq) {
        s_link_stats.lost_frames += (uint8_t)(seq - s_last_seq - 1U);
    }
    s_last_seq = seq;
    s_have_seq = true;

    float v[3];
    memcpy(v, &s_payload[1], sizeof(v));     /* both sides are little-endian IEEE-754 */
    if (!isfinite(v[0]) || !isfinite(v[1]) || !isfinite(v[2])) {
        s_link_stats.value_errors++;              /* never let a NaN reach the control loop */
        return;
    }

    crit_enter();
    s_vx = v[0];
    s_vy = v[1];
    s_wz = v[2];
    s_seq       = seq;
    s_last_tick = HAL_GetTick();
    s_valid     = true;
    s_link_stats.frames_ok++;
    crit_exit();
}

void cmd_link_init(void)
{
    s_st = ST_SOF1;
    s_valid = false;
    s_have_seq = false;
    memset(&s_link_stats, 0, sizeof(s_link_stats));
}

void cmd_link_feed(const uint8_t *data, uint32_t len)
{
    s_link_stats.bytes += len;

    for (uint32_t i = 0; i < len; i++) {
        const uint8_t b = data[i];

        switch (s_st) {
        case ST_SOF1:
            if (b == SOF1) s_st = ST_SOF2;
            break;

        case ST_SOF2:
            if (b == SOF2)      s_st = ST_LEN;
            else if (b != SOF1) s_st = ST_SOF1;     /* A5 A5 5A still syncs */
            break;

        case ST_LEN:
            if (b == CMD_PAYLOAD_LEN) {
                s_crc = crc16_step(0xFFFFU, b);
                s_idx = 0;
                s_st  = ST_PAYLOAD;
            } else {
                s_link_stats.len_errors++;
                s_st = (b == SOF1) ? ST_SOF2 : ST_SOF1;
            }
            break;

        case ST_PAYLOAD:
            s_payload[s_idx++] = b;
            s_crc = crc16_step(s_crc, b);
            if (s_idx >= CMD_PAYLOAD_LEN) s_st = ST_CRC_LO;
            break;

        case ST_CRC_LO:
            s_crc_lo = b;
            s_st = ST_CRC_HI;
            break;

        case ST_CRC_HI:
        default:
            if ((uint16_t)(s_crc_lo | ((uint16_t)b << 8)) == s_crc) accept_frame();
            else                                                   s_link_stats.crc_errors++;
            s_st = ST_SOF1;
            break;
        }
    }
}

void cmd_link_get(cmd_vel_t *out)
{
    if (!out) return;

    crit_enter();
    const uint32_t last = s_last_tick;
    out->ever_received = s_valid;
    out->vx  = s_vx;
    out->vy  = s_vy;
    out->wz  = s_wz;
    out->seq = s_seq;
    crit_exit();

    out->age_ms = HAL_GetTick() - last;
    out->online = out->ever_received && (out->age_ms <= CMD_TIMEOUT_MS);

    if (!out->online) {                      /* failsafe: ask for a standstill */
        out->vx = 0.0f;
        out->vy = 0.0f;
        out->wz = 0.0f;
    }
}

void cmd_link_get_stats(cmd_link_stats_t *out)
{
    if (!out) return;
    crit_enter();
    *out = s_link_stats;
    crit_exit();
}
