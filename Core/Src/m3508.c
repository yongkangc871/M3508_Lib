#include "m3508.h"
#include "bsp_can.h"
#include "util.h"
#include <math.h>
#include <string.h>

static m3508_t s_motors[M3508_NUM_MOTORS];

#define A_TO_RAW        ((float)M3508_CURRENT_RAW_MAX / M3508_CURRENT_A_MAX)
#define RAW_TO_A        (M3508_CURRENT_A_MAX / (float)M3508_CURRENT_RAW_MAX)
#define DEG_PER_ECD     (360.0f / ((float)M3508_ECD_RANGE * M3508_GEAR_RATIO))

/* ---------------- helpers ---------------- */

static inline float amps_to_raw_limit(float amps)
{
    return clampf(amps, 0.0f, M3508_CURRENT_A_MAX) * A_TO_RAW;
}

static inline bool is_closed_loop(m3508_mode_t mode)
{
    return mode == M3508_MODE_SPEED || mode == M3508_MODE_ANGLE || mode == M3508_MODE_BRAKE;
}

/**
 * Move sp toward tgt by at most one ramp step.
 * "Speeding up" = moving away from zero without changing direction (uses accel); everything else
 * uses decel. A reversal first ramps down to zero, then speeds up the other way. Rate 0 = no ramp.
 */
static float ramp_to(float sp, float tgt, float accel, float decel, float dt)
{
    const bool  speeding_up = (sp * tgt >= 0.0f) && (fabsf(tgt) > fabsf(sp));
    const float rate = speeding_up ? accel : decel;
    if (rate <= 0.0f) return tgt;

    const float step = rate * dt;
    const float goal = (sp * tgt < 0.0f) ? 0.0f : tgt;
    if (goal > sp + step) return sp + step;
    if (goal < sp - step) return sp - step;
    return goal;
}

static void set_cmd(m3508_t *m, m3508_mode_t mode, float target, m3508_brake_t brake)
{
    if (!m) return;
    crit_enter();
    m->cmd.target = target;
    m->cmd.brake  = brake;
    m->cmd.mode   = mode;
    crit_exit();
}

/* ---------------- lifecycle ---------------- */

void m3508_init_all(void)
{
    static const m3508_limits_t def = {
        .accel_rpm_s       = M3508_DEF_ACCEL_RPM_S,
        .decel_rpm_s       = M3508_DEF_DECEL_RPM_S,
        .brake_decel_rpm_s = M3508_DEF_BRAKE_DECEL_RPM_S,
        .max_speed_rpm     = M3508_DEF_MAX_SPEED_RPM,
        .current_limit_a   = M3508_DEF_CURRENT_A,
        .brake_current_a   = M3508_DEF_BRAKE_CURRENT_A,
    };

    memset(s_motors, 0, sizeof(s_motors));
    for (uint8_t i = 0; i < M3508_NUM_MOTORS; i++) {
        m3508_t *m = &s_motors[i];
        m->id = (uint8_t)(i + 1);
        pid_init(&m->speed_pid, M3508_SPEED_KP, M3508_SPEED_KI, M3508_SPEED_KD,
                 (float)M3508_CURRENT_RAW_LIMIT, M3508_SPEED_I_MAX);
        pid_init(&m->angle_pid, M3508_ANGLE_KP, M3508_ANGLE_KI, M3508_ANGLE_KD,
                 M3508_ROTOR_RPM_MAX, M3508_ANGLE_I_MAX);
        m3508_set_limits(m, &def);
    }
    bsp_can_register_rx(m3508_can_rx_handler);
}

m3508_t *m3508_get(uint8_t id)
{
    if (id == 0 || id > M3508_NUM_MOTORS) return 0;
    return &s_motors[id - 1];
}

/* ---------------- motion commands ---------------- */

void m3508_set_speed(m3508_t *m, float rpm)
{
    float rotor = clampf(rpm * M3508_GEAR_RATIO, -M3508_ROTOR_RPM_MAX, M3508_ROTOR_RPM_MAX);
    set_cmd(m, M3508_MODE_SPEED, rotor, M3508_BRAKE_SOFT);
}

void m3508_set_angle(m3508_t *m, float deg)
{
    set_cmd(m, M3508_MODE_ANGLE, deg, M3508_BRAKE_SOFT);
}

void m3508_move_angle(m3508_t *m, float delta_deg)
{
    if (!m) return;
    crit_enter();
    float base = (m->cmd.mode == M3508_MODE_ANGLE) ? m->cmd.target : m->fb.angle_out_deg;
    m->cmd.target = base + delta_deg;
    m->cmd.mode   = M3508_MODE_ANGLE;
    crit_exit();
}

void m3508_set_current(m3508_t *m, float amps)
{
    float raw = clampf(amps, -M3508_CURRENT_A_MAX, M3508_CURRENT_A_MAX) * A_TO_RAW;
    set_cmd(m, M3508_MODE_CURRENT, raw, M3508_BRAKE_SOFT);
}

void m3508_set_torque(m3508_t *m, float nm)
{
    m3508_set_current(m, nm / M3508_TORQUE_CONST_NM_A);
}

void m3508_brake(m3508_t *m, m3508_brake_t type)
{
    set_cmd(m, M3508_MODE_BRAKE, 0.0f, type);
}

void m3508_coast(m3508_t *m)
{
    set_cmd(m, M3508_MODE_OFF, 0.0f, M3508_BRAKE_SOFT);
}

void m3508_brake_all(m3508_brake_t type)
{
    for (uint8_t i = 0; i < M3508_NUM_MOTORS; i++) m3508_brake(&s_motors[i], type);
}

void m3508_coast_all(void)
{
    for (uint8_t i = 0; i < M3508_NUM_MOTORS; i++) m3508_coast(&s_motors[i]);
}

/* ---------------- configuration ---------------- */

void m3508_set_limits(m3508_t *m, const m3508_limits_t *lim)
{
    if (!m || !lim) return;
    const float accel       = fmaxf(lim->accel_rpm_s, 0.0f)       * M3508_GEAR_RATIO;
    const float decel       = fmaxf(lim->decel_rpm_s, 0.0f)       * M3508_GEAR_RATIO;
    const float brake_decel = fmaxf(lim->brake_decel_rpm_s, 0.0f) * M3508_GEAR_RATIO;
    const float max_rpm     = clampf(lim->max_speed_rpm * M3508_GEAR_RATIO, 0.0f, M3508_ROTOR_RPM_MAX);
    const float cur         = amps_to_raw_limit(lim->current_limit_a);
    const float brake_cur   = amps_to_raw_limit(lim->brake_current_a);

    crit_enter();
    m->lim.accel         = accel;
    m->lim.decel         = decel;
    m->lim.brake_decel   = brake_decel;
    m->lim.max_rpm       = max_rpm;
    m->lim.cur_raw       = cur;
    m->lim.brake_cur_raw = brake_cur;
    crit_exit();
}

void m3508_get_limits(m3508_t *m, m3508_limits_t *lim)
{
    if (!m || !lim) return;
    crit_enter();
    lim->accel_rpm_s       = m->lim.accel / M3508_GEAR_RATIO;
    lim->decel_rpm_s       = m->lim.decel / M3508_GEAR_RATIO;
    lim->brake_decel_rpm_s = m->lim.brake_decel / M3508_GEAR_RATIO;
    lim->max_speed_rpm     = m->lim.max_rpm / M3508_GEAR_RATIO;
    lim->current_limit_a   = m->lim.cur_raw * RAW_TO_A;
    lim->brake_current_a   = m->lim.brake_cur_raw * RAW_TO_A;
    crit_exit();
}

void m3508_set_accel(m3508_t *m, float accel_rpm_s, float decel_rpm_s)
{
    m3508_limits_t l;
    if (!m) return;
    m3508_get_limits(m, &l);
    l.accel_rpm_s = accel_rpm_s;
    l.decel_rpm_s = decel_rpm_s;
    m3508_set_limits(m, &l);
}

void m3508_set_current_limit(m3508_t *m, float amps)
{
    m3508_limits_t l;
    if (!m) return;
    m3508_get_limits(m, &l);
    l.current_limit_a = amps;
    m3508_set_limits(m, &l);
}

void m3508_set_brake(m3508_t *m, float brake_current_a, float brake_decel_rpm_s)
{
    m3508_limits_t l;
    if (!m) return;
    m3508_get_limits(m, &l);
    l.brake_current_a   = brake_current_a;
    l.brake_decel_rpm_s = brake_decel_rpm_s;
    m3508_set_limits(m, &l);
}

void m3508_set_max_speed(m3508_t *m, float rpm)
{
    m3508_limits_t l;
    if (!m) return;
    m3508_get_limits(m, &l);
    l.max_speed_rpm = rpm;
    m3508_set_limits(m, &l);
}

void m3508_set_speed_pid(m3508_t *m, float kp, float ki, float kd)
{
    if (!m) return;
    crit_enter();
    pid_set_gains(&m->speed_pid, kp, ki, kd);
    crit_exit();
}

void m3508_set_angle_pid(m3508_t *m, float kp, float ki, float kd)
{
    if (!m) return;
    crit_enter();
    pid_set_gains(&m->angle_pid, kp, ki, kd);
    crit_exit();
}

void m3508_zero_angle(m3508_t *m)
{
    if (!m) return;
    crit_enter();
    m->fb.round_cnt     = 0;
    m->fb.total_ecd     = 0;
    m->fb.angle_out_deg = 0.0f;
    m->hold_angle       = 0.0f;
    crit_exit();
}

/* ---------------- read back ---------------- */

void m3508_get_state(m3508_t *m, m3508_state_t *out)
{
    if (!m || !out) return;
    crit_enter();
    out->online        = m->online;
    out->stopped       = m->stopped;
    out->mode          = m->act_mode;
    out->angle_deg     = m->fb.angle_out_deg;
    out->speed_rpm     = (float)m->fb.rpm / M3508_GEAR_RATIO;
    out->speed_sp_rpm  = m->sp_rpm / M3508_GEAR_RATIO;
    out->current_a     = (float)m->fb.current_raw * RAW_TO_A;
    out->current_cmd_a = (float)m->current_cmd * RAW_TO_A;
    out->temperature_c = m->fb.temperature_c;
    out->rx_count      = m->fb.rx_count;
    crit_exit();
    out->torque_nm     = out->current_a * M3508_TORQUE_CONST_NM_A;
}

bool m3508_is_online(m3508_t *m)  { return m && m->online; }
bool m3508_is_stopped(m3508_t *m) { return m && m->stopped; }

/* ---------------- CAN RX (ISR context) ---------------- */

void m3508_can_rx_handler(uint32_t std_id, const uint8_t data[8], uint8_t len)
{
    if (len < 7U) return;
    uint32_t id = std_id - M3508_CAN_ID_RX_BASE;         /* 1..8 */
    if (id == 0 || id > M3508_NUM_MOTORS) return;
    m3508_t *m = &s_motors[id - 1];

    /* Copy raw bytes only - keep the ISR short. Decoding happens in the control task. */
    for (int i = 0; i < 8; i++) m->rx_raw[i] = data[i];
    m->rx_seq++;
    m->last_rx_tick = HAL_GetTick();
}

/* ---------------- decode (control task) ---------------- */

static void m3508_decode(m3508_t *m)
{
    uint8_t raw[8];

    /* Snapshot the mailbox atomically against the ISR. */
    taskENTER_CRITICAL();
    uint32_t seq = m->rx_seq;
    if (seq == m->rx_seq_done) {
        taskEXIT_CRITICAL();
        return;
    }
    for (int i = 0; i < 8; i++) raw[i] = m->rx_raw[i];
    m->rx_seq_done = seq;
    taskEXIT_CRITICAL();

    m3508_feedback_t *fb = &m->fb;
    fb->ecd           = (uint16_t)((raw[0] << 8) | raw[1]);
    fb->rpm           = (int16_t) ((raw[2] << 8) | raw[3]);
    fb->current_raw   = (int16_t) ((raw[4] << 8) | raw[5]);
    fb->temperature_c = raw[6];
    fb->rx_count++;

    if (!m->ecd_valid) {
        m->last_ecd  = fb->ecd;
        m->ecd_valid = true;
    }

    /* Multi-turn tracking: wrap detection on the 8192-count rotor encoder */
    int32_t delta = (int32_t)fb->ecd - (int32_t)m->last_ecd;
    if (delta > M3508_ECD_HALF) {
        fb->round_cnt--;
        delta -= M3508_ECD_RANGE;
    } else if (delta < -M3508_ECD_HALF) {
        fb->round_cnt++;
        delta += M3508_ECD_RANGE;
    }
    fb->total_ecd += delta;
    m->last_ecd = fb->ecd;

    fb->angle_out_deg = (float)fb->total_ecd * DEG_PER_ECD;
}

/* ---------------- control (control task, 1 kHz) ---------------- */

/** Outer position loop: P controller capped by a "can still stop in time" speed, i.e. a trapezoid. */
static float angle_to_rpm(m3508_t *m, float target_deg, float dt)
{
    const float angle = m->fb.angle_out_deg;
    m->angle_pid.out_max = m->lim.max_rpm;
    float v = pid_update(&m->angle_pid, target_deg, angle, dt);

    if (m->lim.decel > 0.0f) {
        /* v^2 = 2*a*d with v in rpm, a in rpm/s, d in rotor revolutions -> v^2 = 120*a*d */
        const float dist_rev = fabsf(target_deg - angle) * (M3508_GEAR_RATIO / 360.0f);
        const float v_stop   = sqrtf(120.0f * m->lim.decel * dist_rev);
        v = clampf(v, -v_stop, v_stop);
    }
    return v;
}

static void m3508_update_one(m3508_t *m, float dt)
{
    m3508_decode(m);

    /* Read the ISR-written tick first: an interrupt between the two reads can then only make
       the age smaller, never wrap it negative. */
    const uint32_t last_rx = m->last_rx_tick;
    const uint32_t age     = HAL_GetTick() - last_rx;
    const bool     online  = m->ecd_valid && (age <= M3508_FEEDBACK_TIMEOUT_MS);

    if (!online) {                 /* safety: never drive a motor we cannot hear */
        m->online      = false;
        m->stopped     = false;
        m->current_cmd = 0;
        m->sp_rpm      = 0.0f;
        m->act_mode    = M3508_MODE_OFF;   /* forces a bumpless restart when it comes back */
        return;
    }
    m->online = true;

    /* This task has the highest priority and setters are atomic (critical section), so a plain
       copy here is a consistent command. */
    const m3508_mode_t  mode  = m->cmd.mode;
    const m3508_brake_t brake = m->cmd.brake;
    const float         tgt   = m->cmd.target;
    const float         rpm   = (float)m->fb.rpm;

    /* ---- bumpless mode entry ---- */
    if (mode != m->act_mode) {
        if (!is_closed_loop(m->act_mode)) {
            /* Coming from coast / direct current: start the ramp at the real speed and preload the
               integrator with the current already flowing, so torque does not jump. */
            m->sp_rpm = rpm;
            pid_seed(&m->speed_pid, (float)m->current_cmd, rpm);
        }
        /* Between closed-loop modes the speed loop simply keeps running: no reset, no jerk. */
        pid_reset(&m->angle_pid);
        m->hold_latched = false;
        m->act_mode     = mode;
    }
    if (brake != m->act_brake) {
        m->hold_latched = false;
        m->act_brake    = brake;
    }

    float out     = 0.0f;
    float cur_lim = m->lim.cur_raw;

    switch (mode) {
    case M3508_MODE_CURRENT:
        m->sp_rpm = rpm;                                   /* keep the ramp tracking for a clean hand-over */
        out = clampf(tgt, -cur_lim, cur_lim);
        break;

    case M3508_MODE_SPEED:
        m->sp_rpm = ramp_to(m->sp_rpm, tgt, m->lim.accel, m->lim.decel, dt);
        break;

    case M3508_MODE_ANGLE: {
        const float v = angle_to_rpm(m, tgt, dt);
        /* The profile already slows down at `decel`; give the ramp headroom so it does not lag it. */
        m->sp_rpm = ramp_to(m->sp_rpm, v, m->lim.accel, 2.0f * m->lim.decel, dt);
        break;
    }

    case M3508_MODE_BRAKE:
        cur_lim = (brake == M3508_BRAKE_HARD) ? (float)M3508_CURRENT_RAW_LIMIT : m->lim.brake_cur_raw;
        if (!m->hold_latched) {
            m->sp_rpm = (brake == M3508_BRAKE_HARD)
                      ? 0.0f
                      : ramp_to(m->sp_rpm, 0.0f, m->lim.brake_decel, m->lim.brake_decel, dt);
            if (brake != M3508_BRAKE_SOFT && m->sp_rpm == 0.0f && fabsf(rpm) < M3508_STOP_RPM) {
                m->hold_latched = true;                    /* stopped: lock onto this position */
                m->hold_angle   = m->fb.angle_out_deg;
                pid_reset(&m->angle_pid);
            }
        }
        if (m->hold_latched) {
            m->angle_pid.out_max = m->lim.max_rpm;
            m->sp_rpm = pid_update(&m->angle_pid, m->hold_angle, m->fb.angle_out_deg, dt);
        }
        break;

    case M3508_MODE_OFF:
    default:
        m->sp_rpm = rpm;
        out = 0.0f;
        break;
    }

    if (is_closed_loop(mode)) {
        m->speed_pid.out_max = cur_lim;
        out = pid_update(&m->speed_pid, m->sp_rpm, rpm, dt);
    }

    m->stopped     = (fabsf(rpm) < M3508_STOP_RPM) && (fabsf(m->sp_rpm) < M3508_STOP_RPM);
    m->current_cmd = (int16_t)clampf(out, -(float)M3508_CURRENT_RAW_LIMIT, (float)M3508_CURRENT_RAW_LIMIT);
}

void m3508_update_all(float dt_s)
{
    for (uint8_t i = 0; i < M3508_NUM_MOTORS; i++) {
        m3508_update_one(&s_motors[i], dt_s);
    }
}

HAL_StatusTypeDef m3508_send_group(uint8_t group)
{
    uint8_t data[8] = {0};
    uint8_t base = (uint8_t)(group * 4U);             /* index of first motor in this group */
    for (uint8_t k = 0; k < 4; k++) {
        uint8_t idx = (uint8_t)(base + k);
        int16_t c = (idx < M3508_NUM_MOTORS) ? s_motors[idx].current_cmd : 0;
        data[2 * k]     = (uint8_t)((uint16_t)c >> 8);
        data[2 * k + 1] = (uint8_t)((uint16_t)c & 0xFF);
    }
    uint32_t id = (group == 0) ? M3508_CAN_ID_TX_GROUP0 : M3508_CAN_ID_TX_GROUP1;
    return bsp_can_send_std(&ROBOT_CAN_HANDLE, id, data);
}
