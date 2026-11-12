#include "chassis.h"
#include "m3508.h"
#include "robot_config.h"
#include "util.h"
#include <math.h>

/* wheel surface speed (m/s) <-> output-shaft rpm */
#define MPS_TO_RPM   (60.0f / (2.0f * 3.14159265f * CHASSIS_WHEEL_RADIUS_M))
#define RPM_TO_MPS   (1.0f / MPS_TO_RPM)
/* lever arm that turns wz into wheel speed on a mecanum base */
#define K_ROT        (CHASSIS_HALF_BASE_M + CHASSIS_HALF_TRACK_M)

static m3508_t      *s_motor[CH_NUM_WHEELS];
static const float   s_dir[CH_NUM_WHEELS] = { CHASSIS_DIR_FL, CHASSIS_DIR_FR, CHASSIS_DIR_RL, CHASSIS_DIR_RR };

static float           s_sp_vx, s_sp_vy, s_sp_wz;   /* ramped robot velocity */
static chassis_state_t s_chassis;                     /* snapshot for other tasks */

/* ---------------- helpers ---------------- */

/** Move (x, y) toward (tx, ty) along a straight line, by at most rate*dt. */
static void ramp_vec2(float *x, float *y, float tx, float ty, float rate, float dt)
{
    const float dx = tx - *x, dy = ty - *y;
    const float dist = sqrtf(dx * dx + dy * dy);
    const float step = rate * dt;
    if (dist <= step) {
        *x = tx; *y = ty;
    } else {
        const float k = step / dist;
        *x += dx * k; *y += dy * k;
    }
}

static float ramp_scalar(float v, float target, float rate, float dt)
{
    const float step = rate * dt;
    if (target > v + step) return v + step;
    if (target < v - step) return v - step;
    return target;
}

static void coast_all(void)
{
    for (int i = 0; i < CH_NUM_WHEELS; i++) m3508_coast(s_motor[i]);
}

/* ---------------- API ---------------- */

void chassis_init(void)
{
    s_motor[CH_FL] = m3508_get(CHASSIS_ID_FL);
    s_motor[CH_FR] = m3508_get(CHASSIS_ID_FR);
    s_motor[CH_RL] = m3508_get(CHASSIS_ID_RL);
    s_motor[CH_RR] = m3508_get(CHASSIS_ID_RR);

    /* The chassis ramps the whole robot, so the per-motor ramps are switched off: each wheel
       follows its setpoint directly and none of them lags behind the others. */
    for (int i = 0; i < CH_NUM_WHEELS; i++) m3508_set_accel(s_motor[i], 0.0f, 0.0f);

    s_sp_vx = s_sp_vy = s_sp_wz = 0.0f;
}

void chassis_update(const cmd_vel_t *cmd, float dt)
{
    /* ---- 1. measure: wheel speeds -> robot velocity (forward kinematics) ---- */
    float w[CH_NUM_WHEELS];          /* wheel surface speed, m/s, + = robot forward */
    bool  all_online = true;
    for (int i = 0; i < CH_NUM_WHEELS; i++) {
        const float rpm = (float)s_motor[i]->fb.rpm * (1.0f / M3508_GEAR_RATIO);
        s_chassis.wheel_rpm[i] = rpm;
        w[i] = rpm * s_dir[i] * RPM_TO_MPS;
        if (!s_motor[i]->online) all_online = false;
    }
    const float meas_vx = ( w[CH_FL] + w[CH_FR] + w[CH_RL] + w[CH_RR]) * 0.25f;
    const float meas_vy = (-w[CH_FL] + w[CH_FR] + w[CH_RL] - w[CH_RR]) * 0.25f;
    const float meas_wz = (-w[CH_FL] + w[CH_FR] - w[CH_RL] + w[CH_RR]) * (0.25f / K_ROT);

    /* ---- 2. decide what to do ---- */
    chassis_mode_t mode;
    if (CHASSIS_REQUIRE_ALL_MOTORS && !all_online) mode = CHASSIS_MOTOR_FAULT;
    else if (!cmd->ever_received)                  mode = CHASSIS_IDLE;
    else if (!cmd->online)                         mode = CHASSIS_FAILSAFE;
    else                                           mode = CHASSIS_DRIVE;

    /* command, limited to what the robot is allowed to do (cmd is already zero when offline) */
    float tvx = cmd->vx, tvy = cmd->vy;
    const float lin = sqrtf(tvx * tvx + tvy * tvy);
    if (lin > CHASSIS_MAX_LIN_MPS) {             /* shrink the vector, keep its direction */
        const float k = CHASSIS_MAX_LIN_MPS / lin;
        tvx *= k; tvy *= k;
    }
    const float twz = clampf(cmd->wz, -CHASSIS_MAX_ANG_RPS, CHASSIS_MAX_ANG_RPS);

    float wheel_sp[CH_NUM_WHEELS] = {0};

    if (mode == CHASSIS_IDLE || mode == CHASSIS_MOTOR_FAULT) {
        /* Not driving. Keep the ramp at the real velocity so driving resumes without a jump. */
        coast_all();
        s_sp_vx = meas_vx; s_sp_vy = meas_vy; s_sp_wz = meas_wz;
    } else {
        /* ---- 3. ramp the robot velocity (this is what synchronises the wheels) ---- */
        const bool  brake    = (mode == CHASSIS_FAILSAFE);
        const bool  slowing  = (tvx * tvx + tvy * tvy) < (s_sp_vx * s_sp_vx + s_sp_vy * s_sp_vy);
        const float rate_lin = brake ? CHASSIS_BRAKE_LIN : (slowing ? CHASSIS_DECEL_LIN : CHASSIS_ACCEL_LIN);
        const float rate_ang = brake ? CHASSIS_BRAKE_ANG
                             : ((fabsf(twz) < fabsf(s_sp_wz)) ? CHASSIS_DECEL_ANG : CHASSIS_ACCEL_ANG);

        ramp_vec2(&s_sp_vx, &s_sp_vy, tvx, tvy, rate_lin, dt);
        s_sp_wz = ramp_scalar(s_sp_wz, twz, rate_ang, dt);

        /* ---- 4. inverse kinematics: robot velocity -> wheel surface speeds ---- */
        float v[CH_NUM_WHEELS];
        v[CH_FL] = s_sp_vx - s_sp_vy - K_ROT * s_sp_wz;
        v[CH_FR] = s_sp_vx + s_sp_vy + K_ROT * s_sp_wz;
        v[CH_RL] = s_sp_vx + s_sp_vy - K_ROT * s_sp_wz;
        v[CH_RR] = s_sp_vx - s_sp_vy + K_ROT * s_sp_wz;

        /* ---- 5. to motor rpm; if one wheel is over the limit, scale all four equally ---- */
        float peak = 0.0f;
        for (int i = 0; i < CH_NUM_WHEELS; i++) {
            wheel_sp[i] = v[i] * MPS_TO_RPM * s_dir[i];
            const float a = fabsf(wheel_sp[i]);
            if (a > peak) peak = a;
        }
        const float scale = (peak > CHASSIS_MAX_WHEEL_RPM) ? (CHASSIS_MAX_WHEEL_RPM / peak) : 1.0f;

        /* ---- 6. hand the setpoints to the speed loops ---- */
        for (int i = 0; i < CH_NUM_WHEELS; i++) {
            wheel_sp[i] *= scale;
            m3508_set_speed(s_motor[i], wheel_sp[i]);
        }
    }

    /* ---- 7. publish a snapshot for other tasks / the debugger ---- */
    taskENTER_CRITICAL();
    s_chassis.mode          = mode;
    s_chassis.motors_online = all_online;
    s_chassis.cmd_vx = tvx;      s_chassis.cmd_vy = tvy;      s_chassis.cmd_wz = twz;
    s_chassis.sp_vx  = s_sp_vx;  s_chassis.sp_vy  = s_sp_vy;  s_chassis.sp_wz  = s_sp_wz;
    s_chassis.meas_vx = meas_vx; s_chassis.meas_vy = meas_vy; s_chassis.meas_wz = meas_wz;
    for (int i = 0; i < CH_NUM_WHEELS; i++) s_chassis.wheel_sp_rpm[i] = wheel_sp[i];
    taskEXIT_CRITICAL();
}

void chassis_get_state(chassis_state_t *out)
{
    if (!out) return;
    taskENTER_CRITICAL();
    *out = s_chassis;
    taskEXIT_CRITICAL();
}
