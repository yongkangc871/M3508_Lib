/**
 * m3508.h - RoboMaster M3508 motor + C620 ESC driver (CAN protocol).
 *
 * C620 protocol summary
 *   TX 0x200 : current commands for ESC IDs 1..4  (int16 big-endian x4, -16384..16384 = -20..20 A)
 *   TX 0x1FF : current commands for ESC IDs 5..8
 *   RX 0x200+ID : [0..1] rotor angle 0..8191, [2..3] rotor rpm int16, [4..5] torque current int16, [6] temp C
 *   Feedback rate 1 kHz per motor.
 *
 * Control cascade (the C620 closes the innermost current loop itself, in hardware)
 *   angle (deg) --P--> speed profile --ramp--> speed PID --> current command --CAN--> C620 current loop
 *
 * Units: every public function uses OUTPUT-SHAFT units - rpm, rpm/s, degrees (multi-turn), amps, N.m.
 *
 * Threading model
 *   - CAN ISR      : copies the 8 raw bytes into the motor's mailbox, nothing else
 *   - processTask  : m3508_update_all() decodes feedback and runs ramps + PIDs (1 kHz)
 *   - outputTask   : m3508_send_group() transmits the resulting currents
 *   - Other tasks  : may call the setters / getters below at any time. They never block and never
 *                    touch the CAN bus; the control loop picks a new command up within 1 ms.
 *   Setters and getters use a short critical section: call them from tasks, NOT from interrupts.
 */
#ifndef M3508_H
#define M3508_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_config.h"
#include "pid.h"

#define M3508_CAN_ID_TX_GROUP0   0x200U   /* motors 1..4 */
#define M3508_CAN_ID_TX_GROUP1   0x1FFU   /* motors 5..8 */
#define M3508_CAN_ID_RX_BASE     0x200U   /* feedback = base + ESC id */

typedef enum {
    M3508_MODE_OFF = 0,      /* zero current: motor coasts */
    M3508_MODE_CURRENT,      /* direct torque: current goes straight to the C620, no ramp */
    M3508_MODE_SPEED,        /* speed closed loop with accel / decel ramp */
    M3508_MODE_ANGLE,        /* position cascade with trapezoidal speed profile */
    M3508_MODE_BRAKE,        /* controlled stop, see m3508_brake_t */
} m3508_mode_t;

typedef enum {
    M3508_BRAKE_SOFT = 0,    /* ramp to zero at brake_decel, then hold zero speed          */
    M3508_BRAKE_HOLD,        /* ramp to zero at brake_decel, then lock the position        */
    M3508_BRAKE_HARD,        /* stop as fast as the current ceiling allows, then lock position */
} m3508_brake_t;

/** Motion limits, output-shaft units. A rate of 0 means "no ramp" (instant). */
typedef struct {
    float accel_rpm_s;        /* speeding up                                          */
    float decel_rpm_s;        /* slowing down in speed / angle mode                   */
    float brake_decel_rpm_s;  /* slowing down in brake mode (SOFT / HOLD)             */
    float max_speed_rpm;      /* cruise speed for angle moves and position holding    */
    float current_limit_a;    /* max torque current while driving (also caps MODE_CURRENT) */
    float brake_current_a;    /* max torque current in brake mode (SOFT / HOLD)       */
} m3508_limits_t;

/** Consistent snapshot of one motor, output-shaft units. Filled by m3508_get_state(). */
typedef struct {
    bool         online;          /* feedback seen within M3508_FEEDBACK_TIMEOUT_MS */
    bool         stopped;         /* measured and commanded speed are both ~zero    */
    m3508_mode_t mode;            /* mode the control loop is actually running      */
    float        angle_deg;       /* multi-turn, 0 = position at power-up or m3508_zero_angle() */
    float        speed_rpm;       /* measured */
    float        speed_sp_rpm;    /* ramped setpoint the speed loop is tracking right now */
    float        current_a;       /* measured torque current */
    float        current_cmd_a;   /* current being commanded */
    float        torque_nm;       /* estimated from measured current */
    uint8_t      temperature_c;
    uint32_t     rx_count;        /* feedback frames decoded */
} m3508_state_t;

/* ---- internal types: visible so you can watch them in the debugger, do not write to them ---- */

typedef struct {
    uint16_t ecd;             /* rotor encoder 0..8191 */
    int16_t  rpm;             /* rotor rpm */
    int16_t  current_raw;     /* measured torque current (same scale as command) */
    uint8_t  temperature_c;
    int32_t  round_cnt;       /* full rotor revolutions since init (signed) */
    int64_t  total_ecd;       /* continuous rotor encoder count since init */
    float    angle_out_deg;   /* output-shaft multi-turn angle, deg */
    uint32_t rx_count;
} m3508_feedback_t;

typedef struct {
    uint8_t id;                       /* ESC id 1..8 */

    /* command: written by setters (critical section), read by the control task */
    struct {
        m3508_mode_t  mode;
        m3508_brake_t brake;
        float         target;         /* raw current | rotor rpm | output deg, depending on mode */
    } cmd;

    /* limits in internal units (rotor rpm, rotor rpm/s, raw current) */
    struct {
        float accel, decel, brake_decel, max_rpm;
        float cur_raw, brake_cur_raw;
    } lim;

    pid_ctrl_t speed_pid;
    pid_ctrl_t angle_pid;

    /* control state: owned by the control task */
    m3508_mode_t  act_mode;
    m3508_brake_t act_brake;
    float         sp_rpm;             /* ramped rotor speed setpoint */
    bool          hold_latched;       /* brake has stopped and locked onto hold_angle */
    float         hold_angle;
    bool          stopped;
    bool          online;
    int16_t       current_cmd;        /* last command handed to CAN */

    m3508_feedback_t fb;
    bool             ecd_valid;       /* first frame seen */
    uint16_t         last_ecd;

    /* ISR -> task mailbox */
    volatile uint8_t  rx_raw[8];
    volatile uint32_t rx_seq;         /* incremented by ISR on every frame */
    volatile uint32_t last_rx_tick;   /* HAL tick of last frame */
    uint32_t          rx_seq_done;
} m3508_t;

/* ---- lifecycle ---- */
void      m3508_init_all(void);                 /* call once before the scheduler starts */
m3508_t  *m3508_get(uint8_t id);                /* id 1..M3508_NUM_MOTORS, NULL otherwise */

/* ---- motion commands (non-blocking, take effect on the next 1 kHz cycle) ---- */
void      m3508_set_speed(m3508_t *m, float rpm);        /* ramps at accel / decel            */
void      m3508_set_angle(m3508_t *m, float deg);        /* absolute, multi-turn               */
void      m3508_move_angle(m3508_t *m, float delta_deg); /* relative to the current target     */
void      m3508_set_current(m3508_t *m, float amps);     /* instant torque, no ramp            */
void      m3508_set_torque(m3508_t *m, float nm);        /* same, in N.m at the output shaft   */
void      m3508_brake(m3508_t *m, m3508_brake_t type);   /* controlled stop                    */
void      m3508_coast(m3508_t *m);                       /* zero current, free-wheel           */
void      m3508_brake_all(m3508_brake_t type);
void      m3508_coast_all(void);

/* ---- configuration (safe to change while running) ---- */
void      m3508_set_limits(m3508_t *m, const m3508_limits_t *lim);
void      m3508_get_limits(m3508_t *m, m3508_limits_t *lim);
void      m3508_set_accel(m3508_t *m, float accel_rpm_s, float decel_rpm_s);
void      m3508_set_current_limit(m3508_t *m, float amps);
void      m3508_set_brake(m3508_t *m, float brake_current_a, float brake_decel_rpm_s);
void      m3508_set_max_speed(m3508_t *m, float rpm);
void      m3508_set_speed_pid(m3508_t *m, float kp, float ki, float kd);
void      m3508_set_angle_pid(m3508_t *m, float kp, float ki, float kd);
void      m3508_zero_angle(m3508_t *m);         /* current position becomes 0 deg. Not while in angle mode. */

/* ---- read back ---- */
void      m3508_get_state(m3508_t *m, m3508_state_t *out);
bool      m3508_is_online(m3508_t *m);
bool      m3508_is_stopped(m3508_t *m);

/* ---- control loop: m3508_update_all() from processTask, m3508_send_group() from outputTask ---- */
void      m3508_update_all(float dt_s);            /* decode feedback + run ramps and PIDs for all motors */
HAL_StatusTypeDef m3508_send_group(uint8_t group); /* 0 -> 0x200 (ids 1..4), 1 -> 0x1FF (ids 5..8) */

/* ---- CAN glue (registered automatically by m3508_init_all) ---- */
void      m3508_can_rx_handler(uint32_t std_id, const uint8_t data[8], uint8_t len);

#endif /* M3508_H */
