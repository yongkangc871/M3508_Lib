/**
 * chassis.h - mecanum chassis: robot velocity (m/s, rad/s) -> four synchronised wheel speeds.
 *
 * What keeps the four wheels in step
 *   - Acceleration is limited on the ROBOT velocity (vx, vy as one vector, wz separately), and the
 *     wheel speeds are computed from that. All four therefore ramp in proportion, and the robot
 *     keeps its direction while speeding up or slowing down.
 *   - If any wheel would exceed CHASSIS_MAX_WHEEL_RPM, all four are scaled by the same factor.
 *   - All four speed loops run in the same 1 kHz cycle and their currents leave in ONE CAN frame.
 *   - With CHASSIS_REQUIRE_ALL_MOTORS = 1, the chassis only drives when all four motors answer.
 *
 * Behaviour by situation
 *   - No command received since boot ........ motors coast (safe for bring-up)
 *   - Commands arriving ...................... follow them, with accel / decel limits
 *   - Commands stop (CMD_TIMEOUT_MS) ......... brake to a standstill at the brake rate, then hold
 *   - A motor is offline (REQUIRE_ALL = 1) ... all motors coast until it returns
 *
 * Frame: x forward, y left, wz counter-clockwise. Wheel order everywhere: FL, FR, RL, RR.
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include "cmd_link.h"

enum { CH_FL = 0, CH_FR, CH_RL, CH_RR, CH_NUM_WHEELS };

typedef enum {
    CHASSIS_IDLE = 0,      /* no command since boot: coasting              */
    CHASSIS_DRIVE,         /* following commands                           */
    CHASSIS_FAILSAFE,      /* command link lost: braking / holding still   */
    CHASSIS_MOTOR_FAULT,   /* a motor is offline: coasting                 */
} chassis_mode_t;

typedef struct {
    chassis_mode_t mode;
    bool  motors_online;                 /* all four motors answering              */
    float cmd_vx, cmd_vy, cmd_wz;        /* command after the speed limits         */
    float sp_vx,  sp_vy,  sp_wz;         /* ramped setpoint the wheels are following */
    float meas_vx, meas_vy, meas_wz;     /* measured from wheel speeds (odometry velocity) */
    float wheel_sp_rpm[CH_NUM_WHEELS];   /* wheel setpoints, motor sign, output-shaft rpm */
    float wheel_rpm[CH_NUM_WHEELS];      /* measured wheel speeds, same convention */
} chassis_state_t;

void chassis_init(void);                                /* after m3508_init_all(), before the scheduler */
void chassis_update(const cmd_vel_t *cmd, float dt_s);  /* processTask only, every cycle */
void chassis_get_state(chassis_state_t *out);           /* any task */

#endif /* CHASSIS_H */
