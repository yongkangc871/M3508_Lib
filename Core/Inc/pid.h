/**
 * pid.h - small, allocation-free float PID for Cortex-M4F.
 * Derivative on measurement (no setpoint kick), clamped integrator, conditional anti-windup.
 *
 * The type is called pid_ctrl_t (not pid_t) because newlib's <sys/types.h> already defines pid_t.
 */
#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    /* gains (ki and kd are per second: dt is passed to pid_update) */
    float kp, ki, kd;
    /* limits */
    float out_max;      /* |output| <= out_max */
    float i_max;        /* |integral term| <= min(i_max, out_max) */
    /* state */
    float integ;
    float prev_meas;
    float out;
    bool  first_run;
} pid_ctrl_t;

void  pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float out_max, float i_max);
void  pid_set_gains(pid_ctrl_t *p, float kp, float ki, float kd);
void  pid_reset(pid_ctrl_t *p);
/** Bumpless start: preload the integrator so the first output equals `output` when the error is zero. */
void  pid_seed(pid_ctrl_t *p, float output, float measurement);
/** dt in seconds. Returns the clamped output (also stored in p->out). */
float pid_update(pid_ctrl_t *p, float setpoint, float measurement, float dt);

#endif /* PID_H */
