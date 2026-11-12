#include "pid.h"
#include "util.h"

static inline float integ_limit(const pid_ctrl_t *p)
{
    return (p->i_max < p->out_max) ? p->i_max : p->out_max;
}

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float out_max, float i_max)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->out_max = out_max;
    p->i_max   = i_max;
    pid_reset(p);
}

void pid_set_gains(pid_ctrl_t *p, float kp, float ki, float kd)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
}

void pid_reset(pid_ctrl_t *p)
{
    p->integ     = 0.0f;
    p->prev_meas = 0.0f;
    p->out       = 0.0f;
    p->first_run = true;
}

void pid_seed(pid_ctrl_t *p, float output, float measurement)
{
    p->integ     = clamp_absf(output, integ_limit(p));
    p->prev_meas = measurement;
    p->out       = p->integ;
    p->first_run = false;
}

float pid_update(pid_ctrl_t *p, float setpoint, float measurement, float dt)
{
    const float err = setpoint - measurement;

    /* Proportional */
    const float p_term = p->kp * err;

    /* Derivative on measurement: avoids spikes when the setpoint steps */
    float d_term = 0.0f;
    if (!p->first_run && dt > 0.0f && p->kd != 0.0f) {
        d_term = -p->kd * (measurement - p->prev_meas) / dt;
    }
    p->prev_meas = measurement;
    p->first_run = false;

    /* out_max can be lowered at run time (e.g. brake current limit): keep the integrator inside it */
    const float i_lim = integ_limit(p);
    p->integ = clamp_absf(p->integ, i_lim);

    /* Integral with conditional anti-windup:
       only integrate if output is not saturated, or if the error would pull it out of saturation */
    const float unsat = p_term + p->integ + d_term;
    const bool  saturated = (unsat > p->out_max) || (unsat < -p->out_max);
    if (!saturated || (err * unsat < 0.0f)) {
        p->integ = clamp_absf(p->integ + p->ki * err * dt, i_lim);
    }

    p->out = clamp_absf(p_term + p->integ + d_term, p->out_max);
    return p->out;
}
