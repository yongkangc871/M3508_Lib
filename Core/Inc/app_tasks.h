/**
 * app_tasks.h - the three application tasks.
 *
 *   inputTask    (input_task.c)    Jetson UART bytes  ->  velocity command            event-driven
 *   processTask  (process_task.c)  command + feedback ->  four motor currents         1 kHz
 *   outputTask   (output_task.c)   four motor currents -> one CAN frame               triggered by processTask
 *
 * Data flows one way: input -> process -> output. Each task owns its stage and hands over through
 * a small API (cmd_link_get, outputTask_trigger), never by sharing variables directly.
 *
 * Priorities: outputTask > processTask > inputTask > everything else.
 * outputTask is highest so the CAN frame leaves the instant processTask finishes computing it.
 */
#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdint.h>
#include <stdbool.h>

/** Timing health of the control loop. Read it from a low-priority task or the debugger. */
typedef struct {
    bool     can_ok;          /* CAN filter / start succeeded                              */
    uint32_t loops;           /* control cycles executed                                   */
    uint32_t overruns;        /* cycles that ran past their 1 ms slot                      */
    uint32_t exec_us_last;    /* last cycle: compute + CAN send, microseconds              */
    uint32_t exec_us_max;     /* worst cycle since boot                                    */
} process_stats_t;

/** Initialise the modules and create the three tasks. Call from MX_FREERTOS_Init(). */
void app_tasks_create(void);

void processTask_get_stats(process_stats_t *out);
uint32_t outputTask_frame_count(void);       /* CAN command frames handed to the bus */

/* ---- used between the task files only ---- */
void inputTask(void *arg);
void processTask(void *arg);
void outputTask(void *arg);
void outputTask_trigger(void);               /* processTask -> outputTask: "currents are ready, send them" */

#endif /* APP_TASKS_H */
