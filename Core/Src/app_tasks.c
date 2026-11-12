#include "app_tasks.h"
#include "cmsis_os2.h"
#include "m3508.h"
#include "chassis.h"
#include "cmd_link.h"

/* Stack sizes are in bytes. Priorities: output > process > input (see app_tasks.h). */
static const osThreadAttr_t s_input_attr = {
    .name = "inputTask",   .stack_size = 1024, .priority = osPriorityHigh,
};
static const osThreadAttr_t s_process_attr = {
    .name = "processTask", .stack_size = 1024, .priority = osPriorityRealtime,
};
static const osThreadAttr_t s_output_attr = {
    .name = "outputTask",  .stack_size = 512,  .priority = osPriorityRealtime1,
};

void app_tasks_create(void)
{
    /* Module state is set up before the scheduler starts, so every task sees it initialised. */
    cmd_link_init();
    m3508_init_all();
    chassis_init();

    (void)osThreadNew(outputTask,  NULL, &s_output_attr);
    (void)osThreadNew(processTask, NULL, &s_process_attr);
    (void)osThreadNew(inputTask,   NULL, &s_input_attr);
}
