
#ifndef __TASK_H__
#define __TASK_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stddef.h>

#define SCH_NULL_PTR NULL
#define SCH_MAX_TASKS 8

// User-defined type to store required data for each task
typedef struct
{
    // Pointer to the task
    // (must be a 'uint32_t (void)' function)
    uint32_t (*pTask)(void);
    //  void (*pTask) (void);

    // Delay (ticks) until the task will (next) be run
    uint32_t Delay;

    // Interval (ticks) between subsequent runs.
    uint32_t Period;
} sTask_t;

extern sTask_t SCH_tasks_g[];
extern uint32_t Current_Task_id;
extern uint32_t Tick_count_g;

extern void SCH_Add_Task(uint32_t (*pTask)(), const uint32_t DELAY, const uint32_t PERIOD);
extern void SCH_delete_Task(uint32_t (*pTask)());
extern void SCH_change_Task(uint32_t (*pTask)(), const uint32_t DELAY, const uint32_t PERIOD);
extern void SCH_Dispatch_Tasks(void);
extern void TIMX_IRQHandler_user(void);

#endif
