#include "tcm5160.h"

#include <stdbool.h>
#include <stdint.h>

#include "bricklib2/hal/system_timer/system_timer.h"
#include "bricklib2/os/coop_task.h"
#include "bricklib2/logging/logging.h"

TCM5160 tcm5160;
CoopTask tcm5160_task;

void tcm5160_tick_task(void) {
    while(true) {
        coop_task_sleep_ms(1000);
        logd("tick\n\r");
    }
}

void tcm5160_init(void) {
    coop_task_init(&tcm5160_task, tcm5160_tick_task);
}

void tcm5160_tick(void) {
    coop_task_tick(&tcm5160_task);
}