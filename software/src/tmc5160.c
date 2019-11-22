#include "tmc5160.h"

#include <stdbool.h>
#include <stdint.h>

#include "configs/config_tmc5160.h"

#include "bricklib2/hal/system_timer/system_timer.h"
#include "bricklib2/os/coop_task.h"
#include "bricklib2/logging/logging.h"

TMC5160 tmc5160;
CoopTask tmc5160_task;

void tmc5160_tick_task(void) {
    while(true) {
        coop_task_sleep_ms(1000);
        logd("tick\n\r");
    }
}

void tmc5160_init(void) {
    coop_task_init(&tmc5160_task, tmc5160_tick_task);
}

void tmc5160_tick(void) {
    coop_task_tick(&tmc5160_task);
}