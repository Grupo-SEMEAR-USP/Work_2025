#include <stdio.h>
#include "i2c_communication.h"
#include "mqtt_communication.h"
#include "pid.h"
#include "h_bridge.h"
#include "sincronization.h"
#include "task_manager.h"
#include <unistd.h>

void app_main(void) {

    // sync_init();
    init_tasks();
}