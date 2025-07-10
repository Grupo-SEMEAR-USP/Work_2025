#ifndef MQTT_COMMUNICATION_H
#define MQTT_COMMUNICATION_H

#include "mqtt_client.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "utils.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "mqtt_communication.h"
#include "pid_ctrl.h"
#include "encoder.h"
#include "h_bridge.h"
#include "utils.h"
#include "math.h"

#define WIFI_SSID "atenaopen2023"
#define WIFI_PASS "rrrmmmaaa"
#define BROKER_URI "mqtt://192.168.0.101"

void mqtt_start(const char *broker_uri);
void mqtt_publish_encoders(pcnt_unit_handle_t upcnt_unit_R, pcnt_unit_handle_t upcnt_unit_L);
void mqtt_stop(void);
void wifi_init(void);

#endif // MQTT_COMMUNICATION_H