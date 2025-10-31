#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Inicializa Wi-Fi em modo STA usando WIFI_SSID / WIFI_PASS (definidos via menuconfig ou macros)
void wifi_init(void);

// Inicia o cliente MQTT e registra callbacks
void mqtt_start(const char *broker_uri);

// Encerra cliente MQTT
void mqtt_stop(void);

void mqtt_publish_ultrassonics(void);

#define MQTT_TOPIC_CMD "cmd/manipulator"   

#define WIFI_SSID "atenaopen2023"
#define WIFI_PASS "rrrmmmaaa"

#define MQTT_TOPIC_US  "state/ultrassonics" 

#ifdef __cplusplus
}
#endif
