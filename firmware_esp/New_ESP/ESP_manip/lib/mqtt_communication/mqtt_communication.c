#include "mqtt_communication.h"
#include "utils.h"
#include "ultrassonics.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "esp_timer.h"

#include <cJSON.h>
#include <string.h>
#include <math.h>

static const char *TAG = "MQTT_COMM";

static esp_mqtt_client_handle_t client = NULL;

void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {};
    snprintf((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), WIFI_SSID);
    snprintf((char *)wifi_config.sta.password, sizeof(wifi_config.sta.password), WIFI_PASS);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

static void handle_cmd_json(const char *payload, int len)
{
    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        ESP_LOGW(TAG, "JSON invalido");
        return;
    }

    const cJSON *arm   = cJSON_GetObjectItem(root, "arm");
    const cJSON *base  = cJSON_GetObjectItem(root, "base");
    const cJSON *wrist = cJSON_GetObjectItem(root, "wrist");
    const cJSON *grip  = cJSON_GetObjectItem(root, "grip");

    if (cJSON_IsNumber(arm)) {
        STEPPER_ARM = (float)arm->valuedouble;
    }
    if (cJSON_IsNumber(base)) {
        STEPPER_ROTATORY_BASE = (float)base->valuedouble;
    }
    if (cJSON_IsNumber(wrist)) {
        float v = (float)wrist->valuedouble;
        if (v < 0.f) v = 0.f; else if (v > 180.f) v = 180.f;
        SERVO_WRIST = v;
    }
    if (cJSON_IsNumber(grip)) {
        float v = (float)grip->valuedouble;
        if (v < 0.f) v = 0.f; else if (v > 180.f) v = 180.f;
        SERVO_GRIPPER = v;
    }

    cJSON_Delete(root);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            esp_mqtt_client_subscribe(client, MQTT_TOPIC_CMD, 2);
            ESP_LOGI(TAG, "MQTT conectado e inscrito em %s", MQTT_TOPIC_CMD);
            break;

        case MQTT_EVENT_DATA: {
            const char *topic = event->topic;
            int tlen = event->topic_len;
            const char *data = event->data;
            int dlen = event->data_len;

            if (!topic || tlen <= 0 || !data || dlen <= 0) break;

            if ((int)strlen(MQTT_TOPIC_CMD) == tlen &&
                strncmp(topic, MQTT_TOPIC_CMD, tlen) == 0) {
                handle_cmd_json(data, dlen);
            }
            break;
        }

        default:
            break;
    }
}

void mqtt_start(const char *broker_uri)
{
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = broker_uri; 

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

#include "mqtt_communication.h"
#include "cJSON.h"
#include "ultrassonics.h"
#include <math.h>
#include <stdio.h>

void mqtt_publish_ultrassonics(void)
{
    if (!client) return;

    float d0 = G_US_CM[0];
    float d1 = G_US_CM[1];
    float d2 = G_US_CM[2];
    // ESP_LOGI("US:","d0: %f; d1: %f; d2: %f",d0,d1,d2);
    uint64_t ts_ms = G_US_TS_MS;

    cJSON *root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "ts_ms", (double)ts_ms);

    cJSON_AddNumberToObject(root, "front_left",  isnan(d0) ? 0.0 : d0);
    cJSON_AddNumberToObject(root, "front_right",   isnan(d1) ? 0.0 : d1);
    cJSON_AddNumberToObject(root, "rear_left",  isnan(d2) ? 0.0 : d2);

    char *payload = cJSON_PrintUnformatted(root);

    esp_mqtt_client_publish(client, MQTT_TOPIC_US, payload, 0, 2, 0);

    free(payload);
    cJSON_Delete(root);
}

void mqtt_stop(void)
{
    if (client) {
        esp_mqtt_client_stop(client);
        esp_mqtt_client_destroy(client);
        client = NULL;
    }
}