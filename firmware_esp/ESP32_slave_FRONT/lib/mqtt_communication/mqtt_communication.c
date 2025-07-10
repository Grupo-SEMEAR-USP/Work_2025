#include "mqtt_communication.h"
#include "esp_log.h"
#include <cJSON.h>

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


static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            esp_mqtt_client_subscribe(client, "comandos/motores/front", 1);
            break;

        case MQTT_EVENT_DATA:
            cJSON *root = cJSON_ParseWithLength(event->data, event->data_len);
            if (root) {
                cJSON *left = cJSON_GetObjectItem(root, "left");
                cJSON *right = cJSON_GetObjectItem(root, "right");
                if (cJSON_IsNumber(left) && cJSON_IsNumber(right)) {
                    TARGET_VALUE_L = left->valueint;
                    TARGET_VALUE_R = right->valueint;

                    TARGET_VALUE_L = TARGET_VALUE_L / 1000;
                    TARGET_VALUE_R = TARGET_VALUE_R / 1000;
                }
                cJSON_Delete(root);
            }
            break;

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

    // ESP_LOGI(TAG, "MQTT iniciado.");
}

void mqtt_publish_encoders(pcnt_unit_handle_t upcnt_unit_R, pcnt_unit_handle_t upcnt_unit_L)
{
    static int64_t last_time_us = 0;

    int64_t now_us = esp_timer_get_time();

    float left_value = 0;
    float right_value = 0;

    int64_t delta_us = 0;
    if (last_time_us != 0) {
        delta_us = (now_us - last_time_us);
    }
    last_time_us = now_us;

    ENCODER_READ_L = pulse_count(upcnt_unit_L);
    ENCODER_READ_R = pulse_count(upcnt_unit_R);

    RADS_L = (ENCODER_READ_L * 2 * PI / (ENCODER_RESOLUTION * delta_us)) * 1000000;
    RADS_R = (ENCODER_READ_R * 2 * PI / (ENCODER_RESOLUTION * delta_us)) * 1000000;

    if (client) {
        int32_t left = RADS_L * 1000;
        int32_t right = RADS_R * 1000;

        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "left", left);
        cJSON_AddNumberToObject(root, "right", right);

        char *payload = cJSON_PrintUnformatted(root);

        esp_mqtt_client_publish(client, "estado/encoders/front", payload, 0, 1, 0);

        free(payload);
        cJSON_Delete(root);
    }
}

void mqtt_stop(void)
{
    if (client) {
        esp_mqtt_client_stop(client);
        esp_mqtt_client_destroy(client);
        client = NULL;
    }
}