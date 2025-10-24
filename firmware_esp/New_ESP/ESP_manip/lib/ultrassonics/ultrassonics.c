#include "ultrassonics.h"

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "ultrasonic.h"  
#include "utils.h"       

_Static_assert(US_NUM_SENSORS <= 3, "US_NUM_SENSORS deve ser <= 3 (tamanho de G_US_CM[])");

// Transforma as macros de pinos em arrays estáticos
static const int s_trig_pins[US_NUM_SENSORS] = US_TRIG_PINS;
static const int s_echo_pins[US_NUM_SENSORS] = US_ECHO_PINS;

// Vetor de sensores usados
static ultrasonic_sensor_t s_sensors[US_NUM_SENSORS];
static int s_ns = 0;
static bool s_ready = false;

static void log_err(esp_err_t res, int idx)
{
    printf("[US%d] ", idx);
    switch (res) {
        case ESP_ERR_ULTRASONIC_PING:
            printf("Erro: ping inválido (device em estado incorreto)\n"); break;
        case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
            printf("Erro: ping timeout (sem resposta do device)\n"); break;
        case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
            printf("Erro: echo timeout (distância muito grande)\n"); break;
        default:
            printf("Erro %d: %s\n", res, esp_err_to_name(res)); break;
    }
}

esp_err_t ultrassonic_init(void)
{
    s_ns = 0;

    for (int i = 0; i < US_NUM_SENSORS; ++i) {
        const int tp = s_trig_pins[i];
        const int ep = s_echo_pins[i];

        if (tp < 0 || ep < 0) {
            continue;
        }

        ultrasonic_sensor_t sensor = {
            .trigger_pin = tp,
            .echo_pin    = ep
        };
        ultrasonic_init(&sensor);

        s_sensors[s_ns++] = sensor;
    }

    if (s_ns == 0) {
        s_ready = false;
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < US_NUM_SENSORS; ++i) G_US_CM[i] = NAN;
    G_US_TS_MS = 0ULL;

    s_ready = true;
    return ESP_OK;
}

void ultrassonic_sample_once(void)
{
    if (!s_ready) {
        return;
    }

    for (int i = 0; i < s_ns; ++i) {
        float dist_m = NAN;
        esp_err_t res = ultrasonic_measure(&s_sensors[i], US_MAX_DISTANCE_M, &dist_m);

        if (res != ESP_OK) {
            // log_err(res, i);
            G_US_CM[i] = 0.0f; 
        } else {
            float dist_cm = dist_m * 100.0f;
            G_US_CM[i] = dist_cm;
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }

    G_US_TS_MS = (uint64_t)(esp_timer_get_time() / 1000ULL);
}

float ultrassonic_read_cm(int idx)
{
    if (idx < 0 || idx >= s_ns) return 0.0f;
    return G_US_CM[idx];
}
