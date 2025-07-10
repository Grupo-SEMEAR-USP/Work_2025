#include "sincronization.h"
#include "esp_log.h"

static const char *TAG = "sync";

// I2C access mutex
static SemaphoreHandle_t i2c_mutex = NULL;

// Target value mutex and data
static SemaphoreHandle_t target_mutex = NULL;
static float target_left = 0;
static float target_right = 0;

// Initializes all semaphores
void sync_init(void) {
    if (i2c_mutex == NULL) {
        i2c_mutex = xSemaphoreCreateMutex();
        if (!i2c_mutex) {
            ESP_LOGE(TAG, "Failed to create I2C mutex");
        }
    }

    if (target_mutex == NULL) {
        target_mutex = xSemaphoreCreateMutex();
        if (!target_mutex) {
            ESP_LOGE(TAG, "Failed to create target mutex");
        }
    }
}

// Locks the I2C mutex
bool sync_i2c_lock(TickType_t timeout) {
    if (!i2c_mutex) return false;
    return xSemaphoreTake(i2c_mutex, timeout) == pdTRUE;
}

// Unlocks the I2C mutex
void sync_i2c_unlock(void) {
    if (i2c_mutex) {
        xSemaphoreGive(i2c_mutex);
    }
}

// Updates target values (thread-safe)
void sync_set_target(float left, float right) {
    if (xSemaphoreTake(target_mutex, pdMS_TO_TICKS(10))) {
        target_left = left;
        target_right = right;
        xSemaphoreGive(target_mutex);
    } else {
        ESP_LOGW(TAG, "Target write failed");
    }
}

// Reads target values (thread-safe)
void sync_get_target(float *left, float *right) {
    if (xSemaphoreTake(target_mutex, pdMS_TO_TICKS(10))) {
        if (left) *left = target_left;
        if (right) *right = target_right;
        xSemaphoreGive(target_mutex);
    } else {
        ESP_LOGW(TAG, "Target read failed");
        if (left) *left = 0;
        if (right) *right = 0;
    }
}
