#include "stepper_control.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h" 
#include "esp_log.h"
#include <math.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "stepper";

static int   GP_STEP_E, GP_DIR_E, GP_STEP_B, GP_DIR_B;
static float MAX_SPS, ACC_SPS2;

static _Atomic int s_target_e = 0;
static _Atomic int s_target_b = 0;
static _Atomic int s_remain_e = 0;
static _Atomic int s_remain_b = 0;

static inline void pulse_once(int gpio_step, int gpio_dir, int dir)
{
    gpio_set_level(gpio_dir, dir > 0 ? 1 : 0);
    gpio_set_level(gpio_step, 1);
    esp_rom_delay_us(3);
    gpio_set_level(gpio_step, 0);
}

static void move_blocking(_Atomic int *remain_ptr, int gpio_step, int gpio_dir)
{
    int remain = atomic_exchange(remain_ptr, 0);
    if (remain == 0) return;

    int dir   = (remain > 0) ? 1 : 0;
    int steps = abs(remain);

    float sps = MAX_SPS;              
    if (sps < 1.0f) sps = 1.0f;

    const uint32_t pulse_high_us = 3;   
    uint32_t period_us = (uint32_t)(1000000.0f / sps);
    if (period_us <= pulse_high_us) period_us = pulse_high_us + 1;

    uint32_t low_phase_us = period_us - pulse_high_us;

    gpio_set_level(gpio_dir, dir);

    for (int i = 0; i < steps; ++i) {
        gpio_set_level(gpio_step, 1);
        esp_rom_delay_us(pulse_high_us);
        gpio_set_level(gpio_step, 0);

        if (low_phase_us >= 2000) {
            vTaskDelay(pdMS_TO_TICKS(low_phase_us / 1000));
            uint32_t rem = low_phase_us % 1000;
            if (rem) esp_rom_delay_us(rem);
        } else {
            esp_rom_delay_us(low_phase_us);
        }

        if ((i & 0xFF) == 0) taskYIELD();
    }
}

esp_err_t stepper_init()
{
    GP_STEP_E = GPIO_STEP_ELEVATOR;
    GP_DIR_E  = GPIO_DIR_ELEVATOR;
    GP_STEP_B = GPIO_STEP_BASE;
    GP_DIR_B  = GPIO_DIR_BASE;
    MAX_SPS   = MAX_SPEED;
    ACC_SPS2  = ACCEL;

    int gpios[] = {GP_STEP_E, GP_DIR_E, GP_STEP_B, GP_DIR_B};
    for (int i = 0; i < 4; ++i){
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << gpios[i],
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        esp_err_t err = gpio_config(&io);
        if (err != ESP_OK) return err;
        gpio_set_level(gpios[i], 0);
    }

    atomic_store(&s_target_e, 0);
    atomic_store(&s_target_b, 0);
    atomic_store(&s_remain_e, 0);
    atomic_store(&s_remain_b, 0);

    return ESP_OK;
}

void stepper_elevator_move(int steps)
{
    atomic_fetch_add(&s_target_e, steps);
    atomic_fetch_add(&s_remain_e, steps);
}

void stepper_base_move(int steps)
{
    atomic_fetch_add(&s_target_b, steps);
    atomic_fetch_add(&s_remain_b, steps);
}

void stepper_task_loop(void)
{
    if (atomic_load(&s_remain_e) != 0) {
        move_blocking(&s_remain_e, GP_STEP_E, GP_DIR_E);
    }
    if (atomic_load(&s_remain_b) != 0) {
        move_blocking(&s_remain_b, GP_STEP_B, GP_DIR_B);
    }
}

bool stepper_is_idle(void)
{
    return (atomic_load(&s_remain_e) == 0 &&
            atomic_load(&s_remain_b) == 0);
}
