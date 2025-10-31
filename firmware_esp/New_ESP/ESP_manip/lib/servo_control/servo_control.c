#include "servo_control.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "servo";
static int s_gpio_gripper=-1, s_gpio_wrist=-1;
static float s_last_gripper = -1.f, s_last_wrist = -1.f;

#define SERVO_MAX_ANGLE 180.0f

#define SERVO_HZ 50
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

static inline uint32_t us_to_duty(uint32_t period_us, uint32_t pulse_us, uint32_t max_duty)
{
    if (pulse_us < 500) pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;
    return (uint32_t)((((uint64_t)pulse_us) * max_duty) / period_us);
}

static esp_err_t channel_cfg(int channel, int gpio)
{
    ledc_channel_config_t ch = {
        .gpio_num   = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    return ledc_channel_config(&ch);
}

esp_err_t servo_init()
{

    ledc_timer_config_t t = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_20_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = SERVO_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    ESP_ERROR_CHECK(channel_cfg(LEDC_CHANNEL_0, GPIO_GRIPPER));
    ESP_ERROR_CHECK(channel_cfg(LEDC_CHANNEL_1, GPIO_WRIST));

    s_last_gripper = -1.f;
    s_last_wrist   = -1.f;

    return ESP_OK;
}

static esp_err_t write_angle(int channel, float angle_deg, float *last_store)
{
    if (angle_deg < 0.f) angle_deg = 0.f;
    if (angle_deg > SERVO_MAX_ANGLE) angle_deg = SERVO_MAX_ANGLE;

    float pulse = SERVO_MIN_US + (angle_deg / SERVO_MAX_ANGLE) * (SERVO_MAX_US - SERVO_MIN_US);

    uint32_t max_duty = (1<<20) - 1;
    uint32_t period_us = 1000000 / SERVO_HZ;
    uint32_t duty = us_to_duty(period_us, (uint32_t)pulse, max_duty);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));

    *last_store = angle_deg;
    return ESP_OK;
}

esp_err_t servo_write_gripper(float angle_deg) { return write_angle(LEDC_CHANNEL_0, angle_deg, &s_last_gripper); }
esp_err_t servo_write_wrist  (float angle_deg) { return write_angle(LEDC_CHANNEL_1, angle_deg, &s_last_wrist); }

float servo_last_gripper(void){ return s_last_gripper; }
float servo_last_wrist(void)  { return s_last_wrist; }
