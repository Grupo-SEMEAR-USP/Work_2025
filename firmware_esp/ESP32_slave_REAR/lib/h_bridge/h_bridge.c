#include "h_bridge.h"

const char *TAG_PWM = "PID";

void init_gpio()
{
    gpio_set_direction(INPUT_LEFT_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(INPUT_LEFT_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);

    gpio_set_direction(INPUT_RIGHT_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(INPUT_RIGHT_2, GPIO_MODE_OUTPUT);

    gpio_set_direction(LEDC_OUTPUT_RIGHT, GPIO_MODE_OUTPUT);
    gpio_set_direction(LEDC_OUTPUT_LEFT, GPIO_MODE_OUTPUT);

    gpio_set_level(GPIO_NUM_33, HIGH);
}

void init_pwm()
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_left_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_LEFT,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_LEFT,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0,
    };

    ledc_channel_config(&ledc_left_channel);

    ledc_channel_config_t ledc_right_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_RIGHT,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_RIGHT,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };

    ledc_channel_config(&ledc_right_channel);
}

esp_err_t update_motor(motor_side_t motor, int u)
{

    // ESP_LOGI(TAG_PWM, "PWM: %d", u);

    u > 0 ? _set_forward(motor) : _set_backward(motor);
    
    u = u > 0 ? u : -u;  // Abs of action control u

    ledc_set_duty(LEDC_MODE, MOTOR_CHANNEL(motor), u);
    ledc_update_duty(LEDC_MODE, MOTOR_CHANNEL(motor));

    return ESP_OK;
}

esp_err_t _set_forward(motor_side_t motor)
{
    gpio_set_level(MOTOR_INPUT_1(motor), HIGH);
    gpio_set_level(MOTOR_INPUT_2(motor), LOW);

    return ESP_OK;
}

esp_err_t _set_backward(motor_side_t motor)
{
    gpio_set_level(MOTOR_INPUT_1(motor), LOW);
    gpio_set_level(MOTOR_INPUT_2(motor), HIGH);

    return ESP_OK;
}