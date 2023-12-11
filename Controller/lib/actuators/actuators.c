#include "actuators.h"

esp_err_t init_gpio()
{
    gpio_set_direction(H_BRIDGE_INPUT_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(H_BRIDGE_INPUT_1, GPIO_MODE_OUTPUT);

    return ESP_OK;
}

esp_err_t init_pwm()
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };


    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    return ESP_OK;
}

esp_err_t update_motor(int u, int debug)
{   
    u > 0 ? _set_forward() : _set_backward();
    
    u = u > 0 ? u : -u;  //Abs of action control u
    
    if (debug == 1)
        ESP_LOGI(TAG, "Action control %d\n", u);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, u);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    return ESP_OK;
}

esp_err_t _set_forward()
{
    gpio_set_level(H_BRIDGE_INPUT_1, HIGH);
    gpio_set_level(H_BRIDGE_INPUT_2, LOW);

    return ESP_OK;
}

esp_err_t _set_backward()
{
    gpio_set_level(H_BRIDGE_INPUT_1, LOW);
    gpio_set_level(H_BRIDGE_INPUT_2, HIGH);

    return ESP_OK;
}
