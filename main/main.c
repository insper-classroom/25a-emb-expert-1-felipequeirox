#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#define PAN_GPIO   15  // Servo lateral
#define TILT_GPIO  16  // Servo vertical

#define ADC_X      0
#define ADC_Y      1
#define ADC_LDR    2

void setup_servo_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.f);
    pwm_config_set_wrap(&config, 39062);  // 50Hz

    pwm_init(slice, &config, true);
}

uint16_t adc_to_pwm_us(uint16_t adc_val) {
    return 1000 + (adc_val * 2000) / 4095;
}

void update_servo(uint gpio, uint16_t us) {
    uint level = (us * 39062) / 20000;
    pwm_set_gpio_level(gpio, level);
}

void control_task(void *p) {
    adc_init();
    adc_gpio_init(26); // ADC0 - X
    adc_gpio_init(27); // ADC1 - Y
    adc_gpio_init(28); // ADC2 - LDR

    setup_servo_pwm(PAN_GPIO);
    setup_servo_pwm(TILT_GPIO);

    while (true) {
        adc_select_input(ADC_X);
        uint16_t x = adc_read();

        adc_select_input(ADC_Y);
        uint16_t y = adc_read();

        adc_select_input(ADC_LDR);
        uint16_t ldr = adc_read();

        // PAN: só o eixo X (lateral)
        uint16_t pan_us = adc_to_pwm_us(x);

        // TILT: mistura de joystick Y com LDR (luz)
        uint16_t tilt_mix = ((y * 7) + (ldr * 3)) / 10;
        uint16_t tilt_us = adc_to_pwm_us(tilt_mix);

        update_servo(PAN_GPIO, pan_us);
        update_servo(TILT_GPIO, tilt_us);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

struct led_task_arg {
    int gpio;
    int delay;
};

void led_task(void *p) {
    struct led_task_arg *a = (struct led_task_arg *)p;
    gpio_init(a->gpio);
    gpio_set_dir(a->gpio, GPIO_OUT);
    while (true) {
        gpio_put(a->gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(a->delay));
        gpio_put(a->gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(a->delay));
    }
}

int main() {
    stdio_uart_init();

    struct led_task_arg arg1 = {20, 100};
    xTaskCreate(led_task, "LED_Task 1", 256, &arg1, 1, NULL);
    xTaskCreate(control_task, "PanTilt_Control", 768, NULL, 2, NULL);

    vTaskStartScheduler();

    while (true);
}
