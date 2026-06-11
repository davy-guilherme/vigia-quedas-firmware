#include <stdio.h>
#include <stdint.h>
#include <math.h>             // sqrt
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "bmi160.h"
#include "esp_timer.h"       // tempo em microssegundos

#define LED GPIO_NUM_25
#define BTN GPIO_NUM_32

// Ajuste de sensibilidade (Tweak aqui para detectar mais ou menos)
#define FALL_THRESHOLD_LOW   0.4f   // Queda livre (mais baixo = mais sensível)
// #define FALL_THRESHOLD_HIGH  2.5f   // Impacto (mais baixo = detecta qualquer esbarrão)
#define FALL_THRESHOLD_HIGH  1.5f
// #define STABLE_THRESHOLD     0.5f   // Diferença máxima para considerar "parado"
#define SAMPLE_PERIOD_MS     20     // 50Hz de amostragem

#define ACC_SENS 16384.0    // 1g no modo ±2g
#define GYRO_SENS 131.0     // 1°/s no modo ±250°/s

typedef enum {
    STATE_MONITORING,
    STATE_FREE_FALL,
    STATE_IMPACT_DETECTED,
    STATE_POST_IMPACT,
    STATE_FALL_CONFIRMED
} fall_state_t;

void configure_led () {
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
}

void configure_btn () {
    gpio_reset_pin(BTN);
    gpio_set_direction(BTN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN, GPIO_PULLUP_ONLY);
}

void app_main(void) {

    bmi160_config_t bmi_config = {
        .i2c_port = I2C_NUM_0,
        .sda = GPIO_NUM_19,
        .scl = GPIO_NUM_22,
        .clk_speed = 100000,
        .address = 0x69
    };

    bmi160_init(&bmi_config);

    configure_led();
    configure_btn();

    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    float ax, ay, az, gx, gy, gz;
    float acc_mag;

    int64_t freefall_start_time = 0;
    int64_t impact_time = 0;

    fall_state_t state = STATE_MONITORING;
    gpio_set_level(LED, 0);

    while (1) {

        // 1. Leitura rápida
        bmi160_read_accel(&bmi_config, &ax_raw, &ay_raw, &az_raw);
        // bmi160_read_gyro(&bmi_config, &gx_raw, &gy_raw, &gz_raw);
        // ler por interrupção e tirar uma média das últimas leituras

        // Conversão
        ax = ax_raw / ACC_SENS; ay = ay_raw / ACC_SENS; az = az_raw / ACC_SENS;
        gx = gx_raw / GYRO_SENS; gy = gy_raw / GYRO_SENS; gz = gz_raw / GYRO_SENS;

        // calcula a magnitude total da aceleração
        // usa formula da distância vetorial
        acc_mag = sqrtf(ax*ax + ay*ay + az*az);
        float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);

        switch(state) {
            case STATE_MONITORING:
                if (acc_mag < FALL_THRESHOLD_LOW) {
                    printf("FREE FALL DETECTED\n");
                    freefall_start_time = esp_timer_get_time();
                    state = STATE_FREE_FALL;
                }
                break;

            case STATE_FREE_FALL: {
                    // impacto detectado
                    if (acc_mag > FALL_THRESHOLD_HIGH) {
                        printf("IMPACT DETECTED\n");
                        impact_time = esp_timer_get_time();
                        state = STATE_IMPACT_DETECTED;
                    } else if ( esp_timer_get_time() - freefall_start_time > 500000 ) {
                        // timeout
                        printf("FREE FALL TIMEOUT\n");
                        state = STATE_MONITORING;
                    }
                    break;
                }

                case STATE_IMPACT_DETECTED: {
                    if ( esp_timer_get_time() - impact_time > 1000000) {
                        state = STATE_POST_IMPACT;
                    }
                    break;
                }

                case STATE_POST_IMPACT: {
                    bmi160_read_gyro(&bmi_config, &gx_raw, &gy_raw, &gz_raw);

                    gx = gx_raw / GYRO_SENS;
                    gy = gy_raw / GYRO_SENS;
                    gz = gz_raw / GYRO_SENS;

                    float final_gyro = sqrtf(gx*gx + gy*gy + gz*gz);

                    if (
                        acc_mag > 0.75f &&
                        acc_mag < 1.25f &&
                        final_gyro < 10.0f
                    ) {
                        state = STATE_FALL_CONFIRMED;
                    } else {
                        printf("FALSE POSITIVE\n");
                        state = STATE_MONITORING;
                    }
                    break;
                }

                case STATE_FALL_CONFIRMED: {
                    printf("FALL CONFIRMED\n");
                    gpio_set_level(LED, 1);
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    gpio_set_level(LED, 0);
                    state = STATE_MONITORING;
                    break;
                }
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS)); // 1000ms / 20ms = 50Hz -> 50 leituras por segundo
    }

}