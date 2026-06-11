#ifndef BMI160_H
#define BMI160_h

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

typedef struct {
    i2c_port_t i2c_port;
    gpio_num_t sda;
    gpio_num_t scl;
    uint32_t clk_speed;
    uint8_t address;
} bmi160_config_t;

// Inicialização
esp_err_t bmi160_init(bmi160_config_t *bmi_conf);

// Leitura
esp_err_t bmi160_read_accel(bmi160_config_t *bmi_conf, int16_t *ax, int16_t *ay, int16_t *az);
esp_err_t bmi160_read_gyro(bmi160_config_t *bmi_conf, int16_t *gx, int16_t *gy, int16_t *gz);

#endif