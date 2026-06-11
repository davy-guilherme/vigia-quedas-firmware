#include "bmi160.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "driver/i2c.h"
#include "esp_err.h"

// ------ Funções Internas - Comunicação

static esp_err_t bmi160_write(bmi160_config_t *bmi_config, uint8_t reg, uint8_t data) {

    uint8_t buf[2] = {reg, data}; // buffer: registrador + dado

    return i2c_master_write_to_device(
        bmi_config->i2c_port,
        bmi_config->address,
        buf,
        2,
        pdMS_TO_TICKS(100)
    );
}

// lê vários bytes a partir de um registrador
static esp_err_t bmi160_read(bmi160_config_t *bmi_config, uint8_t reg, uint8_t *data, size_t len) {

    return i2c_master_write_read_device(
        bmi_config->i2c_port,
        bmi_config->address,
        &reg,
        1,
        data,
        len,
        pdMS_TO_TICKS(100)
    );
}

// ------ Inicialização
esp_err_t bmi160_init(bmi160_config_t *bmi_conf) {
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = bmi_conf->sda,
        .scl_io_num = bmi_conf->scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = bmi_conf->clk_speed
    };

    i2c_param_config(bmi_conf->i2c_port, &i2c_conf);
    i2c_driver_install(bmi_conf->i2c_port, i2c_conf.mode, 0, 0, 0);  // DESCOBRIR DIFERENCA -> .

    vTaskDelay(pdMS_TO_TICKS(200));

    bmi160_write(bmi_conf, 0x7E, 0xB6);                 // reset
    vTaskDelay(pdMS_TO_TICKS(200));

    bmi160_write(bmi_conf, 0x7E, 0x11);                 // ativa acelerômetro
    vTaskDelay(pdMS_TO_TICKS(200));

    bmi160_write(bmi_conf, 0x7E, 0x15);                 // ativa giroscópio
    vTaskDelay(pdMS_TO_TICKS(200));

    return ESP_OK;
}

// ------ Leituras
esp_err_t bmi160_read_accel(bmi160_config_t *cfg, int16_t *ax, int16_t *ay, int16_t *az) {

    uint8_t data[6];

    bmi160_read(cfg, 0x12, data, 6);

    *ax = (int16_t)(data[1] << 8 | data[0]);
    *ay = (int16_t)(data[3] << 8 | data[2]);
    *az = (int16_t)(data[5] << 8 | data[4]);

    return ESP_OK;
}

esp_err_t bmi160_read_gyro(bmi160_config_t *cfg, int16_t *gx, int16_t *gy, int16_t *gz) {

    uint8_t data[6];

    bmi160_read(cfg, 0x0C, data, 6);

    *gx = (int16_t)(data[1] << 8 | data[0]);
    *gy = (int16_t)(data[3] << 8 | data[2]);
    *gz = (int16_t)(data[5] << 8 | data[4]);

    return ESP_OK;
}

