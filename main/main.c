#include <stdio.h>
#include <stdint.h>
#include <math.h>             // sqrt
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "bmi160.h"
#include "esp_timer.h"       // tempo em microssegundos

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h" // ESP_LOGI(), ESP_LOGE(), ESP_LOGW()
#include "nvs_flash.h" // memoria flash usada para armezenar dados do Wi-Fi, calibração RF, configurações persistentes
#include "mqtt_client.h" // cliente MQTT, conexao com broker, pub, sub, callbacks MQTT

#include <time.h>
#include <sys/time.h>

// #define WIFI_SSID "2GFelicidade"
// #define WIFI_PASS "MiraMaria"
#define WIFI_SSID "DGiPhone"
#define WIFI_PASS "usaasuarede"

#define LED GPIO_NUM_25
#define BTN GPIO_NUM_32

static const char *TAG = "Vigia";

// Ajuste de sensibilidade (Tweak aqui para detectar mais ou menos)
#define FALL_THRESHOLD_LOW   0.4f   // Queda livre (mais baixo = mais sensível)
// #define FALL_THRESHOLD_HIGH  2.5f   // Impacto (mais baixo = detecta qualquer esbarrão)
#define FALL_THRESHOLD_HIGH  1.5f
// #define STABLE_THRESHOLD     0.5f   // Diferença máxima para considerar "parado"
#define SAMPLE_PERIOD_MS     20     // 50Hz de amostragem

#define ACC_SENS 16384.0    // 1g no modo ±2g
#define GYRO_SENS 131.0     // 1°/s no modo ±250°/s

static bool wifi_connected = false;
static bool mqtt_connected = false;

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

static esp_mqtt_client_handle_t mqtt_client;

static void mqtt_event_handler (
    void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data
) {
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT conectado");
            printf("MQTT conectado - printf");
            esp_mqtt_client_publish(
                event->client,
                "teste/esp32",
                "ESP32 conectado via MQTT",
                0,
                1,
                0
            );

            break;
        
        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGW(TAG, "MQTT DESCONECTADO");
            break;

        case MQTT_EVENT_ERROR:
            mqtt_connected = false; 
            ESP_LOGE(TAG, "MQTT ERROR");
            break;

        // outros eventos mqtt
        default:
            break;
    }
}

const char *wifi_reason_to_string(uint8_t reason) {
    switch (reason) {
        case WIFI_REASON_NO_AP_FOUND:
            return "AP nao encontrado";

        case WIFI_REASON_AUTH_FAIL:
            return "Falha de autenticacao";

        case WIFI_REASON_ASSOC_FAIL:
            return "Falha de associacao";

        case WIFI_REASON_HANDSHAKE_TIMEOUT:
            return "Timeout no handshake";

        case WIFI_REASON_BEACON_TIMEOUT:
            return "Timeout de beacon";

        default:
            return "Motivo desconhecido";
    }
}

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base, // base do evento. ex: WIFI_EVENT, IP_EVENT
    int32_t event_id, // identificador do evento. ex: WIFI_EVENT_STA_START, WIFI_EVENT_STA_DISCONNECTED, IP_EVENT_STA_GOT_IP
    void *event_data // dado do evento. alguns eventos enviam informações extras. ex: evento de IP envia o endereço IP recebido
) {
    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "Wi-Fi iniciado");
        esp_wifi_connect(); // Esta função: inicia autenticação, tenta associar no AP, inicia DHCP
    }

    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_DISCONNECTED)
    {   
        wifi_connected = false;
        mqtt_connected = false;
        wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *) event_data;
        ESP_LOGW(
            TAG,
            "Wi-Fi desconectado. Motivo: %s (%d)",
            wifi_reason_to_string(event->reason),
            event->reason
        );
        
        esp_wifi_connect();
    }

    if (event_base == IP_EVENT &&
        event_id == IP_EVENT_STA_GOT_IP)
    {
        wifi_connected = true;
        ESP_LOGI(TAG, "Conectado no Wi-Fi");
        printf("Conectado no wi-fi\n");

        ip_event_got_ip_t *event =
            (ip_event_got_ip_t *) event_data;

        ESP_LOGI(
            TAG,
            "IP: " IPSTR,
            IP2STR(&event->ip_info.ip)
        );

        esp_mqtt_client_start(mqtt_client);
    }
}

void app_main(void) {

    // WIFI E MQTT

    printf(">>> Iniciando 1\n");

    nvs_flash_init(); // inicalizar mamória NVS, necessaria pra o wi-fi funcionar
    esp_netif_init(); // inicalizar o satack de rede: TCP/IP, interfaces de rede, gerenciamento de IP, DHCP
    esp_event_loop_create_default(); // cria loop global de eventos - todos os eventos passam por ele
    esp_netif_create_default_wifi_sta(); // Cria interface Wi-Fi STA

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // preenche a estrutura com valore padrões

    esp_wifi_init(&cfg); // inicializa o hardware e prepara o diver, mas ainda não conecta

    // Registra função de eventos Wi-Fi
    esp_event_handler_register(
        WIFI_EVENT, // base de eventos wi-fi
        ESP_EVENT_ANY_ID, // todos os eventos Wi-FI
        &wifi_event_handler, // função de callback
        NULL // argumento opcional, dado personalizado
    );

    // Registra eventos do sistema IP
    esp_event_handler_register(
        IP_EVENT, // base de eventos IP
        IP_EVENT_STA_GOT_IP, // Evento: recebeu IP
        &wifi_event_handler, // função de callback
        NULL // argumento opcional, dado personalizado
    );

    wifi_config_t wifi_config = {
        // Configuração do modo Station
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        }
    };

    esp_wifi_set_mode(WIFI_MODE_STA); // Define modo de operação do Wi-Fi como station, cliente WiFi

    // Aplica configuração da rede
    esp_wifi_set_config(
        WIFI_IF_STA, // interface STA
        &wifi_config // estrutura com configurações
    );


    // cofiguração mqtt
    esp_mqtt_client_config_t mqtt_cfg = {

        // .broker.address.uri = "mqtt://broker.hivemq.com"
        .broker.address.uri = "mqtt://52.202.93.46"
    };

    // criar cliente mqtt, se se conectar no broker
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    // registra eventos MQTT que serão enviados para mqtt_event_handler()
    esp_mqtt_client_register_event(
        mqtt_client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL
    );

    esp_wifi_start(); // Inicia o Wi-Fi // Após isso: WIFI_EVENT_STA_START será disparado


    // ACELEROMETRO

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

        gpio_set_level(LED, (!wifi_connected || !mqtt_connected));

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

                    int64_t start_time = esp_timer_get_time(); // microssegundos

                    bool led_state = false;
                    bool false_alarm = false;

                    while ((esp_timer_get_time() - start_time) < 5000000 && false_alarm == false) { // 5 segundos
                        // Verifica se o botão foi pressionado
                        if (gpio_get_level(BTN) == 0) { // botão ligado ao GND
                            printf("Botão pressionado!\n");

                            gpio_set_level(LED, 0); // apaga LED
                            false_alarm = true;
                        }

                        // Alterna estado do LED
                        led_state = !led_state;
                        gpio_set_level(LED, led_state);

                        vTaskDelay(pdMS_TO_TICKS(250)); // pisca a cada 250 ms
                    }

                    if (!false_alarm) {
                        gpio_set_level(LED, 0);
                        printf("Tempo esgotado e botão não pressionado.\n");
                        char payload[256];
                        int battery = 10;

                        time_t timestamp;
                        time(&timestamp);

                        snprintf(
                            payload,
                            sizeof(payload),
                            "{"
                                "\"event\":\"fall_detected\","
                                "\"device_id\":\"esp32-001\","
                                "\"timestamp\":%lld"
                            "}",
                            (long long) timestamp
                        );

                        // gpio_set_level(LED, 1);
                        // eviar mensagem mqtt
                        esp_mqtt_client_publish(
                            mqtt_client,
                            "vigiaquedas/device/esp32-001/fall",
                            // "fall/esp32-001/alert",
                            payload,
                            0,
                            1,
                            0
                        );

                        // vTaskDelay(pdMS_TO_TICKS(5000));
                        
                    }

                    // Garante que o LED fique apagado ao final dos 5 segundos
                    gpio_set_level(LED, 0);

                    state = STATE_MONITORING;
                    break;
                }
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS)); // 1000ms / 20ms = 50Hz -> 50 leituras por segundo
    }

}