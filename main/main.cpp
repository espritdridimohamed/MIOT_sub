extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/event_groups.h"
    #include "freertos/task.h"
    #include "freertos/timers.h"
    #include "esp_event.h"
    #include "esp_log.h"
    #include "nvs_flash.h"
    #include "esp_netif.h"
    #include "esp_wifi.h"
    #include "mqtt_client.h"
    #include "esp_netif_ip_addr.h"
    #include "driver/gpio.h"
    #include "driver/ledc.h"
    #include "wifi_provisioning/manager.h"
    #include "wifi_provisioning/scheme_softap.h"
    #include "qrcode.h"
    #include "esp_vfs.h"
    #include "esp_spiffs.h"
}

#include <cstring>
#include <cstdlib>
#include <dirent.h>  // For SPIFFS listing if needed

static volatile bool s_mqtt_connected = false;

// SPIFFS paths for certs
static const char CA_CERT_PATH[] = "/spiffs/ca.pem";
static const char CLIENT_CERT_PATH[] = "/spiffs/client.pem";
static const char CLIENT_KEY_PATH[] = "/spiffs/key.pem";

static const char *TAG = "MIOT_SUB";

// MQTT broker
static constexpr const char *BROKER_URI = "mqtts://192.168.1.17:8883";

// Topics to subscribe
static constexpr const char *SUB_TOPICS[] = {
    "miot/ir/code",
    "miot/ldr/raw",
    "miot/dht22/temperature",
    "miot/dht22/humidity"
};
static constexpr int NUM_TOPICS = sizeof(SUB_TOPICS) / sizeof(SUB_TOPICS[0]);

// Event group
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

static esp_mqtt_client_handle_t s_mqtt_client = nullptr;

// GPIO pins
static constexpr int LED1_GPIO = 15;
static constexpr int LED2_GPIO = 4;
static constexpr int LED3_GPIO = 5;
static constexpr int SERVO_GPIO = 18;
static constexpr int BUTTON_GPIO = 19;
static constexpr int BUZZER_GPIO = 21;

// LED states
static volatile bool s_led1_on = false;
static volatile bool s_led2_on = false;

// Auto mode for LDR control
static volatile bool auto_mode = true;

// High temperature flag
static volatile bool high_temp = false;

// Blink timer
static TimerHandle_t s_blink_timer = nullptr;

// Buzzer timers
static TimerHandle_t buzzer_blink_timer = nullptr;
static TimerHandle_t high_temp_beep_timer = nullptr;

// LEDC config for servo
static constexpr ledc_timer_t SERVO_LEDC_TIMER = LEDC_TIMER_0;
static constexpr ledc_channel_t SERVO_LEDC_CHANNEL = LEDC_CHANNEL_0;
static constexpr ledc_mode_t SERVO_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static constexpr uint32_t SERVO_FREQ_HZ = 50;
static constexpr ledc_timer_bit_t SERVO_RES = LEDC_TIMER_16_BIT;

// Helper to read files from SPIFFS
static char* read_file_from_spiffs(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open %s", path);
        return nullptr;
    }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    char* buffer = (char*)malloc(fsize + 1);
    if (!buffer) {
        ESP_LOGE(TAG, "Malloc failed for %s", path);
        fclose(f);
        return nullptr;
    }

    fread(buffer, 1, fsize, f);
    fclose(f);
    buffer[fsize] = 0; // Null-terminate
    return buffer;
}

// List files in SPIFFS for debugging (optional, call in init_spiffs if needed for debug)
static void list_spiffs_files(void) {
    DIR* dir = opendir("/spiffs");
    if (dir) {
        struct dirent* ent;
        while ((ent = readdir(dir)) != NULL) {
            ESP_LOGI(TAG, "SPIFFS File: %s", ent->d_name);
        }
        closedir(dir);
    } else {
        ESP_LOGE(TAG, "Failed to open /spiffs");
    }
}

// Helper to publish feedback to push/ topics
static void mqtt_publish_push(const char* subtopic, const char* payload) {
    if (!s_mqtt_client || !s_mqtt_connected) return;

    char topic[32];
    snprintf(topic, sizeof(topic), "push/%s", subtopic);

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, 1, 0);
    if (msg_id >= 0) {
        ESP_LOGI(TAG, "Published %s → %s (msg_id=%d)", topic, payload, msg_id);
    } else {
        ESP_LOGE(TAG, "Failed to publish to %s", topic);
    }
}

static void set_servo_angle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    const float min_us = 1000.0f;
    const float max_us = 2000.0f;
    const float period_us = 1000000.0f / SERVO_FREQ_HZ;
    float pulse_us = min_us + (max_us - min_us) * (angle / 180.0f);
    uint32_t max_duty = (1u << SERVO_RES) - 1u;
    uint32_t duty = (uint32_t)((pulse_us / period_us) * max_duty);
    ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
}

static void blink_timer_cb(TimerHandle_t) {
    gpio_set_level((gpio_num_t)LED3_GPIO, 0);
}

static void buzzer_blink_cb(TimerHandle_t) {
    gpio_set_level((gpio_num_t)BUZZER_GPIO, 0);
}

static void high_temp_beep_cb(TimerHandle_t timer) {
    if (!high_temp) {
        gpio_set_level((gpio_num_t)BUZZER_GPIO, 0);
        xTimerStop(timer, 0);
        return;
    }
    gpio_set_level((gpio_num_t)BUZZER_GPIO, !gpio_get_level((gpio_num_t)BUZZER_GPIO));
}

static void handle_ir_code(uint32_t code) {
    switch (code) {
        case 0x00FF30CF: // Button 1: toggle LED1
            s_led1_on = !s_led1_on;
            gpio_set_level((gpio_num_t)LED1_GPIO, s_led1_on);
            ESP_LOGI(TAG, "LED1 %s", s_led1_on ? "ON" : "OFF");
            mqtt_publish_push("led1", s_led1_on ? "ON" : "OFF");
            break;

        case 0x00FF18E7: // Button 2: toggle LED2
            s_led2_on = !s_led2_on;
            gpio_set_level((gpio_num_t)LED2_GPIO, s_led2_on);
            ESP_LOGI(TAG, "LED2 %s", s_led2_on ? "ON" : "OFF");
            mqtt_publish_push("led2", s_led2_on ? "ON" : "OFF");
            break;

        case 0x00FF02FD: // Servo to 180°
            set_servo_angle(180);
            mqtt_publish_push("servo", "180");
            break;

        case 0x00FF22DD: // Servo to 0°
            set_servo_angle(0);
            mqtt_publish_push("servo", "0");
            break;

        default:
            ESP_LOGI(TAG, "Unknown IR code: 0x%08X", code);
            break;
    }

    // Blink LED3 and brief buzzer beep on recognized action
    if (code != 0) {
        gpio_set_level((gpio_num_t)LED3_GPIO, 1);
        xTimerReset(s_blink_timer, 0);

        gpio_set_level((gpio_num_t)BUZZER_GPIO, 1);
        xTimerReset(buzzer_blink_timer, 0);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected, subscribing...");
            s_mqtt_connected = true;
            for (int i = 0; i < NUM_TOPICS; ++i) {
                esp_mqtt_client_subscribe(s_mqtt_client, SUB_TOPICS[i], 1);
            }
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected - retrying in 5s");
            s_mqtt_connected = false;
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_mqtt_client_reconnect(s_mqtt_client);  // Optimized reconnect
            break;

        case MQTT_EVENT_DATA: {
            char topic[64] = {0};
            char data[64] = {0};
            strncpy(topic, event->topic, event->topic_len);
            strncpy(data, event->data, event->data_len);

            // Trim data
            char *start = data;
            while (*start && (*start == ' ' || *start == '\t' || *start == '\n' || *start == '\r')) ++start;
            char *end = data + strlen(data);
            while (end > start && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\n' || end[-1] == '\r')) *--end = '\0';

            if (strcmp(topic, "miot/ir/code") == 0) {
                uint32_t code = (uint32_t)strtoul(start, nullptr, 0);
                handle_ir_code(code);
            } else if (strcmp(topic, "miot/ldr/raw") == 0) {
                int ldr_value = atoi(start);
                if (auto_mode) {
                    bool should_on = (ldr_value > 2800);
                    if (should_on != s_led1_on) {
                        s_led1_on = should_on;
                        s_led2_on = should_on;
                        gpio_set_level((gpio_num_t)LED1_GPIO, should_on);
                        gpio_set_level((gpio_num_t)LED2_GPIO, should_on);
                        mqtt_publish_push("led1", should_on ? "ON" : "OFF");
                        mqtt_publish_push("led2", should_on ? "ON" : "OFF");
                    }
                }
            } else if (strcmp(topic, "miot/dht22/temperature") == 0) {
                float temp = atof(start);
                high_temp = (temp > 22.0f);
                if (high_temp && !xTimerIsTimerActive(high_temp_beep_timer)) {
                    gpio_set_level((gpio_num_t)BUZZER_GPIO, 1);
                    xTimerStart(high_temp_beep_timer, 0);
                }
            }
            break;
        }
        default:
            break;
    }
}

static void start_mqtt_client(void) {
    char* ca_cert = read_file_from_spiffs(CA_CERT_PATH);
    char* client_cert = read_file_from_spiffs(CLIENT_CERT_PATH);
    char* client_key = read_file_from_spiffs(CLIENT_KEY_PATH);

    if (!ca_cert) {
        ESP_LOGE(TAG, "Failed to load CA cert - aborting MQTT");
        return;
    }

    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri = BROKER_URI;
    cfg.broker.verification.certificate = ca_cert;
    cfg.broker.verification.certificate_len = 0;
    cfg.broker.verification.skip_cert_common_name_check = true;

    cfg.credentials.authentication.certificate = client_cert;
    cfg.credentials.authentication.certificate_len = 0;
    cfg.credentials.authentication.key = client_key;
    cfg.credentials.authentication.key_len = 0;

    s_mqtt_client = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt_client));
}

static void print_qr(void) {
    const char* qr_payload = "{\"ver\":\"v1\",\"name\":\"PROV_ESP\",\"pop\":\"abcd1234\",\"transport\":\"softap\"}";
    esp_qrcode_config_t qr_cfg = ESP_QRCODE_CONFIG_DEFAULT();
    esp_qrcode_generate(&qr_cfg, qr_payload);
}

static void wifi_prov_handler(void *user_data, wifi_prov_cb_event_t event, void *event_data) {
    switch (event) {
        case WIFI_PROV_START:
            ESP_LOGI(TAG, "Provisioning started");
            break;
        case WIFI_PROV_CRED_RECV:
            ESP_LOGI(TAG, "Credentials received");
            break;
        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "Provisioning success");
            break;
        case WIFI_PROV_CRED_FAIL:
            ESP_LOGE(TAG, "Provisioning failed");
            break;
        case WIFI_PROV_END:
            wifi_prov_mgr_deinit();
            break;
        default:
            break;
    }
}

static void prov_start(void) {
    wifi_prov_mgr_config_t cfg = {
        .scheme = wifi_prov_scheme_softap,
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
        .app_event_handler = {
            .event_cb = wifi_prov_handler,
            .user_data = NULL
        },
        .wifi_prov_conn_cfg = {}  // silence warning
    };

    ESP_ERROR_CHECK(wifi_prov_mgr_init(cfg));

    bool is_provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&is_provisioned));

    wifi_prov_mgr_disable_auto_stop(10000);

    if (is_provisioned) {
        ESP_LOGI(TAG, "Already provisioned - starting STA mode");
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else {
        ESP_LOGI(TAG, "Starting provisioning (SoftAP)");
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(
            WIFI_PROV_SECURITY_1, "abcd1234", "PROV_ESP", NULL));
        print_qr();
    }
}

static void connection_event_handler(void* arg, esp_event_base_t event_base,
                                     int32_t event_id, void* event_data) {
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Wi-Fi connected - Got IP!");
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected - retrying");
        esp_wifi_connect();
    }
}

extern "C" void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize SPIFFS
    esp_vfs_spiffs_conf_t spiffs_cfg = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs_cfg));
    ESP_LOGI(TAG, "SPIFFS mounted");
    // list_spiffs_files();  // Uncomment if needed for debug

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    s_wifi_event_group = xEventGroupCreate();

    // Register connection events
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &connection_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                                               &connection_event_handler, NULL));

    // Start provisioning
    prov_start();

    // Wait for connection
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    // GPIO setup
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED1_GPIO) | (1ULL << LED2_GPIO) | (1ULL << LED3_GPIO) | (1ULL << BUZZER_GPIO);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level((gpio_num_t)LED1_GPIO, 0);
    gpio_set_level((gpio_num_t)LED2_GPIO, 0);
    gpio_set_level((gpio_num_t)LED3_GPIO, 0);
    gpio_set_level((gpio_num_t)BUZZER_GPIO, 0);

    gpio_config_t btn_conf = {};
    btn_conf.intr_type = GPIO_INTR_DISABLE;
    btn_conf.mode = GPIO_MODE_INPUT;
    btn_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    btn_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&btn_conf));

    // Timers
    s_blink_timer = xTimerCreate("blink", pdMS_TO_TICKS(150), pdFALSE, nullptr, blink_timer_cb);
    buzzer_blink_timer = xTimerCreate("buzzer_blink", pdMS_TO_TICKS(150), pdFALSE, nullptr, buzzer_blink_cb);
    high_temp_beep_timer = xTimerCreate("high_temp_beep", pdMS_TO_TICKS(500), pdTRUE, nullptr, high_temp_beep_cb);

    // Servo setup
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = SERVO_LEDC_MODE;
    ledc_timer.duty_resolution = SERVO_RES;
    ledc_timer.timer_num = SERVO_LEDC_TIMER;
    ledc_timer.freq_hz = SERVO_FREQ_HZ;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_chan = {};
    ledc_chan.gpio_num = SERVO_GPIO;
    ledc_chan.speed_mode = SERVO_LEDC_MODE;
    ledc_chan.channel = SERVO_LEDC_CHANNEL;
    ledc_chan.intr_type = LEDC_INTR_DISABLE;
    ledc_chan.timer_sel = SERVO_LEDC_TIMER;
    ledc_chan.duty = 0;
    ledc_chan.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_chan));
    set_servo_angle(0);

    // Start MQTT
    start_mqtt_client();

    // Main loop for button
    TickType_t last_button_time = 0;
    bool last_button_state = true;
    while (true) {
        bool button_state = gpio_get_level((gpio_num_t)BUTTON_GPIO);
        if (button_state == 0 && last_button_state == 1) {
            TickType_t now = xTaskGetTickCount();
            if (now - last_button_time > pdMS_TO_TICKS(200)) {
                auto_mode = !auto_mode;
                ESP_LOGI(TAG, "Auto mode %s", auto_mode ? "ENABLED" : "DISABLED");
                mqtt_publish_push("auto_mode", auto_mode ? "ON" : "OFF");
                last_button_time = now;
            }
        }
        last_button_state = button_state;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}