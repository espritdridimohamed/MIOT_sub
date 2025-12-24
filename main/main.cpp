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
}

#include <cstring>
#include <cstdlib>

static const char *TAG = "MIOT_SUB";

// Wi‑Fi credentials
static constexpr const char *WIFI_SSID = "DRIDI-MOHAMED";
static constexpr const char *WIFI_PASS = "26941414";

// MQTT broker (default port 1883)
static constexpr const char *BROKER_URI = "mqtt://192.168.1.17";

// Topics to subscribe
static constexpr const char *SUB_TOPICS[] = {
	"miot/ir/code",
	"miot/ldr/raw",
	"miot/dht22/temperature",
	"miot/dht22/humidity"
};
static constexpr int NUM_TOPICS = sizeof(SUB_TOPICS) / sizeof(SUB_TOPICS[0]);

// Event group to signal when connected
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

static esp_mqtt_client_handle_t s_mqtt_client = nullptr;

// GPIO pins
static constexpr int LED1_GPIO = 15; // D15
static constexpr int LED2_GPIO = 4;  // D4
static constexpr int LED3_GPIO = 5;  // D5 (blink on IR)
static constexpr int SERVO_GPIO = 18; // D18

// LED states
static volatile bool s_led1_on = false;
static volatile bool s_led2_on = false;

// Blink timer for LED3
static TimerHandle_t s_blink_timer = nullptr;

// LEDC (PWM) for servo
static constexpr ledc_timer_t SERVO_LEDC_TIMER = LEDC_TIMER_0;
static constexpr ledc_channel_t SERVO_LEDC_CHANNEL = LEDC_CHANNEL_0;
static constexpr ledc_mode_t SERVO_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static constexpr uint32_t SERVO_FREQ_HZ = 50; // 50 Hz for standard servos
static constexpr ledc_timer_bit_t SERVO_RES = LEDC_TIMER_16_BIT; // 16-bit duty resolution

static void set_servo_angle(int angle) {
	if (angle < 0) angle = 0;
	if (angle > 180) angle = 180;
	const float min_us = 1000.0f; // 1.0 ms
	const float max_us = 2000.0f; // 2.0 ms
	const float period_us = 1000000.0f / (float)SERVO_FREQ_HZ; // 20000 us
	float pulse_us = min_us + (max_us - min_us) * ((float)angle / 180.0f);
	uint32_t max_duty = (1u << SERVO_RES) - 1u;
	uint32_t duty = (uint32_t)((pulse_us / period_us) * (float)max_duty);
	ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
	ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
}

static void blink_timer_cb(TimerHandle_t) {
	gpio_set_level((gpio_num_t)LED3_GPIO, 0);
}

static void handle_ir_code(uint32_t code) {
	switch (code) {
		case 0x00FF30CF: // Button 1: toggle LED1 (GPIO15)
			s_led1_on = !s_led1_on;
			gpio_set_level((gpio_num_t)LED1_GPIO, s_led1_on ? 1 : 0);
			ESP_LOGI(TAG, "LED1 (GPIO%d) %s", LED1_GPIO, s_led1_on ? "ON" : "OFF");
			break;
		case 0x00FF18E7: // Button 2: toggle LED2 (GPIO4)
			s_led2_on = !s_led2_on;
			gpio_set_level((gpio_num_t)LED2_GPIO, s_led2_on ? 1 : 0);
			ESP_LOGI(TAG, "LED2 (GPIO%d) %s", LED2_GPIO, s_led2_on ? "ON" : "OFF");
			break;
		case 0x00FF02FD: // Servo: 0 -> 180 and stay
			ESP_LOGI(TAG, "Servo to 180°");
			set_servo_angle(180);
			break;
		case 0x00FF22DD: // Servo: 180 -> 0
			ESP_LOGI(TAG, "Servo to 0°");
			set_servo_angle(0);
			break;
		default:
			ESP_LOGI(TAG, "Unknown IR code: 0x%08X", code);
			break;
	}
	// Blink LED3 briefly on each IR code reception
	gpio_set_level((gpio_num_t)LED3_GPIO, 1);
	if (s_blink_timer) {
		xTimerStop(s_blink_timer, 0);
		xTimerChangePeriod(s_blink_timer, pdMS_TO_TICKS(150), 0);
		xTimerStart(s_blink_timer, 0);
	}
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
	if (event_base == WIFI_EVENT) {
		switch (event_id) {
			case WIFI_EVENT_STA_START:
				ESP_LOGI(TAG, "Wi‑Fi started, connecting to AP...");
				esp_wifi_connect();
				break;
			case WIFI_EVENT_STA_DISCONNECTED:
				ESP_LOGW(TAG, "Disconnected from AP, retrying...");
				esp_wifi_connect();
				xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
				break;
			default:
				break;
		}
	}
}

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
	if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		auto *event = static_cast<ip_event_got_ip_t *>(event_data);
		ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
	esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);
	switch (event_id) {
		case MQTT_EVENT_CONNECTED:
			ESP_LOGI(TAG, "MQTT connected, subscribing to topics...");
			for (int i = 0; i < NUM_TOPICS; ++i) {
				int msg_id = esp_mqtt_client_subscribe(s_mqtt_client, SUB_TOPICS[i], 1);
				ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", SUB_TOPICS[i], msg_id);
			}
			break;
		case MQTT_EVENT_DISCONNECTED:
			ESP_LOGW(TAG, "MQTT disconnected");
			break;
		case MQTT_EVENT_DATA: {
			// Copy topic and data to zero-terminated buffers for printing
			int tlen = event->topic_len;
			int dlen = event->data_len;
			char *topic = (char *) malloc(tlen + 1);
			char *data = (char *) malloc(dlen + 1);
			if (topic && data) {
				memcpy(topic, event->topic, tlen);
				topic[tlen] = '\0';
				memcpy(data, event->data, dlen);
				data[dlen] = '\0';
				ESP_LOGI(TAG, "[%s] %s", topic, data);

				// If IR code topic, parse and handle
				if (strcmp(topic, "miot/ir/code") == 0) {
					// Trim leading/trailing whitespace
					char *start = data;
					while (*start == ' ' || *start == '\t' || *start == '\n' || *start == '\r') start++;
					char *end = data + strlen(data);
					while (end > start && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\n' || end[-1] == '\r')) {
						*--end = '\0';
					}

					// Accept formats like "0x00FF30CF", plain hex (A-F), or decimal
					const char *p = start;
					uint32_t code = 0;
					if ((end - start) >= 2 && start[0] == '0' && (start[1] == 'x' || start[1] == 'X')) {
						p += 2;
						code = strtoul(p, nullptr, 16);
					} else {
						// Detect non-hex chars to decide base
						bool all_hex = true;
						for (const char *q = p; q < end; ++q) {
							char c = *q;
							if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) {
								all_hex = false;
								break;
							}
						}
						code = strtoul(p, nullptr, all_hex ? 16 : 10);
					}
					ESP_LOGI(TAG, "Parsed IR code: 0x%08X", code);
					handle_ir_code(code);
				}
			}
			free(topic);
			free(data);
			break;
		}
		default:
			break;
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

	// Initialize network stack and event loop
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	// Create default Wi‑Fi station
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	// Create event group before starting Wi‑Fi to avoid null access in handlers
	s_wifi_event_group = xEventGroupCreate();

	// Register event handlers
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));

	// Configure Wi‑Fi
	wifi_config_t wifi_config = {};
	strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), WIFI_SSID, sizeof(wifi_config.sta.ssid));
	strncpy(reinterpret_cast<char *>(wifi_config.sta.password), WIFI_PASS, sizeof(wifi_config.sta.password));
	wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; // adjust if AP uses different auth

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	// Wait for IP
	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
	if (bits & WIFI_CONNECTED_BIT) {
		ESP_LOGI(TAG, "Wi‑Fi connected, starting MQTT");
	} else {
		ESP_LOGE(TAG, "Failed to connect to Wi‑Fi");
	}

	// Init GPIOs for LEDs
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL << LED1_GPIO) | (1ULL << LED2_GPIO) | (1ULL << LED3_GPIO);
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	ESP_ERROR_CHECK(gpio_config(&io_conf));
	gpio_set_level((gpio_num_t)LED1_GPIO, 0);
	gpio_set_level((gpio_num_t)LED2_GPIO, 0);
	gpio_set_level((gpio_num_t)LED3_GPIO, 0);
	s_led1_on = false;
	s_led2_on = false;

	// Create blink timer for LED3
	s_blink_timer = xTimerCreate("blink", pdMS_TO_TICKS(150), pdFALSE, nullptr, blink_timer_cb);

	// Init LEDC for servo PWM
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
	// Start servo at 0°
	set_servo_angle(0);

	// Configure and start MQTT
	esp_mqtt_client_config_t mqtt_cfg = {};
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
	mqtt_cfg.broker.address.uri = BROKER_URI;
#else
	mqtt_cfg.uri = BROKER_URI;
#endif

	s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
	ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_mqtt_client, (esp_mqtt_event_id_t) ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
	ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt_client));

	// Keep the app running
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

