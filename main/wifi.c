/*
 * wifi.c
 *
 *  Created on: 9 mar. 2018
 *      Author: leandro
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "sdkconfig.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "wifi.h"
#include "mqtt.h"
#include "ubx.h"


#define SSID "ofi204"
#define PASSWORD "peql1234"
#define UDP_PORT 2207
#define IP_ADDRESS "10.73.32.164"


#define CONNECTED_BIT (1<<0)
#define WIFI_BIT (1<<1)
// MQTT client configuration


static EventGroupHandle_t wifi_event_group;
static const char *TAG = "Wifi";


esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch(event->event_id){
		case SYSTEM_EVENT_STA_START:
			ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
			ESP_ERROR_CHECK(esp_wifi_connect());
			break;
		case SYSTEM_EVENT_STA_DISCONNECTED:
			xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
			break;
		case SYSTEM_EVENT_STA_GOT_IP:
			xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
			ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
			ESP_LOGI(TAG, "got ip:%s\n", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
			break;/**< ESP32 station got IP from connected AP */
		case SYSTEM_EVENT_STA_LOST_IP:
			break;
		default:
			break;
	}
    return ESP_OK;
}



void wifi_init2(void){

    wifi_event_group = xEventGroupCreate();

	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

	wifi_config_t wifi_config = {
        .sta = {
            .ssid = SSID,
            .password = PASSWORD,
        },
    };
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));;
	/*
	ESP_ERROR_CHECK(esp_wifi_start());
	printf("Connecting to %s\n", SSID);

	ESP_LOGI(TAG, "Loop task: waiting for connection to the wifi network... ");
	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
	ESP_LOGI(TAG, "connected!\n");
	*/
}

void connect_wifi(void){
	ESP_ERROR_CHECK(esp_wifi_start());
	//printf("Connecting to %s\n", SSID);

	ESP_LOGI(TAG, "Loop task: waiting for connection to the wifi network... ");
	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
	ESP_LOGI(TAG, "connected!\n");
}
