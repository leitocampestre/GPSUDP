#include <stdio.h>
#include <string.h>
#include <math.h>

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

// espmqtt library
#include "mqtt.h"

#include "gps.h"
#include "wifi.h"
#include "sleep.h"

/*
struct Payload{
	uint32_t latitude;
	uint32_t longitude;
};
*/
//struct sockaddr_in saddr;

static const char *TAG = "init";

int conexion;
void app_main()
{

	esp_log_level_set(TAG, ESP_LOG_INFO);
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    conexion =0;
    wifi_init2();

    timerinit();

    xTaskCreate(&gps_task, "gps_task", 2048, NULL, 5, NULL);  //GPS

}



