/*
 * sleep.c
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
#include "driver/gpio.h"

#include "sleep.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 30 /* Time ESP32 will go to sleep (in seconds) */

#define ENABLE_GPS 2

TimerHandle_t xTimers;

static const char *TAG = "Deep_sleep";


void vTimerCallback( TimerHandle_t xTimer )
 {
 const uint32_t ulMaxExpiryCountBeforeStopping = 3;
 uint32_t ulCount;

    /* The number of times this timer has expired is saved as the
    timer's ID.  Obtain the count. */
    ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );

    /* Increment the count, then test to see if the timer has expired
    ulMaxExpiryCountBeforeStopping yet. */
    ulCount++;

    ESP_LOGI(TAG, "Cuentas de 10 segundos %d", ulCount);
    ESP_LOGI(TAG, "GPIO to Vcc");


    /* If the timer has expired 10 times then stop it from running. */
    if( ulCount >= ulMaxExpiryCountBeforeStopping )
    {
        /* Do not use a block time if calling a timer API function
        from a timer callback function, as doing so could cause a
        deadlock! */
        xTimerStop( xTimers, 0 );
        gpio_set_level(ENABLE_GPS, 0);
        ESP_LOGI(TAG, "GPIO to Ground");
        esp_deep_sleep(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    }
    else
    {
       /* Store the incremented count back into the timer's ID field
       so it can be read back again the next time this software timer
       expires. */
       vTimerSetTimerID( xTimer, ( void * ) ulCount );
    }

 }

void timerinit(void){

	gpio_pad_select_gpio(ENABLE_GPS);
	gpio_set_direction(ENABLE_GPS, GPIO_MODE_OUTPUT);
	gpio_set_level(ENABLE_GPS, 1);

    xTimers = xTimerCreate( "Timer", pdMS_TO_TICKS( 10000 ), pdTRUE, ( void * ) 0, vTimerCallback );

    if( xTimerStart( xTimers, 0 ) != pdPASS )
      {
		ESP_LOGE(TAG, "Failed to Send pLeer");
      }

}

