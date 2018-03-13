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
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/*
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
*/

// espmqtt library
#include "mqtt.h"

#include "ubx.h"

#define SSID "ofi204"
#define PASSWORD "peql1234"
#define UDP_PORT 2207
#define IP_ADDRESS "10.73.32.164"

#define CONNECTED_BIT (1<<0)

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 30 /* Time ESP32 will go to sleep (in seconds) */

#define ENABLE_GPS 2

static const char *TAG = "wifi";

static EventGroupHandle_t wifi_event_group;

/*
struct Payload{
	uint32_t latitude;
	uint32_t longitude;
};
*/

//struct sockaddr_in saddr;

QueueHandle_t UDP_Queue_Handle=0;

TimerHandle_t xTimers;

static char * gps_rx_buffer;

static struct GPS_RX_STATS gpsRxStats;



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
/*
static int create_socket()
{
    int sock = -1;

    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(UDPTAG, "Failed to create socket. Error %d", errno);
        close(sock);
        return -1;
    }

    memset((char *) &saddr, 0, sizeof(struct sockaddr_in));

    saddr.sin_family = PF_INET;
    saddr.sin_port = htons(UDP_PORT);
    saddr.sin_addr.s_addr = inet_addr(IP_ADDRESS);

    // All set, socket is configured for sending and receiving
    return sock;

}
*/


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

void mqtt_connected_callback(mqtt_client *client, mqtt_event_data_t *event_data)
{

	GPSPositionData *pMandar;

    while(1) {

		if (xQueueReceive(UDP_Queue_Handle, &(pMandar), portMAX_DELAY)) {
			char pos_string[60];
			//char lon_string[30];
			sprintf(pos_string, "{ \"latitude\": %.7f, \"longitude\": %.7f }", pMandar->Latitude, pMandar->Longitude);
			//sprintf(lon_string, "%.7f", pMandar->Longitude);
			mqtt_publish(client, "/esp/1/pos", pos_string, strlen(pos_string), 0, 0);
//			mqtt_publish(client, "/esp/longitude", lon_string, strlen(lon_string), 0, 0);
			printf("Latitud Publicada: %.7f Longitude enviada: %.7f\n", pMandar->Latitude, pMandar->Longitude);
		}
    }
}


    // MQTT client configuration
    mqtt_settings settings = {
    	.host = "10.73.32.152",
    	.port = 1883,
    	.client_id = "esp01",
    	.clean_session = 0,
    	.keepalive = 120,
    	.connected_cb = mqtt_connected_callback
    };

void gps_task(void *pvParameter){

	printf("GPS Demo\r\n\r\n");
	portTickType xDelay = 100 / portTICK_RATE_MS;
		// configure the UART1 controller, connected to the GPS receiver
		uart_config_t uart_config = {
	        .baud_rate = 9600,
	        .data_bits = UART_DATA_8_BITS,
	        .parity    = UART_PARITY_DISABLE,
	        .stop_bits = UART_STOP_BITS_1,
	        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	    };

	    uart_param_config(UART_NUM_1, &uart_config);
	    uart_set_pin(UART_NUM_1, 4, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); //4 16
	    uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);

	    GPSPositionData gpsposition;

	    GPSPositionData *pLeer;
	    //struct Payload Paquete1;
	    //struct Payload *pLeer = &Paquete1;
	    pLeer= (GPSPositionData *) pvPortMalloc(sizeof(GPSPositionData));
	    //pLeer = &Paquete1;
	    pLeer = & gpsposition;
		// parse any incoming messages and print it
		float previuslat=0.0;
		float previuslon=0.0;
		float distancia=0.0;
	    while (1)
	    	{
	    		uint8_t c;
	    		// This blocks the task until there is something on the buffer
	    		while (uart_read_bytes(UART_NUM_1, &c, 1, xDelay) > 0)
	    		{
	    			int res;

	    			res = parse_ubx_stream (c, gps_rx_buffer, &gpsposition, &gpsRxStats);
	    			if (res == PARSER_COMPLETE) {
	    				//pLeer ->Latitude=gpsposition.Latitude;
	    				//pLeer ->Longitude=gpsposition.Longitude;
	    				printf("Parce complete:\n");
	    				printf("pLeer:\n	latitude:  %.7f\n	longitude  %.7f, Status:%d \n", pLeer->Latitude, pLeer->Longitude, gpsposition.Status);
	    				printf("pLeer:\n	Position DOP:  %.2f\n	Horizontal DOP  %.2f, Vertical DOP:%.2f \n", pLeer->PDOP, pLeer->HDOP, pLeer->VDOP);
	    				if(previuslat!=0.0)
	    					distancia=distance(previuslat, previuslon, pLeer->Latitude, pLeer->Longitude);
	    				previuslat=pLeer->Latitude;
	    				previuslon=pLeer->Longitude;
	    				printf("Distancia: %.2f\n", distancia);
	    				if (gpsposition.Status!=GPSPOSITION_STATUS_NOFIX){
	    					if (!xQueueSend(UDP_Queue_Handle, ( void * ) &pLeer, (TickType_t) 0)) {
	    						ESP_LOGE(TAG, "Failed to Send pLeer");
	    					}
	    				else
	    					ESP_LOGI(TAG, "GPS is not Fix");
	    				}
	    			}


	    		}
	    }
}
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
	ESP_ERROR_CHECK(esp_wifi_start());
	printf("Connecting to %s\n", SSID);

	ESP_LOGI(TAG, "Loop task: waiting for connection to the wifi network... ");
	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
	ESP_LOGI(TAG, "connected!\n");

	// start the MQTT client
	printf("Connecting to the MQTT server... ");
	mqtt_start(&settings);

	gpio_pad_select_gpio(ENABLE_GPS);
	gpio_set_direction(ENABLE_GPS, GPIO_MODE_OUTPUT);
	gpio_set_level(ENABLE_GPS, 1);

	gps_rx_buffer = pvPortMalloc(sizeof(struct UBXPacket));

	UDP_Queue_Handle =xQueueCreate(3,sizeof(GPSPositionData));

    xTimers = xTimerCreate( "Timer", pdMS_TO_TICKS( 10000 ), pdTRUE, ( void * ) 0, vTimerCallback );
    if( xTimerStart( xTimers, 0 ) != pdPASS )
      {
		ESP_LOGE(TAG, "Failed to Send pLeer");
      }

    xTaskCreate(&gps_task, "gps_task", 2048, NULL, 5, NULL);  //GPS

}



