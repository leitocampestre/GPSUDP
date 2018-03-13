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

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "minmea.h"

#define SSID "ofi204"
#define PASSWORD "peql1234"
#define UDP_PORT 2207
#define IP_ADDRESS "10.73.32.164"

#define CONNECTED_BIT (1<<0)

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 10 /* Time ESP32 will go to sleep (in seconds) */

static const char *TAG = "wifi";
static const char *UDPTAG = "udp";

static EventGroupHandle_t wifi_event_group;

struct Payload{
	int latitude;
	int longitude;
};

struct sockaddr_in saddr;

QueueHandle_t UDP_Queue_Handle=0;

TimerHandle_t xTimers;

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

// read a line from the UART controller
char* read_line(uart_port_t uart_controller) {

	static char line[MINMEA_MAX_LENGTH];
	char *ptr = line;

	while(1) {

		int num_read = uart_read_bytes(uart_controller, (unsigned char *)ptr, 1, portMAX_DELAY);
		if(num_read == 1) {

			// new line found, terminate the string and return
			if(*ptr == '\n') {
				ptr++;
				*ptr = '\0';
				return line;
			}

			// else move to the next char
			ptr++;
		}
	}
}
/*
void Swap (struct Payload * Swap1, struct Payload * Swap2) {
    struct Payload pSwap = *Swap1;
    *Swap1 = *Swap2;
    *Swap2 = pSwap;
    return;
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

    /* If the timer has expired 10 times then stop it from running. */
    if( ulCount >= ulMaxExpiryCountBeforeStopping )
    {
        /* Do not use a block time if calling a timer API function
        from a timer callback function, as doing so could cause a
        deadlock! */
        xTimerStop( xTimers, 0 );
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

void loop_task(void *pvParameter)
{
	int sock;
	//struct Payload Paquete2;
	//struct Payload *pMandar = &Paquete2;
	struct Payload *pMandar;
	// wait for connection
	ESP_LOGI(TAG, "Loop task: waiting for connection to the wifi network... ");
	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
	ESP_LOGI(TAG, "connected!\n");

	tcpip_adapter_ip_info_t ip_info;
	ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
	printf("IP Address:  %s\n", ip4addr_ntoa(&ip_info.ip));
	printf("Subnet mask: %s\n", ip4addr_ntoa(&ip_info.netmask));
	printf("Gateway:     %s\n", ip4addr_ntoa(&ip_info.gw));

    sock = create_socket();
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
    }

    while(1) {

		if (xQueueReceive(UDP_Queue_Handle, &(pMandar), portMAX_DELAY)) {
			int r;
			//Swap(pLeer, pMandar);
			r=sendto(sock, (unsigned char *) pMandar, sizeof(struct Payload), 0, (struct sockaddr *) &saddr , sizeof(struct sockaddr_in));
			if(r!=sizeof(struct Payload)){
				ESP_LOGE(UDPTAG, "Failed to send UDP");
			}

			printf("Send UDP to:\n");
			printf("pMandar:\n	latitude:  %d\n	longitude  %d\n", pMandar->latitude, pMandar->longitude);
			//vTaskDelay(5000 / portTICK_RATE_MS);
		}

    }
}

void gps_task(void *pvParameter){

	printf("GPS Demo\r\n\r\n");
	bool lastNaN=pdFALSE;
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

		// GPS variables and initial state
		float latitude = -1.0;
		float longitude = -1.0;
		int fix_quality = -1;
	    int satellites_tracked = -1;

	    struct Payload Paquete1;
	    struct Payload *pLeer = &Paquete1;

		// parse any incoming messages and print it
		while(1) {
			// read a line from the receiver
		    pLeer->latitude=0;
			pLeer->longitude=0;
			char *line = read_line(UART_NUM_1);
			//pLeer->latitude=13;
			//pLeer->longitude=19;
			// parse the line
			switch (minmea_sentence_id(line, false)) {

	            case MINMEA_SENTENCE_RMC: {

					struct minmea_sentence_rmc frame;
	                if (minmea_parse_rmc(&frame, line)) {

						// latitude valid and changed? apply a threshold
						float new_latitude = minmea_tocoord(&frame.latitude);
						if((new_latitude != NAN) && (abs(new_latitude - latitude) > 0.001)) {
							latitude = new_latitude;
							pLeer->latitude=new_latitude;
							printf("New latitude: %f\n", latitude);
						}

						// longitude valid and changed? apply a threshold
						float new_longitude = minmea_tocoord(&frame.longitude);
						if((new_longitude != NAN) && (abs(new_longitude - longitude) > 0.001)) {
							longitude = minmea_tocoord(&frame.longitude);
							pLeer->longitude=longitude;
							printf("New longitude: %f\n", longitude);

							if(lastNaN==pdTRUE){
								ESP_LOGI(TAG, "Last longitude is not a number");
								if (!isnan(longitude)) {
									lastNaN=pdFALSE;
									if (!xQueueSend(UDP_Queue_Handle, ( void * ) &pLeer, portMAX_DELAY)) {
										ESP_LOGE(TAG, "Failed to Send pLeer");
									}
								}
							}

							if (!isnan(longitude)) {
								if (!xQueueSend(UDP_Queue_Handle, ( void * ) &pLeer, portMAX_DELAY)) {
									ESP_LOGE(TAG, "Failed to Send pLeer");
								}
							}
							else{
								lastNaN=pdTRUE;
							}

						}
					}
	            } break;

	            case MINMEA_SENTENCE_GGA: {

					struct minmea_sentence_gga frame;
	                if (minmea_parse_gga(&frame, line)) {

						// fix quality changed?
						if(frame.fix_quality != fix_quality) {
							fix_quality = frame.fix_quality;
							printf("New fix quality: %d\n", fix_quality);
						}
	                }
	            } break;

	            case MINMEA_SENTENCE_GSV: {

					struct minmea_sentence_gsv frame;
	                if (minmea_parse_gsv(&frame, line)) {

						// number of satellites changed?
						if(frame.total_sats != satellites_tracked) {
							satellites_tracked = frame.total_sats;
							printf("New satellites tracked: %d\n", satellites_tracked);
						}
					}
	            } break;

				default: break;
	        }
			printf("pLeer:\n	latitude:  %d\n	longitude  %d\n", pLeer->latitude, pLeer->longitude);
			vTaskDelay(2000 / portTICK_RATE_MS);
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

	UDP_Queue_Handle =xQueueCreate(3,sizeof(struct Payload));

    xTimers = xTimerCreate( "Timer", pdMS_TO_TICKS( 10000 ), pdTRUE, ( void * ) 0, vTimerCallback );
    if( xTimerStart( xTimers, 0 ) != pdPASS )
      {
		ESP_LOGE(TAG, "Failed to Send pLeer");
      }

    xTaskCreate(&loop_task, "loop_task", 2048, NULL, 5, NULL); //UDP

    xTaskCreate(&gps_task, "gps_task", 2048, NULL, 5, NULL);  //GPS
}
