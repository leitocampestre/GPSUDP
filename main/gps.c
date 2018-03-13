/*
 * gps.c
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
#include "freertos/FreeRTOS.h"
#include "driver/uart.h"

#include "ubx.h"
#include "mqtt.h"
#include "gps.h"
#include "wifi.h"

static char * gps_rx_buffer;
static struct GPS_RX_STATS gpsRxStats;
QueueHandle_t UDP_Queue_Handle=0;
static const char *TAG = "Gps";

extern int conexion;

mqtt_settings settings = {
    	.host = "10.73.32.152",
    	.port = 1883,
    	.client_id = "esp01",
    	.clean_session = 0,
    	.keepalive = 120,
    	.connected_cb = mqtt_connected_callback
    };

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

void mqtt_init(void){

	printf("Connecting to the MQTT server... ");
	mqtt_start(&settings);

}

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
	    UDP_Queue_Handle = xQueueCreate(5,sizeof(GPSPositionData));//revisar

	    pLeer= (GPSPositionData *) pvPortMalloc(sizeof(GPSPositionData));
	    //pLeer = &Paquete1;
	    pLeer = & gpsposition;
		// parse any incoming messages and print it
		float previuslat=0.0;
		float previuslon=0.0;
		float distancia=0.0;

		gps_rx_buffer = pvPortMalloc(sizeof(struct UBXPacket));

	    while (1)
	    	{
	    		uint8_t c;

	    		if (conexion==1){
	    			//connect_wifi();
	    			mqtt_init();
	    			conexion=0;
	    		}
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
