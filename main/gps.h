/*
 * gps.h
 *
 *  Created on: 9 mar. 2018
 *      Author: leandro
 */

#ifndef MAIN_GPS_H_
#define MAIN_GPS_H_

void mqtt_connected_callback(mqtt_client *client, mqtt_event_data_t *event_data);
void mqtt_init(void);
void gps_task(void *pvParameter);

#endif /* MAIN_GPS_H_ */
