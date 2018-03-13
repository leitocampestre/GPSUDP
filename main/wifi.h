/*
 * wifi.h
 *
 *  Created on: 9 mar. 2018
 *      Author: leandro
 */

#ifndef MAIN_WIFI_H_
#define MAIN_WIFI_H_

#include "wifi.h"

void wifi_init2(void);

void connect_wifi(void);

esp_err_t event_handler(void *ctx, system_event_t *event);

#endif /* MAIN_WIFI_H_ */
