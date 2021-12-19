/* (c)Andriy Malyshenko Dec 2021
	simple wifi implementation after upgared to ESPIDF@4
 * Copyright 2021 Andriy Malyshenko (andriy@malyshenko.com)
*/

#ifndef __WIFI_H__
#define __WIFI_H__

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define ESP_WIFI_SSID      CONFIG_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  10

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

void wifi_init_sta(void);

#endif