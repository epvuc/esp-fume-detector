/*
Copyright (c) 2017 Tony Pottier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

@file wifi_manager.c
@author Tony Pottier
@brief Defines all functions necessary for esp32 to connect to a wifi/scan wifis

Contains the freeRTOS task and all necessary support

@see https://idyl.io
@see https://github.com/tonyp7/esp32-wifi-manager
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "mdns.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/netdb.h"

#include "json.h"
#include "http_server.h"
#include "wifi_manager.h"




SemaphoreHandle_t wifi_manager_json_mutex = NULL;
uint16_t ap_num = MAX_AP_NUM;
wifi_ap_record_t *accessp_records; //[MAX_AP_NUM];
char *accessp_json = NULL;
char *ip_info_json = NULL;
wifi_config_t* wifi_manager_config_sta = NULL;

const char wifi_manager_nvs_namespace[] = "espwifimgr";

EventGroupHandle_t wifi_manager_event_group;

/* @brief indicate that the ESP32 is currently connected. */
const int WIFI_MANAGER_WIFI_CONNECTED_BIT = BIT0;


const int WIFI_MANAGER_AP_STA_CONNECTED_BIT = BIT1;

/* @brief Set automatically once the SoftAP is started */
const int WIFI_MANAGER_AP_STARTED = BIT2;

/* @brief When set, means a client requested to connect to an access point.*/
const int WIFI_MANAGER_REQUEST_STA_CONNECT_BIT = BIT3;

/* @brief This bit is set automatically as soon as a connection was lost */
const int WIFI_MANAGER_STA_DISCONNECT_BIT = BIT4;

/* @brief When set, means a client requested to scan wireless networks. */
const int WIFI_MANAGER_REQUEST_WIFI_SCAN = BIT5;

/* @brief When set, means a client requested to disconnect from currently connected AP. */
const int WIFI_MANAGER_REQUEST_WIFI_DISCONNECT = BIT6;


void wifi_manager_scan_async(){
	xEventGroupSetBits(wifi_manager_event_group, WIFI_MANAGER_REQUEST_WIFI_SCAN);
}

void wifi_manager_disconnect_async(){
	xEventGroupSetBits(wifi_manager_event_group, WIFI_MANAGER_REQUEST_WIFI_DISCONNECT);
}


esp_err_t wifi_manager_save_sta_config(){

	nvs_handle handle;
	esp_err_t esp_err;
	if(wifi_manager_config_sta){

		esp_err = nvs_open(wifi_manager_nvs_namespace, NVS_READWRITE, &handle);
		if (esp_err != ESP_OK) return esp_err;

		esp_err = nvs_set_blob(handle, "ssid", wifi_manager_config_sta->sta.ssid, 32);
		if (esp_err != ESP_OK) return esp_err;

		esp_err = nvs_set_blob(handle, "password", wifi_manager_config_sta->sta.password, 64);
		if (esp_err != ESP_OK) return esp_err;

		esp_err = nvs_commit(handle);
		if (esp_err != ESP_OK) return esp_err;

		nvs_close(handle);
	}

	return ESP_OK;
}

bool wifi_manager_fetch_wifi_sta_config(){

	nvs_handle handle;
	esp_err_t esp_err;
	if(nvs_open(wifi_manager_nvs_namespace, NVS_READONLY, &handle) == ESP_OK){

		if(wifi_manager_config_sta == NULL){
			wifi_manager_config_sta = (wifi_config_t*)malloc(sizeof(wifi_config_t));
		}
		memset(wifi_manager_config_sta, 0x00, sizeof(wifi_config_t));

		/* allocate 64 bytes of buffer */
		size_t sz = sizeof(wifi_manager_config_sta->sta.password);
		uint8_t *buff = (uint8_t*)malloc(sizeof(uint8_t) * sz);
		memset(buff, 0x00, sizeof(sz));

		/* ssid */
		sz = sizeof(wifi_manager_config_sta->sta.ssid);
		esp_err = nvs_get_blob(handle, "ssid", buff, &sz);
		if(esp_err != ESP_OK){
			free(buff);
			return false;
		}
		memcpy(wifi_manager_config_sta->sta.ssid, buff, sz);

		/* password */
		sz = sizeof(wifi_manager_config_sta->sta.password);
		esp_err = nvs_get_blob(handle, "password", buff, &sz);
		if(esp_err != ESP_OK){
			free(buff);
			return false;
		}
		memcpy(wifi_manager_config_sta->sta.password, buff, sz);


		free(buff);
		nvs_close(handle);

#if WIFI_MANAGER_DEBUG
		printf("wifi_manager_fetch_wifi_sta_config: ssid:%s password:%s\n",wifi_manager_config_sta->sta.ssid,wifi_manager_config_sta->sta.password);
#endif
		return wifi_manager_config_sta->sta.ssid[0] != '\0';

	}
	else{
		return false;
	}

}


void wifi_manager_clear_ip_info_json(){
	strcpy(ip_info_json, "{}\n");
}


void wifi_manager_generate_ip_info_json(update_reason_code_t update_reason_code){

	wifi_config_t *config = wifi_manager_get_wifi_sta_config();
	if(config){

		const char ip_info_json_format[] = ",\"ip\":\"%s\",\"netmask\":\"%s\",\"gw\":\"%s\",\"urc\":%d}\n";

		memset(ip_info_json, 0x00, JSON_IP_INFO_SIZE);

		/* to avoid declaring a new buffer we copy the data directly into the buffer at its correct address */
		strcpy(ip_info_json, "{\"ssid\":");
		json_print_string(config->sta.ssid,  (unsigned char*)(ip_info_json+strlen(ip_info_json)) );

		if(update_reason_code == UPDATE_CONNECTION_OK){
			/* rest of the information is copied after the ssid */
			tcpip_adapter_ip_info_t ip_info;
			ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
			char ip[IP4ADDR_STRLEN_MAX]; /* note: IP4ADDR_STRLEN_MAX is defined in lwip */
			char gw[IP4ADDR_STRLEN_MAX];
			char netmask[IP4ADDR_STRLEN_MAX];
			strcpy(ip, ip4addr_ntoa(&ip_info.ip));
			strcpy(netmask, ip4addr_ntoa(&ip_info.netmask));
			strcpy(gw, ip4addr_ntoa(&ip_info.gw));

			snprintf( (ip_info_json + strlen(ip_info_json)), JSON_IP_INFO_SIZE, ip_info_json_format,
					ip,
					netmask,
					gw,
					(int)update_reason_code);
		}
		else{
			/* notify in the json output the reason code why this was updated without a connection */
			snprintf( (ip_info_json + strlen(ip_info_json)), JSON_IP_INFO_SIZE, ip_info_json_format,
								"0",
								"0",
								"0",
								(int)update_reason_code);
		}
	}
	else{
		wifi_manager_clear_ip_info_json();
	}


}


void wifi_manager_clear_access_points_json(){
	strcpy(accessp_json, "[]\n");
}
void wifi_manager_generate_acess_points_json(){

	strcpy(accessp_json, "[");


	const char oneap_str[] = ",\"chan\":%d,\"rssi\":%d,\"auth\":%d}%c\n";

	/* stack buffer to hold on to one AP until it's copied over to accessp_json */
	char one_ap[JSON_ONE_APP_SIZE];

	for(int i=0; i<ap_num;i++){

		wifi_ap_record_t ap = accessp_records[i];

		/* ssid needs to be json escaped. To save on heap memory it's directly printed at the correct address */
		strcat(accessp_json, "{\"ssid\":");
		json_print_string( (unsigned char*)ap.ssid,  (unsigned char*)(accessp_json+strlen(accessp_json)) );

		/* print the rest of the json for this access point: no more string to escape */
		snprintf(one_ap, (size_t)JSON_ONE_APP_SIZE, oneap_str,
				ap.primary,
				ap.rssi,
				ap.authmode,
				i==ap_num-1?']':',');

		/* add it to the list */
		strcat(accessp_json, one_ap);
	}

}


bool wifi_manager_lock_json_buffer(TickType_t xTicksToWait){
	if(wifi_manager_json_mutex){
		if( xSemaphoreTake( wifi_manager_json_mutex, xTicksToWait ) == pdTRUE ) {
			return true;
		}
		else{
			return false;
		}
	}
	else{
		return false;
	}

}
void wifi_manager_unlock_json_buffer(){
	xSemaphoreGive( wifi_manager_json_mutex );
}

char* wifi_manager_get_ap_list_json(){
	return accessp_json;
}


esp_err_t wifi_manager_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {

    case SYSTEM_EVENT_AP_START:
    	xEventGroupSetBits(wifi_manager_event_group, WIFI_MANAGER_AP_STARTED);
		break;

    case SYSTEM_EVENT_AP_STACONNECTED:
		xEventGroupSetBits(wifi_manager_event_group, WIFI_MANAGER_AP_STA_CONNECTED_BIT);
		break;

    case SYSTEM_EVENT_AP_STADISCONNECTED:
    	xEventGroupClearBits(wifi_manager_event_group, WIFI_MANAGER_AP_STA_CONNECTED_BIT);
		break;

    case SYSTEM_EVENT_STA_START:
        break;

	case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_manager_event_group, WIFI_MANAGER_WIFI_CONNECTED_BIT);
        break;

	case SYSTEM_EVENT_STA_DISCONNECTED:
		xEventGroupSetBits(wifi_manager_event_group, WIFI_MANAGER_STA_DISCONNECT_BIT);
		xEventGroupClearBits(wifi_manager_event_group, WIFI_MANAGER_WIFI_CONNECTED_BIT);
        break;

	default:
        break;
    }
	return ESP_OK;
}






wifi_config_t* wifi_manager_get_wifi_sta_config(){
	return wifi_manager_config_sta;
}


void wifi_manager_connect_async(){
	/* in order to avoid a false positive on the front end app we need to quickly flush the ip json
	 * There'se a risk the front end sees an IP or a password error when in fact
	 * it's a remnant from a previous connection
	 */
	if(wifi_manager_lock_json_buffer( portMAX_DELAY )){
		wifi_manager_clear_ip_info_json();
		wifi_manager_unlock_json_buffer();
	}
	xEventGroupSetBits(wifi_manager_event_group, WIFI_MANAGER_REQUEST_STA_CONNECT_BIT);
}


char* wifi_manager_get_ip_info_json(){
	return ip_info_json;
}


void wifi_manager_destroy(){

	/* heap buffers */
	free(accessp_records);
	accessp_records = NULL;
	free(accessp_json);
	accessp_json = NULL;
	free(ip_info_json);
	ip_info_json = NULL;
	if(wifi_manager_config_sta){
		free(wifi_manager_config_sta);
		wifi_manager_config_sta = NULL;
	}

	/* RTOS objects */
	vSemaphoreDelete(wifi_manager_json_mutex);
	wifi_manager_json_mutex = NULL;
	vEventGroupDelete(wifi_manager_event_group);

	vTaskDelete(NULL);
}



void wifi_manager( void * pvParameters ){

	/* memory allocation of objects used by the task */
	wifi_manager_json_mutex = xSemaphoreCreateMutex();
	accessp_records = (wifi_ap_record_t*)malloc(sizeof(wifi_ap_record_t) * MAX_AP_NUM);
	accessp_json = (char*)malloc(MAX_AP_NUM * JSON_ONE_APP_SIZE + 4); //4 bytes for json encapsulation of "[\n" and "]\0"
	wifi_manager_clear_access_points_json();
	ip_info_json = (char*)malloc(sizeof(char) * JSON_IP_INFO_SIZE);
	wifi_manager_clear_ip_info_json();
	wifi_manager_config_sta = (wifi_config_t*)malloc(sizeof(wifi_config_t));
	memset(wifi_manager_config_sta, 0x00, sizeof(wifi_config_t));

	/* initialize the tcp stack */
	tcpip_adapter_init();

    /* event handler and event group for the wifi driver */
	wifi_manager_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_manager_event_handler, NULL));

    /* wifi scanner config */
	wifi_scan_config_t scan_config = {
		.ssid = 0,
		.bssid = 0,
		.channel = 0,
		.show_hidden = true
	};


	/* try to get access to previously saved wifi */
	if(wifi_manager_fetch_wifi_sta_config()){
#if WIFI_MANAGER_DEBUG
		printf("wifi_manager: saved wifi found on startup\n");
		/* request a connection */
		xEventGroupSetBits(wifi_manager_event_group, WIFI_MANAGER_REQUEST_STA_CONNECT_BIT);
#endif
	}

	/* start the softAP access point */
	/* stop DHCP */
	ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));

	/* assign a static IP to the network interface */
	tcpip_adapter_ip_info_t info;
	memset(&info, 0x00, sizeof(info));
	IP4_ADDR(&info.ip, 192, 168, 1, 1);
	IP4_ADDR(&info.gw, 192, 168, 1, 1);
	IP4_ADDR(&info.netmask, 255, 255, 255, 0);
	ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));

	/* start dhcp */
	ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));

	/* init wifi as station + access point */
	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

	// configure the softAP and start it */
	wifi_config_t ap_config = {
		.ap = {
			.ssid = AP_SSID,
			.password = AP_PASSWORD,
			.ssid_len = 0,
			.channel = AP_CHANNEL,
			.authmode = WIFI_AUTH_WPA2_PSK,
			.ssid_hidden = AP_SSID_HIDDEN,
			.max_connection = AP_MAX_CONNECTIONS,
			.beacon_interval = AP_BEACON_INTERVAL,
		},
	};
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
	ESP_ERROR_CHECK(esp_wifi_start());

#if WIFI_MANAGER_DEBUG
	printf("wifi_mamager: starting softAP with ssid %s\n", ap_config.ap.ssid);
#endif

	/* wait for access point to start */
	xEventGroupWaitBits(wifi_manager_event_group, WIFI_MANAGER_AP_STARTED, pdFALSE, pdTRUE, portMAX_DELAY );

#if WIFI_MANAGER_DEBUG
	printf("wifi_mamager: softAP started, starting http_server\n");
#endif
	http_server_set_event_start();


	EventBits_t uxBits;
	for(;;){

		/* actions that can trigger: request a connection, a scan, or a disconnection */
		uxBits = xEventGroupWaitBits(wifi_manager_event_group, WIFI_MANAGER_REQUEST_STA_CONNECT_BIT | WIFI_MANAGER_REQUEST_WIFI_SCAN | WIFI_MANAGER_REQUEST_WIFI_DISCONNECT, pdFALSE, pdFALSE, portMAX_DELAY );
		if(uxBits & WIFI_MANAGER_REQUEST_WIFI_DISCONNECT){
			/* user requested a disconnect, this will in effect disconnect the wifi but also erase NVS memory*/

			/*disconnect only if it was connected to begin with! */
			if( uxBits & WIFI_MANAGER_WIFI_CONNECTED_BIT ){
				xEventGroupClearBits(wifi_manager_event_group, WIFI_MANAGER_STA_DISCONNECT_BIT);
				ESP_ERROR_CHECK(esp_wifi_disconnect());

				/* wait until wifi disconnects. From experiments, it seems to take about 150ms to disconnect */
				xEventGroupWaitBits(wifi_manager_event_group, WIFI_MANAGER_STA_DISCONNECT_BIT, pdFALSE, pdTRUE, portMAX_DELAY );
			}
			xEventGroupClearBits(wifi_manager_event_group, WIFI_MANAGER_STA_DISCONNECT_BIT);

			/* erase configuration */
			if(wifi_manager_config_sta){
				memset(wifi_manager_config_sta, 0x00, sizeof(wifi_config_t));
			}

			/* save NVS memory */
			wifi_manager_save_sta_config();

			/* update JSON status */
			if(wifi_manager_lock_json_buffer( portMAX_DELAY )){
				wifi_manager_generate_ip_info_json(UPDATE_USER_DISCONNECT);
				wifi_manager_unlock_json_buffer();
			}
			else{
				/* Even if someone were to furiously refresh a web resource that needs the json mutex,
				 * it seems impossible that this thread cannot obtain the mutex. Abort here is reasonnable.
				 */
				abort();
			}

			/* finally: release the scan request bit */
			xEventGroupClearBits(wifi_manager_event_group, WIFI_MANAGER_REQUEST_WIFI_DISCONNECT);
		}
		if(uxBits & WIFI_MANAGER_REQUEST_STA_CONNECT_BIT){
			//someone requested a connection!

			/* first thing: if the esp32 is already connected to a access point: disconnect */
			if( (uxBits & WIFI_MANAGER_WIFI_CONNECTED_BIT) == (WIFI_MANAGER_WIFI_CONNECTED_BIT) ){

				xEventGroupClearBits(wifi_manager_event_group, WIFI_MANAGER_STA_DISCONNECT_BIT);
				ESP_ERROR_CHECK(esp_wifi_disconnect());

				/* wait until wifi disconnects. From experiments, it seems to take about 150ms to disconnect */
				xEventGroupWaitBits(wifi_manager_event_group, WIFI_MANAGER_STA_DISCONNECT_BIT, pdFALSE, pdTRUE, portMAX_DELAY );
			}

			/* set the new config and connect - reset the disconnect bit first as it is later tested */
			xEventGroupClearBits(wifi_manager_event_group, WIFI_MANAGER_STA_DISCONNECT_BIT);
			ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, wifi_manager_get_wifi_sta_config()));
			ESP_ERROR_CHECK(esp_wifi_connect());

			/* 2 scenarios here: connection is successful and SYSTEM_EVENT_STA_GOT_IP will be posted
			 * or it's a failure and we get a SYSTEM_EVENT_STA_DISCONNECTED with a reason code.
			 * Note that the reason code is not exploited. For all intent and purposes a failure is a failure.
			 */
			uxBits = xEventGroupWaitBits(wifi_manager_event_group, WIFI_MANAGER_WIFI_CONNECTED_BIT | WIFI_MANAGER_STA_DISCONNECT_BIT, pdFALSE, pdFALSE, portMAX_DELAY );

			if(uxBits & (WIFI_MANAGER_WIFI_CONNECTED_BIT | WIFI_MANAGER_STA_DISCONNECT_BIT)){

				/* Update the json regardless of connection status.
				 * If connection was succesful an IP will get assigned.
				 * If the connection attempt is failed we mark it as a failed connection attempt
				 * as it is important for the front end app to distinguish failed attempt to
				 * regular disconnects
				 */
				if(wifi_manager_lock_json_buffer( portMAX_DELAY )){

					/* only save the config if the connection was successful! */
					if(uxBits & WIFI_MANAGER_WIFI_CONNECTED_BIT){

						/* generate the connection info with success */
						wifi_manager_generate_ip_info_json( UPDATE_CONNECTION_OK );

						/* save wifi config in NVS */
						wifi_manager_save_sta_config();
					}
					else{

						/* failed attempt to connect regardles of the reason */
						wifi_manager_generate_ip_info_json( UPDATE_FAILED_ATTEMPT );

						/* otherwise: reset the config */
						memset(wifi_manager_config_sta, 0x00, sizeof(wifi_config_t));
					}
					wifi_manager_unlock_json_buffer();
				}
				else{
					/* Even if someone were to furiously refresh a web resource that needs the json mutex,
					 * it seems impossible that this thread cannot obtain the mutex. Abort here is reasonnable.
					 */
					abort();
				}
			}
			else{
				/* hit portMAX_DELAY limit ? */
				abort();
			}

			/* finally: release the connection request bit */
			xEventGroupClearBits(wifi_manager_event_group, WIFI_MANAGER_REQUEST_STA_CONNECT_BIT);
		}
		else if(uxBits & WIFI_MANAGER_REQUEST_WIFI_SCAN){

			/* safe guard against overflow */
			if(ap_num > MAX_AP_NUM) ap_num = MAX_AP_NUM;

			ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
			ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, accessp_records));

			/* make sure the http server isn't trying to access the list while it gets refreshed */
			if(wifi_manager_lock_json_buffer( ( TickType_t ) 20 )){
				wifi_manager_generate_acess_points_json();
				wifi_manager_unlock_json_buffer();
			}
			else{
#if WIFI_MANAGER_DEBUG
				printf("wifi_manager: could not get access to json mutex in wifi_scan\n");
#endif
			}

			/* finally: release the scan request bit */
			xEventGroupClearBits(wifi_manager_event_group, WIFI_MANAGER_REQUEST_WIFI_SCAN);
		}
	} /* for(;;) */
	vTaskDelay( (TickType_t)10);
} /*void wifi_manager*/
