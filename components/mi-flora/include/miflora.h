#ifndef _MIFLORA_H_
#define _MIFLORA_H_

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "rom/queue.h"
#include "esp_bt_defs.h"
#include "esp_gatt_defs.h"
#include "rom/queue.h"

typedef struct {
	double temperature;
	int light;
	int moisture;
	int fertility;
	int battery;
    bool valid;
} miflora_sensor_data_t;

typedef struct miflora_device {
	STAILQ_ENTRY(miflora_device) next;
	esp_bd_addr_t bda;
    int rssi;
    miflora_sensor_data_t data;
} miflora_device_t;

typedef STAILQ_HEAD(miflora_device_list, miflora_device) miflora_device_list_t;


esp_err_t miflora_init();
esp_err_t miflora_read_sensor(esp_bd_addr_t bda, miflora_sensor_data_t *data, TickType_t tick_to_wait);
miflora_device_list_t *miflora_scan(int max_devices, const char *filter_name, TickType_t tick_to_wait);
esp_err_t miflora_cleanup(miflora_device_list_t *list);
#endif
