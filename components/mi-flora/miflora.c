#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "miflora.h"

#define mutex_lock(x)       while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS);
#define mutex_unlock(x)     xSemaphoreGive(x)
#define mutex_create()      xSemaphoreCreateMutex()
#define mutex_destroy(x)    vQueueDelete(x)


static const char *TAG = "MI_FLORA";

static const int MIFLORA_APPID = 0;
static esp_gatt_if_t miflora_gattcif = 0;
static uint16_t miflora_connid = 0;
static char miflora_data[32];
static char miflora_len = 0;

static miflora_device_list_t *scan_list = NULL;
static int num_scan_devices = 0;
static int max_scan_devices = 0;
static char *device_filter;

static EventGroupHandle_t mi_event;
static xSemaphoreHandle flora_lock;
static const int EVT_READY = BIT0;
static const int EVT_OPEN = BIT1;
static const int EVT_WRITE = BIT2;
static const int EVT_READ = BIT3;
static const int EVT_CLOSE = BIT4;
static const int EVT_SCAN = BIT5;

#define mi_event_clear(bit) xEventGroupClearBits(mi_event, bit)
#define mi_event_set(bit) xEventGroupSetBits(mi_event, bit)
#define mi_event_wait_done(bit, tick_to_wait) (xEventGroupWaitBits(mi_event, bit, false, true, tick_to_wait) & bit)



typedef enum {
	READ_VERSION_BATTERY = 0x38,
	READ_NAME = 0x03,
	READ_SENSOR_DATA = 0x35,
	WRITE_MODE_CHANGE = 0x33,
} miflora_handle_t;


static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
	ESP_LOGD(TAG, "esp_gattc_cb = %d, gattc_if = %d", (int)event, gattc_if);
	if (event == ESP_GATTC_REG_EVT) {
		if (param->reg.status == ESP_GATT_OK) {
			if (param->reg.app_id == MIFLORA_APPID) {
				ESP_LOGD(TAG, "Reg app_id %04x, status %d", param->reg.app_id, param->reg.status);
				miflora_gattcif = gattc_if;
				mi_event_set(EVT_READY);
			}

		} else {
			ESP_LOGD(TAG, "Reg app failed, app_id %04x, status %d",
			         param->reg.app_id,
			         param->reg.status);
			return;
		}
	}

	if (gattc_if == ESP_GATT_IF_NONE || gattc_if == miflora_gattcif) {
		esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;
		switch ((int)event) {
			case ESP_GATTC_READ_CHAR_EVT:
				ESP_LOGD(TAG, "REG_EVT");
				if (param->read.value_len >= 31) {
					param->read.value_len = 31;
				}
				memcpy(miflora_data, param->read.value, param->read.value_len);
				miflora_data[param->read.value_len] = 0;
				miflora_len = param->read.value_len;
				// printf("%s\r\n", miflora_data);
				// esp_log_buffer_hex(TAG, param->read.value, param->read.value_len);
				mi_event_set(EVT_READ);
				break;
			case ESP_GATTC_CONNECT_EVT:
				ESP_LOGD(TAG, "ESP_GATTC_CONNECT_EVT");
				break;
			case ESP_GATTC_WRITE_CHAR_EVT:
				if (p_data->write.status != ESP_GATT_OK) {
					ESP_LOGE(TAG, "write char failed, error status = %x", p_data->write.status);
				} else {
					mi_event_set(EVT_WRITE);
					ESP_LOGD(TAG, "write char success");
				}
				break;
			case ESP_GATTC_REG_EVT:
				ESP_LOGD(TAG, "ESP_GATTC_REG_EVT");
				break;
			case ESP_GATTC_OPEN_EVT:
				if (p_data->open.status != ESP_GATT_OK) {
					ESP_LOGE(TAG, "connect device failed, status %d", p_data->open.status);
					break;
				}

				ESP_LOGD(TAG, "ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d, mtu %d", p_data->open.conn_id, gattc_if, p_data->open.status, p_data->open.mtu);
				// ESP_LOGI(TAG, "REMOTE BDA:");
				// esp_log_buffer_hex(TAG, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
				miflora_connid = p_data->open.conn_id;
				mi_event_set(EVT_OPEN);
				break;
			case ESP_GATTC_DISCONNECT_EVT:
				ESP_LOGD(TAG, "ESP_GATTC_DISCONNECT_EVT");
				mi_event_set(EVT_CLOSE);
				break;
		}
	}
}
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	ESP_LOGD(TAG, "esp_gap_cb = %d", (int)event);
	uint8_t *adv_name = NULL;
	esp_ble_gap_cb_param_t *scan_result;
	uint8_t adv_name_len = 0;
	switch ((int)event) {
		case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
			//scan start complete event to indicate scan start successfully or failed
			if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
				ESP_LOGD(TAG, "Scan start success");
			} else {
				ESP_LOGE(TAG, "Scan start failed");
			}
			break;
		case ESP_GAP_BLE_SCAN_RESULT_EVT:
			scan_result = (esp_ble_gap_cb_param_t *)param;
			switch (scan_result->scan_rst.search_evt) {
				case ESP_GAP_SEARCH_INQ_RES_EVT:
					esp_log_buffer_hex(TAG, scan_result->scan_rst.bda, 6);
					ESP_LOGD(TAG, "Searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
					adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
					                                    ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
					ESP_LOGD(TAG, "Searched Device Name Len %d", adv_name_len);
					// esp_log_buffer_char(TAG, adv_name, adv_name_len);
					ESP_LOGD(TAG, "num_scan_devices = %d, max_scan_devices = %d", num_scan_devices, max_scan_devices);
					if (num_scan_devices < max_scan_devices) {
						miflora_device_t *mi_device;

						STAILQ_FOREACH(mi_device, scan_list, next) {
						    if (memcmp(mi_device->bda, scan_result->scan_rst.bda, sizeof(esp_bd_addr_t)) == 0) {
						    	return;
						    }
						}
                        bool can_add = false;
						if (device_filter) {
                            if (adv_name_len > 0 && strcmp((char*)device_filter, (char*)adv_name) == 0) {
                                can_add = true;
                            }
						} else {
                            can_add = true;
                        }
						if (can_add) {
                            if (adv_name) {
                                ESP_LOGW(TAG, "Add device %s", adv_name);
                            }

							mi_device = calloc(1, sizeof(miflora_device_t));
							assert(mi_device);
							memcpy(mi_device->bda, scan_result->scan_rst.bda, 6);
                            mi_device->rssi = scan_result->scan_rst.rssi;
							STAILQ_INSERT_TAIL(scan_list, mi_device, next);
							num_scan_devices ++;
						}

					} else {
						esp_ble_gap_stop_scanning();
					}
					break;
				case ESP_GAP_SEARCH_INQ_CMPL_EVT:
					break;
				default:
					break;
			}
			break;

		case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
			if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
				ESP_LOGE(TAG, "Scan stop failed");
				break;
			}
			mi_event_set(EVT_SCAN);
			ESP_LOGD(TAG, "Stop scan successfully");
			break;

	}
}
esp_err_t miflora_init()
{
	esp_err_t ret;

	mi_event = xEventGroupCreate();
	mi_event_clear(EVT_READY);
    flora_lock = mutex_create();

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(TAG, "initialize controller failed");
		return ret;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret) {
		ESP_LOGE(TAG, "enable controller failed");
		return ret;
	}

	ret = esp_bluedroid_init();
	if (ret) {
		ESP_LOGE(TAG, "init bluetooth failed");
		return ret;
	}

	ret = esp_bluedroid_enable();
	if (ret) {
		ESP_LOGE(TAG, "enable bluetooth failed");
		return ret;
	}

	//register the callback function to the gattc module
	ret = esp_ble_gattc_register_callback(esp_gattc_cb);
	if (ret) {
		ESP_LOGE(TAG, "gattc register error, error code = %x", ret);
		return ret;
	}

	//register the  callback function to the gap module
	ret = esp_ble_gap_register_callback(esp_gap_cb);
	if (ret) {
		ESP_LOGE(TAG, "gap register error, error code = %x", ret);
		return ret;
	}


	ret = esp_ble_gattc_app_register(MIFLORA_APPID);
	if (ret) {
		ESP_LOGE(TAG, "gattc app register error, error code = %x", ret);
		return ret;
	}

	ret = esp_ble_gatt_set_local_mtu(200);
	if (ret) {
		ESP_LOGE(TAG, "set local MTU failed, error code = %x", ret);
		return ret;
	}

	return ret;
}

esp_err_t miflora_open(esp_bd_addr_t bda, TickType_t tick_to_wait)
{
	if (mi_event_wait_done(EVT_READY, tick_to_wait) == 0) {
		ESP_LOGE(TAG, "error open ble");
	}
	mi_event_clear(EVT_OPEN);
	if (esp_ble_gattc_open(miflora_gattcif, bda, BLE_ADDR_TYPE_PUBLIC, true) != ESP_OK) {
		ESP_LOGE(TAG, "error open gattc");
		return ESP_FAIL;
	}
	if (mi_event_wait_done(EVT_OPEN, tick_to_wait) == 0) {
		ESP_LOGE(TAG, "open timeout");
		return ESP_FAIL;
	}
	return ESP_OK;
}

esp_err_t miflora_close(TickType_t tick_to_wait)
{
	mi_event_clear(EVT_CLOSE);
	if (esp_ble_gattc_close(miflora_gattcif, miflora_connid) != ESP_OK) {
		ESP_LOGE(TAG, "error close");
		return ESP_FAIL;
	}
	if (mi_event_wait_done(EVT_CLOSE, tick_to_wait) == 0) {
		ESP_LOGE(TAG, "close timeout");
		return ESP_FAIL;
	}
	return ESP_OK;
}
esp_err_t miflora_read(esp_bd_addr_t bda, miflora_handle_t handle, TickType_t tick_to_wait)
{
	mi_event_clear(EVT_READ);
	if (esp_ble_gattc_read_char(miflora_gattcif, miflora_connid, handle, ESP_GATT_AUTH_REQ_NONE) != ESP_OK) {
		ESP_LOGE(TAG, "error read");
	}
	if (mi_event_wait_done(EVT_READ, tick_to_wait) == 0) {
		ESP_LOGE(TAG, "read timeout");
		return ESP_FAIL;
	}
	return ESP_OK;
}

esp_err_t miflora_write_magic(esp_bd_addr_t bda, TickType_t tick_to_wait)
{
	mi_event_clear(EVT_WRITE);
	unsigned char magic[] = { 0xA0, 0x1F };
	if (esp_ble_gattc_write_char(miflora_gattcif,
	                             miflora_connid,
	                             WRITE_MODE_CHANGE,
	                             2,
	                             magic,
	                             ESP_GATT_WRITE_TYPE_RSP,
	                             ESP_GATT_AUTH_REQ_NONE) != ESP_OK) {
		ESP_LOGE(TAG, "write faile");
		return ESP_FAIL;
	}
	if (mi_event_wait_done(EVT_WRITE, tick_to_wait) == 0) {
		ESP_LOGE(TAG, "write timeout");
		return ESP_FAIL;
	}
	return ESP_OK;
}

esp_err_t miflora_read_sensor(esp_bd_addr_t bda, miflora_sensor_data_t *data, TickType_t tick_to_wait)
{
    mutex_lock(flora_lock);
	if (miflora_open(bda, tick_to_wait) != ESP_OK) {
		ESP_LOGE(TAG, "Error open");
        mutex_unlock(flora_lock);
		return ESP_FAIL;
	}
	if (miflora_write_magic(bda, tick_to_wait) != ESP_OK) {
		ESP_LOGE(TAG, "Error write magic");
        mutex_unlock(flora_lock);
		return ESP_FAIL;
	}
	if (miflora_read(bda, READ_VERSION_BATTERY, tick_to_wait) != ESP_OK) {
		ESP_LOGE(TAG, "Error read name");
        mutex_unlock(flora_lock);
		return ESP_FAIL;
	}
	ESP_LOGD(TAG, "===Battery & firmware");
	int bat_level = miflora_data[0];
	char *ver = (char *)&miflora_data[2];
	ver[6] = 0;
	ESP_LOGW(TAG, "Version = %s", ver);
	esp_log_buffer_hex(TAG, miflora_data, miflora_len);
	if (miflora_read(bda, READ_SENSOR_DATA, tick_to_wait) != ESP_OK) {
		ESP_LOGE(TAG, "Error read sensor");
        mutex_unlock(flora_lock);
		return ESP_FAIL;
	}
	ESP_LOGD(TAG, "===Sensor data");
	esp_log_buffer_hex(TAG, miflora_data, miflora_len);
	uint16_t *temp = (uint16_t *)&miflora_data[0];
	uint16_t *light = (uint16_t *)&miflora_data[3];
	uint16_t *fertility = (uint16_t *)&miflora_data[8];
	uint8_t moisture = miflora_data[7];
	double temp_double = (double) * temp / 10.0;
	ESP_LOGW(TAG, "Temp = %f, light=%d, fertility=%dµS/cm, moisture=%d, bat_level=%d", temp_double, *light, *fertility, moisture, bat_level);
	if (miflora_close(tick_to_wait) != ESP_OK) {
		ESP_LOGE(TAG, "Error close");
        mutex_unlock(flora_lock);
		return ESP_FAIL;
	}
    data->temperature = temp_double;
    data->fertility = *fertility;
    data->battery = bat_level;
    data->moisture = moisture;
    data->light = *light;
    mutex_unlock(flora_lock);
	return ESP_OK;
}

miflora_device_list_t *miflora_scan(int max_devices, const char *filter_name, TickType_t tick_to_wait)
{
	int duration = tick_to_wait / configTICK_RATE_HZ;
	if (duration <= 0) {
		ESP_LOGE(TAG, "duration = %d", duration);
		return NULL;
	}
	miflora_cleanup(scan_list);
	ESP_LOGD(TAG, "begin scan, duration %ds", duration);
	mi_event_clear(EVT_SCAN);
	max_scan_devices = max_devices;
	num_scan_devices = 0;
	device_filter = (char*)filter_name;
	scan_list = calloc(1, sizeof(miflora_device_list_t));
	assert(scan_list);
	STAILQ_INIT(scan_list);
	esp_ble_gap_start_scanning(duration);

	if (mi_event_wait_done(EVT_SCAN, tick_to_wait) == 0) {
		ESP_LOGE(TAG, "scan timeout");
		return scan_list;
	}
	return scan_list;
}

esp_err_t miflora_cleanup(miflora_device_list_t *list)
{
	num_scan_devices = 0;
	max_scan_devices = 0;
	device_filter = NULL;
	if (scan_list == NULL) {
		ESP_LOGD(TAG, "scan_list == NULL");
		return ESP_FAIL;
	}
	miflora_device_t *item = STAILQ_FIRST(scan_list), *tmp;
	while (item != NULL) {
	    tmp = STAILQ_NEXT(item, next);
	    free(item);
	    item = tmp;
	}
	free(scan_list);
	scan_list = NULL;
	return ESP_OK;
}
