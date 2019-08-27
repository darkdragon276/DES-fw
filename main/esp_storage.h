/*
 * Nanochip Technology JSC License
 *
 * Copyright (c) 2018 <Nanochip Technology JSC License>
 *
 */

#ifndef _ESP_STORAGE_H_
#define _ESP_STORAGE_H_

#include "esp_err.h"
#include <time.h>
#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct esp_storage_t* esp_storage_handle_t;

/**
 * The configuration loaded callback function, this function will be called when data from nvs loaded successful
 * The buffer pointer and loaded_len is actualy data read from NVS
 */

typedef esp_err_t (*storage_unpack_func)(void *context, char *buffer, int loaded_len);

/**
 * The configuration prepare saving function, this function will be called before save data to nvs,
 * we must provide data to buffer, not larger than max_buffer_size and return actualy data
 */
typedef int (*storage_pack_func)(void *context, char *buffer, int max_buffer_size);

typedef struct {
    const char* namespace;
    int         buffer_size;
} esp_storage_config_t;

esp_storage_handle_t esp_storage_init(esp_storage_config_t *config);
esp_err_t esp_storage_add(esp_storage_handle_t storage, const char *key, storage_unpack_func unpack, storage_pack_func pack, void *context);
esp_err_t esp_storage_remove(esp_storage_handle_t storage, const char *key);
esp_err_t esp_storage_load(esp_storage_handle_t storage, const char* key);
esp_err_t esp_storage_save(esp_storage_handle_t storage, const char* key);
esp_err_t esp_storage_destroy(esp_storage_handle_t storage);


#ifdef __cplusplus
}
#endif

#endif
