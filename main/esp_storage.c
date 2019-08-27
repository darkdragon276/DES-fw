/*
 * This file is subject to the terms of the Nanochip License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *                             ./LICENSE
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/queue.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_storage.h"

static const char *TAG = "ESP_STORAGE";

typedef struct esp_storage_item {
    storage_unpack_func         unpack;
    storage_pack_func           pack;
    char*                           key;
    void*                           context;
    STAILQ_ENTRY(esp_storage_item)  next;
} esp_storage_item_t;

struct esp_storage_t {
    char*                                           namespace;
    int                                             buffer_size;
    char*                                           buffer;
    SemaphoreHandle_t                               lock;
    STAILQ_HEAD(esp_storage_list, esp_storage_item) list;
};

#define DEFAULT_STORAGE_BUFFER (1024)

#define _mutex_lock(x)       while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS);
#define _mutex_unlock(x)     xSemaphoreGive(x)
#define _mutex_create()      xSemaphoreCreateMutex()
#define _mutex_destroy(x)    vSemaphoreDelete(x)

esp_storage_handle_t esp_storage_init(esp_storage_config_t *config)
{
    if (config->namespace == NULL) {
        ESP_LOGE(TAG, "namespace required");
        return NULL;
    }
    esp_storage_handle_t storage = calloc(1, sizeof(struct esp_storage_t));
    if (storage == NULL) {
        ESP_LOGE(TAG, "Error calloc memory");
        return NULL;
    }
    storage->buffer_size = config->buffer_size > 0 ? config->buffer_size : DEFAULT_STORAGE_BUFFER;
    storage->buffer = malloc(storage->buffer_size);
    if (storage->buffer == NULL) {
        ESP_LOGE(TAG, "Error calloc buffer");
        goto _storage_init_failed;
    }
    storage->namespace = strdup(config->namespace);
    if (storage->namespace == NULL) {
        ESP_LOGE(TAG, "Error calloc namespace");
        goto _storage_init_failed;
    }
    storage->lock = _mutex_create();
    if (storage->lock == NULL) {
        ESP_LOGE(TAG, "Error create lock");
        goto _storage_init_failed;
    }
    STAILQ_INIT(&storage->list);
    return storage;
_storage_init_failed:
    free(storage->buffer);
    free(storage->namespace);
    if (storage->lock) {
        _mutex_destroy(storage->lock);
    }
    free(storage);
    return NULL;
}

esp_err_t esp_storage_destroy(esp_storage_handle_t storage)
{
    if (storage == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_storage_item_t *item = STAILQ_FIRST(&storage->list);
    esp_storage_item_t *tmp;
    while (item != NULL) {
        tmp = STAILQ_NEXT(item, next);
        free(item->key);
        free(item);
        item = tmp;
    }
    free(storage->buffer);
    free(storage->namespace);
    if (storage->lock) {
        _mutex_destroy(storage->lock);
    }
    storage = NULL;
    return ESP_OK;
}

static esp_storage_item_t* esp_storage_get_item_by_key(esp_storage_handle_t storage, const char *key)
{
    esp_storage_item_t *item;
    STAILQ_FOREACH(item, &storage->list, next) {
        if (strcasecmp(item->key, key) == 0) {
            return item;
        }
    }
    return NULL;
}

esp_err_t esp_storage_add(esp_storage_handle_t storage, const char *key, storage_unpack_func unpack, storage_pack_func pack, void *context)
{
    if (storage == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_storage_item_t *item = esp_storage_get_item_by_key(storage, key);
    if (item != NULL) {
        return ESP_FAIL;
    }
    item = calloc(1, sizeof(esp_storage_item_t));
    if (item == NULL) {
        return ESP_ERR_NO_MEM;
    }
    item->key = strdup(key);
    if (item->key == NULL) {
        free(item);
        return ESP_ERR_NO_MEM;
    }
    item->unpack = unpack;
    item->pack = pack;
    item->context = context;
    STAILQ_INSERT_TAIL(&storage->list, item, next);
    return ESP_OK;
}

esp_err_t esp_storage_remove(esp_storage_handle_t storage, const char *key)
{
    if (storage == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_storage_item_t *item = esp_storage_get_item_by_key(storage, key);
    if (item == NULL) {
        return ESP_FAIL;
    }
    // STAILQ_REMOVE(&storage->list, item, esp_storage_item_t, next);
    free(item->key);
    free(item);
    return ESP_OK;
}

esp_err_t esp_storage_save(esp_storage_handle_t storage, const char *key)
{
    nvs_handle _nvs_handle;
    esp_err_t err;

    if (storage == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_storage_item_t *item = esp_storage_get_item_by_key(storage, key);

    if (item == NULL) {
        return ESP_FAIL;
    }

    _mutex_lock(storage->lock);
    int write_size = item->pack(item->context, storage->buffer, storage->buffer_size);
    _mutex_unlock(storage->lock);
    if (write_size <= 0) {
        ESP_LOGE(TAG, "Error save configuration");
        return ESP_FAIL;
    }
    // Open
    err = nvs_open(storage->namespace, NVS_READWRITE, &_nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(_nvs_handle, item->key, storage->buffer, write_size);
    if (err != ESP_OK) {
        goto _save_config_failed;
    }

    err = nvs_commit(_nvs_handle);
    if (err != ESP_OK) {
        goto _save_config_failed;
    }

_save_config_failed:
    // Close
    nvs_close(_nvs_handle);
    return err;
}

esp_err_t esp_storage_load(esp_storage_handle_t storage, const char* key)
{
    nvs_handle _nvs_handle;
    esp_err_t err;
    if (storage == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_storage_item_t *item = esp_storage_get_item_by_key(storage, key);
    if (item == NULL) {
        return ESP_FAIL;
    }

    if (item->unpack == NULL) {
        return ESP_ERR_INVALID_STATE;
    }


    // Open
    err = nvs_open(storage->namespace, NVS_READWRITE, &_nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    size_t read_size = storage->buffer_size;
    _mutex_lock(storage->lock);
    err = nvs_get_blob(_nvs_handle, item->key, storage->buffer, &read_size);
    _mutex_unlock(storage->lock);
    nvs_close(_nvs_handle);
    if (err != ESP_OK || read_size == 0) {
        return err;
    }

    err = item->unpack(item->context, storage->buffer, read_size);

    if (err != ESP_OK) {
        return err;
    }
    // Close

    return ESP_OK;
}
