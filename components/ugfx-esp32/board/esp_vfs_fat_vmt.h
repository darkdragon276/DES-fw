#ifndef _ESP_VFS_VMT_H
#define _ESP_VFS_VMT_H
#include "esp_vfs_fat.h"
bool_t espVfsMount(const char* drive);
bool_t espVfsUnmount(const char* drive);
bool_t espIsMounted();
#endif
