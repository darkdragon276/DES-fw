/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.org/license.html
 */

/********************************************************
 * The espVfs file-system
 ********************************************************/

#include "gfx.h"

#if GFX_USE_GFILE && GFILE_NEED_USERFS

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/dirent.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "../../../src/gfile/gfile_fs.h"

/********************************************************
 * The FAT file-system VMT
 ********************************************************/

static bool_t espVfsDel(const char* fname);
static bool_t espVfsExists(const char* fname);
static long int espVfsFileSize(const char* fname);
static bool_t espVfsRename(const char* oldname, const char* newname);
static bool_t espVfsOpen(GFILE* f, const char* fname);
static void espVfsClose(GFILE* f);
static int espVfsRead(GFILE* f, void* buf, int size);
static int espVfsWrite(GFILE* f, const void* buf, int size);
static bool_t espVfsSetPos(GFILE* f, long int pos);
static long int espVfsGetSize(GFILE* f);
static bool_t espVfsEOF(GFILE* f);
bool_t espVfsMount(const char* drive);
bool_t espVfsUnmount(const char* drive);
static bool_t espVfsSync(GFILE* f);
#if GFILE_NEED_FILELISTS && _FS_MINIMIZE <= 1
    static gfileList *espVfsFlOpen(const char *path, bool_t dirs);
    static const char *espVfsFlRead(gfileList *pfl);
    static void espVfsFlClose(gfileList *pfl);
#endif

const GFILEVMT FsUSERVMT = {
    GFSFLG_WRITEABLE | GFSFLG_SEEKABLE,
    'V',
    espVfsDel,
    espVfsExists,
    espVfsFileSize,
    espVfsRename,
    espVfsOpen,
    espVfsClose,
    espVfsRead,
    espVfsWrite,
    espVfsSetPos,
    espVfsGetSize,
    espVfsEOF,
    espVfsMount, espVfsUnmount, espVfsSync,

    #if GFILE_NEED_FILELISTS
        #if _FS_MINIMIZE <= 1
            NULL, NULL, NULL
            espVfsFlOpen, espVfsFlRead, espVfsFlClose
        #else
            0, 0, 0
        #endif
    #endif
};

// // Our directory list structure
typedef struct espVfsList {
    gfileList   fl;                 // This must be the first element.
    DIR         dir;
    FILINFO     fno;
    #if _USE_LFN
        char    lfn[_MAX_LFN + 1];   /* Buffer to store the LFN */
    #endif
} espVfsList;

// // optimize these later on. Use an array to have multiple espVfs
static bool_t esp_vfs_mounted = FALSE;
static FATFS espvfs_fs;

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   4
const char *VFS_TAG = "VFS";


FRESULT esp_vfs_init(const char *driver)
{
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = VSPI_HOST;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = PIN_NUM_MISO;
    slot_config.gpio_mosi = PIN_NUM_MOSI;
    slot_config.gpio_sck  = PIN_NUM_CLK;
    slot_config.gpio_cs   = PIN_NUM_CS;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5
    };

    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(driver, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(VFS_TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set format_if_mount_failed = true.");
        } else {
            ESP_LOGE(VFS_TAG, "Failed to initialize the card (%d). "
                "Make sure SD card lines have pull-up resistors in place.", ret);
        }
        return FR_DISK_ERR;
    }
    sdmmc_card_print_info(stdout, card);
    return FR_OK;
}

static BYTE espVfs_flags2mode(GFILE* f)
{
    BYTE mode = 0;

    if (f->flags & GFILEFLG_READ)
        mode |= FA_READ;
    if (f->flags & GFILEFLG_WRITE)
        mode |= FA_WRITE;
    if (f->flags & GFILEFLG_APPEND)
        mode |= FA_OPEN_APPEND;
    if (f->flags & GFILEFLG_TRUNC)
        mode |= FA_CREATE_ALWAYS;

    /* ToDo - Complete */
    return mode;
}

static bool_t espVfsDel(const char* fname)
{
    FRESULT ferr;

    ferr = f_unlink( (const TCHAR*)fname );
    if (ferr != FR_OK)
        return FALSE;

    return TRUE;
}

static bool_t espVfsExists(const char* fname)
{
    FRESULT ferr;
    FILINFO fno;

    ferr = f_stat( (const TCHAR*)fname, &fno);
    if (ferr != FR_OK)
        return FALSE;

    return TRUE;
}

static long int espVfsFileSize(const char* fname)
{
    FRESULT ferr;
    FILINFO fno;

    ferr = f_stat( (const TCHAR*)fname, &fno );
    if (ferr != FR_OK)
        return 0;

    return (long int)fno.fsize;
}

static bool_t espVfsRename(const char* oldname, const char* newname)
{
    FRESULT ferr;

    ferr = f_rename( (const TCHAR*)oldname, (const TCHAR*)newname );
    if (ferr != FR_OK)
        return FALSE;

    return TRUE;
}

static bool_t espVfsOpen(GFILE* f, const char* fname)
{
    FIL* fd;

    #if !GFILE_NEED_NOAUTOMOUNT
        if (!esp_vfs_mounted && !espVfsMount("/sdcard"))
            return FALSE;
    #endif

    if (!(fd = gfxAlloc(sizeof(FIL))))
        return FALSE;

    if (f_open(fd, fname, espVfs_flags2mode(f)) != FR_OK) {
        gfxFree(fd);
        f->obj = 0;

        return FALSE;
    }

    f->obj = (void*)fd;

    #if !GFILE_NEED_NOAUTOSYNC
        // no need to sync when not opening for write
        if (f->flags & GFILEFLG_WRITE) {
            f_sync( (FIL*)f->obj );
        }
    #endif

    return TRUE;
}

static void espVfsClose(GFILE* f)
{
    if ((FIL*)f->obj != 0) {
        f_close( (FIL*)f->obj );
        gfxFree( (FIL*)f->obj );
    }
}

static int espVfsRead(GFILE* f, void* buf, int size)
{
    int br;

    f_read( (FIL*)f->obj, buf, size, (UINT*)&br);
    return br;
}

static int espVfsWrite(GFILE* f, const void* buf, int size)
{
    int wr;

    f_write( (FIL*)f->obj, buf, size, (UINT*)&wr);
    #if !GFILE_NEED_NOAUTOSYNC
        f_sync( (FIL*)f->obj );
    #endif

    return wr;
}

static bool_t espVfsSetPos(GFILE* f, long int pos)
{
    FRESULT ferr;

    ferr = f_lseek( (FIL*)f->obj, (DWORD)pos );
    if (ferr != FR_OK)
        return FALSE;

    return TRUE;
}

static long int espVfsGetSize(GFILE* f)
{
    return (long int)f_size( (FIL*)f->obj );
}

static bool_t espVfsEOF(GFILE* f)
{
    if ( f_eof( (FIL*)f->obj ) != 0)
        return TRUE;
    else
        return FALSE;
}

bool_t espIsMounted()
{
    return esp_vfs_mounted;
}
bool_t espVfsMount(const char* drive)
{
    FRESULT ferr;

    if (!esp_vfs_mounted) {

        ferr = esp_vfs_init(drive);
        if (ferr !=  FR_OK)
            return FALSE;
        esp_vfs_mounted = TRUE;
        return TRUE;
    }

    return esp_vfs_mounted;
}

bool_t espVfsUnmount(const char* drive)
{
    (void)drive;

    if (esp_vfs_mounted) {
        // espVfs does not provide an unmount routine.
        esp_vfs_mounted = FALSE;
        esp_vfs_fat_sdmmc_unmount();
        return TRUE;
    }

    return FALSE;
}

static bool_t espVfsSync(GFILE *f)
{
    FRESULT ferr;

    ferr = f_sync( (FIL*)f->obj );
    if (ferr != FR_OK) {
        return FALSE;
    }

    return TRUE;
}

#if GFILE_NEED_FILELISTS && _FS_MINIMIZE <= 1
    static gfileList *espVfsFlOpen(const char *path, bool_t dirs) {
        espVfsList   *p;
        (void) dirs;

        if (!(p = gfxAlloc(sizeof(espVfsList))))
            return 0;

        if (f_opendir(&p->dir, path) != FR_OK) {
            gfxFree(p);
            return 0;
        }
        return &p->fl;
    }

    static const char *espVfsFlRead(gfileList *pfl) {
        #define ffl     ((espVfsList *)pfl)

        while(1) {
            #if _USE_LFN
                ffl->fno.lfname = ffl->lfn;
                ffl->fno.lfsize = sizeof(ffl->lfn);
            #endif

            // Read the next entry
            if (f_readdir(&ffl->dir, &ffl->fno) != FR_OK || !ffl->fno.fname[0])
                return 0;

            /* Ignore dot entries */
            if (ffl->fno.fname[0] == '.') continue;

            /* Is it a directory */
            if (ffl->fl.dirs) {
                if ((ffl->fno.fattrib & AM_DIR))
                    break;
            } else {
                if (!(ffl->fno.fattrib & AM_DIR))
                    break;
            }
        }

        #if _USE_LFN
            return ffl->fno.lfname[0] ? ffl->fno.lfname : ffl->fno.fname;
        #else
            return ffl->fno.fname;
        #endif
        #undef ffl
    }

    static void espVfsFlClose(gfileList *pfl) {
        f_closedir(&((espVfsList *)pfl)->dir);
        gfxFree(pfl);
    }

#endif

#endif //GFX_USE_GFILE && GFILE_NEED_espVfs

