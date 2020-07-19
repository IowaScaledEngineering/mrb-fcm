/*-----------------------------------------------------------------------
/  Low level disk interface modlue include file   (C)ChaN, 2016
/-----------------------------------------------------------------------*/

#ifndef _MMC_DEFINED
#define _MMC_DEFINED

#include "ff.h"
#include "diskio.h"

#ifdef __cplusplus
extern "C" {
#endif


#define SDCARD_MOSI_PIN      PB5
#define SDCARD_MISO_PIN      PB6
#define SDCARD_SCLK_PIN      PB7


#define SDCARD_INSERTED_PIN  PD5
#define SDCARD_CS_PIN        PD7



/*---------------------------------------*/
/* Prototypes for disk control functions */

DSTATUS mmc_disk_initialize (void);
DSTATUS mmc_disk_status (void);
DRESULT mmc_disk_read (BYTE* buff, LBA_t sector, UINT count);
DRESULT mmc_disk_write (const BYTE* buff, LBA_t sector, UINT count);
DRESULT mmc_disk_ioctl (BYTE cmd, void* buff);
void mmc_disk_timerproc (void);

#ifdef __cplusplus
}
#endif

#endif

