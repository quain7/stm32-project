#ifndef FATFS_SD_H
#define FATFS_SD_H

#include "stm32f1xx_hal.h"
#include "diskio.h"

#ifndef SD_CS_PORT
#define SD_CS_PORT GPIOA
#define SD_CS_PIN  GPIO_PIN_4
#endif

DSTATUS FATFS_SD_DiskInitialize(BYTE pdrv);
DSTATUS FATFS_SD_DiskStatus(BYTE pdrv);
DRESULT FATFS_SD_DiskRead(BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT FATFS_SD_DiskWrite(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT FATFS_SD_DiskIoctl(BYTE pdrv, BYTE cmd, void* buff);

#endif
