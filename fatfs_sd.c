#include "fatfs_sd.h"
#include "stm32f1xx_hal.h"

extern SPI_HandleTypeDef hspi1;

#define SD_CS_LOW()     HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET)

/* Команди SD-картки */
#define CMD0   (0x40+0)     /* GO_IDLE_STATE */
#define CMD1   (0x40+1)     /* SEND_OP_COND */
#define CMD8   (0x40+8)     /* SEND_IF_COND */
#define CMD9   (0x40+9)     /* SEND_CSD */
#define CMD12  (0x40+12)    /* STOP_TRANSMISSION */
#define CMD16  (0x40+16)    /* SET_BLOCKLEN */
#define CMD17  (0x40+17)    /* READ_SINGLE_BLOCK */
#define CMD18  (0x40+18)    /* READ_MULTIPLE_BLOCK */
#define CMD23  (0x40+23)    /* SET_BLOCK_COUNT */
#define CMD24  (0x40+24)    /* WRITE_BLOCK */
#define CMD25  (0x40+25)    /* WRITE_MULTIPLE_BLOCK */
#define ACMD41 (0xC0+41)    /* SEND_OP_COND (SDC) */
#define CMD55  (0x40+55)    /* APP_CMD */
#define CMD58  (0x40+58)    /* READ_OCR */

static volatile DSTATUS Stat = STA_NOINIT;
static BYTE CardType;

/* Відправка/Прийом 1 байта (з мінімальним таймаутом) */
static BYTE SPI_TxRx(BYTE data) {
    BYTE rx = 0xFF;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, 10);
    return rx;
}

/* Відпускаємо шину MISO */
static void SPI_Release(void) {
    SPI_TxRx(0xFF);
}

/* Очікування готовності карти */
static BYTE SD_WaitReady(void) {
    BYTE res;
    uint32_t tmr = HAL_GetTick();
    do {
        res = SPI_TxRx(0xFF);
    } while (res != 0xFF && (HAL_GetTick() - tmr) < 500);
    return res;
}

/* Відправка команди */
static BYTE SD_SendCmd(BYTE cmd, DWORD arg) {
    BYTE n, res;

    if (cmd & 0x80) {
        cmd &= 0x7F;
        res = SD_SendCmd(CMD55, 0);
        if (res > 1) return res;
    }

    SD_CS_LOW();
    SPI_TxRx(0xFF);

    if (SD_WaitReady() != 0xFF) return 0xFF;

    SPI_TxRx(cmd);
    SPI_TxRx((BYTE)(arg >> 24));
    SPI_TxRx((BYTE)(arg >> 16));
    SPI_TxRx((BYTE)(arg >> 8));
    SPI_TxRx((BYTE)arg);

    n = 0x01;
    if (cmd == CMD0) n = 0x95;
    if (cmd == CMD8) n = 0x87;
    SPI_TxRx(n);

    if (cmd == CMD12) SPI_TxRx(0xFF);

    n = 10;
    do {
        res = SPI_TxRx(0xFF);
    } while ((res & 0x80) && --n);

    return res;
}

/* Прийом блоку даних з перевіркою токена */
static int SD_RxDataBlock(BYTE *buff, UINT btr) {
    BYTE token;
    uint32_t tmr = HAL_GetTick();

    do {
        token = SPI_TxRx(0xFF);
    } while ((token == 0xFF) && (HAL_GetTick() - tmr) < 200);

    if (token != 0xFE) return 0; /* Якщо не отримали 0xFE - помилка */

    do {
        *buff++ = SPI_TxRx(0xFF);
        *buff++ = SPI_TxRx(0xFF);
    } while (btr -= 2);

    SPI_TxRx(0xFF); /* CRC (Discard) */
    SPI_TxRx(0xFF);

    return 1;
}

/* Передача блоку даних */
static int SD_TxDataBlock(const BYTE *buff, BYTE token) {
    BYTE resp;
    if (SD_WaitReady() != 0xFF) return 0;

    SPI_TxRx(token);

    if (token != 0xFD) { /* Якщо не STOP_TRAN token */
        for (uint16_t wc = 0; wc < 512; wc++) {
            SPI_TxRx(*buff++);
        }
        SPI_TxRx(0xFF); /* CRC (Dummy) */
        SPI_TxRx(0xFF);

        resp = SPI_TxRx(0xFF);
        if ((resp & 0x1F) != 0x05) return 0; /* Якщо Data Rejected */
    }
    return 1;
}

/* ========================================================================= */
/* API драйвера для FatFS                                                    */
/* ========================================================================= */

DSTATUS FATFS_SD_DiskInitialize(BYTE pdrv) {
    BYTE n, cmd, ty, ocr[4];
    uint32_t tmr;

    if (pdrv) return STA_NOINIT;

    SD_CS_HIGH();
    /* Dummy clocks (мінімум 74) для запуску карти */
    for (n = 0; n < 10; n++) SPI_TxRx(0xFF);

    ty = 0;
    if (SD_SendCmd(CMD0, 0) == 1) {
        if (SD_SendCmd(CMD8, 0x1AA) == 1) {
            for (n = 0; n < 4; n++) ocr[n] = SPI_TxRx(0xFF);
            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
                tmr = HAL_GetTick();
                while ((HAL_GetTick() - tmr) < 1000 && SD_SendCmd(ACMD41, 1UL << 30)) ;
                if ((HAL_GetTick() - tmr) < 1000 && SD_SendCmd(CMD58, 0) == 0) {
                    for (n = 0; n < 4; n++) ocr[n] = SPI_TxRx(0xFF);
                    ty = (ocr[0] & 0x40) ? 6 : 2; /* SDv2 (Block або Byte addr) */
                }
            }
        } else {
            cmd = (SD_SendCmd(ACMD41, 0) <= 1) ? ACMD41 : CMD1;
            tmr = HAL_GetTick();
            while ((HAL_GetTick() - tmr) < 1000 && SD_SendCmd(cmd, 0)) ;
            if ((HAL_GetTick() - tmr) < 1000) {
                ty = 1; /* SDv1 або MMC */
                SD_SendCmd(CMD16, 512); /* Примусово 512 байт для SDSC */
            }
        }
    }

    CardType = ty;
    SD_CS_HIGH();
    SPI_Release();

    Stat = ty ? 0 : STA_NOINIT;
    return Stat;
}

DSTATUS FATFS_SD_DiskStatus(BYTE pdrv) {
    if (pdrv) return STA_NOINIT;
    return Stat;
}

DRESULT FATFS_SD_DiskRead(BYTE pdrv, BYTE* buff, DWORD sector, UINT count) {
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & 4)) sector *= 512; /* Byte addressing для старих карт */

    if (count == 1) {
        if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, 512)) count = 0;
    } else {
        if (SD_SendCmd(CMD18, sector) == 0) {
            do {
                if (!SD_RxDataBlock(buff, 512)) break;
                buff += 512;
            } while (--count);
            SD_SendCmd(CMD12, 0);
        }
    }

    SD_CS_HIGH();
    SPI_Release();
    return count ? RES_ERROR : RES_OK;
}

DRESULT FATFS_SD_DiskWrite(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count) {
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (Stat & STA_PROTECT) return RES_WRPRT;

    if (!(CardType & 4)) sector *= 512;

    if (count == 1) {
        if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlock(buff, 0xFE)) count = 0;
    } else {
        if (CardType & 2) {
            SD_SendCmd(CMD55, 0);
            SD_SendCmd(CMD23, count);
        }
        if (SD_SendCmd(CMD25, sector) == 0) {
            do {
                if (!SD_TxDataBlock(buff, 0xFC)) break;
                buff += 512;
            } while (--count);
            if (!SD_TxDataBlock(0, 0xFD)) count = 1;
        }
    }

    SD_CS_HIGH();
    SPI_Release();
    return count ? RES_ERROR : RES_OK;
}

DRESULT FATFS_SD_DiskIoctl(BYTE pdrv, BYTE cmd, void* buff) {
    DRESULT res = RES_ERROR;
    BYTE n, csd[16];
    DWORD csize;

    if (pdrv) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (cmd) {
        case CTRL_SYNC:
            SD_CS_LOW();
            if (SD_WaitReady() == 0xFF) res = RES_OK;
            break;

        case GET_SECTOR_COUNT:
            if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16)) {
                if ((csd[0] >> 6) == 1) { /* SDC ver 2.00 (SDHC/SDXC) */
                    csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
                    *(DWORD*)buff = csize << 10;
                } else { /* SDC ver 1.XX або MMC */
                    n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                    csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                    *(DWORD*)buff = csize << (n - 9);
                }
                res = RES_OK;
            }
            break;

        case GET_BLOCK_SIZE:
            *(DWORD*)buff = 128; /* Флеш пам'ять зазвичай стирається блоками по 64КБ */
            res = RES_OK;
            break;

        default:
            res = RES_PARERR;
    }

    SD_CS_HIGH();
    SPI_Release();
    return res;
}
