/* ============================================================
 * urbanpulse_main.c  —  Raw Logger (SD Card + UART Live Stream)
 * ============================================================ */

#include "urbanpulse.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1; // <--- ДОДАНО ДЛЯ UART

static FATFS   g_fs;
static FIL     g_fil;
static uint8_t g_file_open = 0;
static uint8_t g_sd_error  = 0;

static uint8_t mpu_dma_buf[14];

/* Double Buffering */
static ImuSample_t g_ping[CHUNK_SIZE];
static ImuSample_t g_pong[CHUNK_SIZE];
static ImuSample_t *g_active_buf = g_ping;
static ImuSample_t *g_ready_buf = NULL;
static volatile uint32_t g_buf_idx = 0;
static volatile uint8_t g_buffer_ready = 0;

/* Large text buffer */
static char g_text_buf[6144];

static HAL_StatusTypeDef MPU_Write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buf, 2, 10);
}

void UP_Init(void)
{
    /* Mount SD Card immediately as in the original code.
       Delaying here causes some SD cards to fail SPI initialization. */
    g_sd_error = 1;
    for (int i = 0; i < 3; i++) {
        if (f_mount(&g_fs, "", 1) == FR_OK) {
            g_sd_error = 0;
            break;
        }
        HAL_Delay(50);
    }

    if (g_sd_error == 0) {
        /* Find the next available filename: ROUTE001.CSV to ROUTE999.CSV */
        char filename[16];
        g_file_open = 0;

        for (int i = 1; i <= 999; i++) {
            snprintf(filename, sizeof(filename), "ROUTE%03d.CSV", i);
            /* FA_CREATE_NEW fails if the file already exists */
            if (f_open(&g_fil, filename, FA_CREATE_NEW | FA_WRITE) == FR_OK) {
                f_puts("timestamp_ms,ax,ay,az,gx,gy,gz\r\n", &g_fil);
                f_sync(&g_fil);
                g_file_open = 1;
                break;
            }
        }

        if (!g_file_open) {
            g_sd_error = 2; /* File error (e.g. all 999 files exist, or SD is read-only) */
        }
    }

    /* Boot sequence: Wait 3 seconds for power stabilization BEFORE starting logging
       and MPU6050 initialization. */
    HAL_Delay(3000);

    /* Configure Timer */
    __HAL_TIM_SET_PRESCALER(&htim3, 799);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 899);

    /* Configure MPU6050 */
    MPU_Write(MPU6050_PWR_MGMT_1, 0x00);
    MPU_Write(MPU6050_SMPLRT_DIV, 0x09);
    MPU_Write(MPU6050_CONFIG_REG,  0x03);
    MPU_Write(MPU6050_GYRO_CFG,    0x00);
    MPU_Write(MPU6050_ACCEL_CFG,   0x00);
    HAL_Delay(10);
}

void UP_RunLoop(void)
{
    if (g_buffer_ready) {
        char *ptr = g_text_buf;
        int remaining = sizeof(g_text_buf);

        for (int i = 0; i < CHUNK_SIZE; i++) {
            ImuSample_t *s = &g_ready_buf[i];
            int len = snprintf(ptr, remaining, "%lu,%d,%d,%d,%d,%d,%d\r\n",
                     (unsigned long)s->timestamp_ms,
                     s->ax, s->ay, s->az,
                     s->gx, s->gy, s->gz);
            if (len > 0 && len < remaining) {
                ptr += len;
                remaining -= len;
            }
        }

        size_t total_len = ptr - g_text_buf;

        /* ОДНОЧАСНА ТРАНСЛЯЦІЯ НА КОМП'ЮТЕР (ЛАЙВ) */
        HAL_UART_Transmit(&huart1, (uint8_t*)g_text_buf, total_len, HAL_MAX_DELAY);

        /* ТА ЗАПИС НА SD-КАРТКУ */
        if (g_file_open) {
            UINT bw;
            if (f_write(&g_fil, g_text_buf, total_len, &bw) == FR_OK && bw == total_len) {
                /* Only sync every 10 chunks (10 seconds) to prevent FAT thrashing
                   and wear-leveling delays on cheap SD cards. */
                static uint8_t sync_counter = 0;
                if (++sync_counter >= 10) {
                    f_sync(&g_fil);
                    sync_counter = 0;
                }
                /* Blink LED to indicate successful write */
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            } else {
                g_file_open = 0; /* Stop writing on error */
                g_sd_error = 2;
            }
        }
        g_buffer_ready = 0;
    } else if (!g_file_open) {
        /* Blink LED to indicate SD error:
           g_sd_error == 1 (No SD card) -> 50ms (strobe)
           g_sd_error == 2 (File error) -> 2000ms (slow blink) */
        static uint32_t last_toggle = 0;
        uint32_t now = HAL_GetTick();
        uint32_t interval = (g_sd_error == 1) ? 50 : 2000;
        if (interval > 0 && (now - last_toggle >= interval)) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            last_toggle = now;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        /* I2C Vibration Recovery */
        if (hi2c1.State != HAL_I2C_STATE_READY) {
            HAL_I2C_DeInit(&hi2c1);
            HAL_I2C_Init(&hi2c1);
        }

        /* Trigger I2C DMA read of MPU6050 data */
        HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT, I2C_MEMADD_SIZE_8BIT, mpu_dma_buf, 14);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        if (g_buffer_ready) {
            /* Main loop didn't process previous chunk fast enough.
               Ignore new samples to prevent overwriting active buffer. */
            return;
        }

        int16_t ax = (int16_t)((mpu_dma_buf[0]  << 8) | mpu_dma_buf[1]);
        int16_t ay = (int16_t)((mpu_dma_buf[2]  << 8) | mpu_dma_buf[3]);
        int16_t az = (int16_t)((mpu_dma_buf[4]  << 8) | mpu_dma_buf[5]);
        int16_t gx = (int16_t)((mpu_dma_buf[8]  << 8) | mpu_dma_buf[9]);
        int16_t gy = (int16_t)((mpu_dma_buf[10] << 8) | mpu_dma_buf[11]);
        int16_t gz = (int16_t)((mpu_dma_buf[12] << 8) | mpu_dma_buf[13]);

        g_active_buf[g_buf_idx].ax = ax;
        g_active_buf[g_buf_idx].ay = ay;
        g_active_buf[g_buf_idx].az = az;
        g_active_buf[g_buf_idx].gx = gx;
        g_active_buf[g_buf_idx].gy = gy;
        g_active_buf[g_buf_idx].gz = gz;
        g_active_buf[g_buf_idx].timestamp_ms = HAL_GetTick();

        g_buf_idx++;
        if (g_buf_idx >= CHUNK_SIZE) {
            g_ready_buf = g_active_buf;
            g_active_buf = (g_active_buf == g_ping) ? g_pong : g_ping;
            g_buf_idx = 0;
            g_buffer_ready = 1;
        }
    }
}
