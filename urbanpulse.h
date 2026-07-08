#ifndef URBANPULSE_H
#define URBANPULSE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string.h>

/* ============================================================
 *  URBANPULSE — Raw Data Logger
 *  Target: STM32F103C8T6 (Blue Pill), STM32CubeIDE / HAL
 *  Sensor: MPU6050 via I2C1 (PB6=SCL, PB7=SDA)
 *  Storage: MicroSD via SPI1 + FatFS
 *  Sample rate: 100 Hz (10 ms timer interrupt)
 * ============================================================ */

/* ---------- Hardware config --------------------------------- */
#define MPU6050_ADDR        (0x68 << 1)   /* AD0 = GND        */
#define MPU6050_SMPLRT_DIV  0x19
#define MPU6050_CONFIG_REG  0x1A
#define MPU6050_GYRO_CFG    0x1B
#define MPU6050_ACCEL_CFG   0x1C
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_ACCEL_XOUT  0x3B

#define SAMPLE_RATE_HZ      100           /* 10 ms per sample  */
#define CHUNK_SIZE          100           /* 1 second of data  */

/* ---------- Data Types -------------------------------------- */
typedef struct {
    uint32_t timestamp_ms;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} ImuSample_t;

/* ---------- Public API -------------------------------------- */
void UP_Init(void);
void UP_RunLoop(void);

#endif /* URBANPULSE_H */
