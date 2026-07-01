/* ============================================================
 * urbanpulse_main.c  —  Top-level application logic  v5 (UART + SD)
 * ============================================================ */

#include "urbanpulse.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

/* ---- Extern HAL handles (генерує CubeMX в main.c) ---------- */
extern I2C_HandleTypeDef hi2c1;   /* PB6=SCL, PB7=SDA          */
extern SPI_HandleTypeDef hspi1;   /* PB13=SCK, PB14=MISO, PB15=MOSI */
extern UART_HandleTypeDef huart1; /* PA9=TX, PA10=RX (Для ПК)  */

/* ---- CS пін SD-картки (SPI2, права сторона) ---------------- */
#define SD_CS_PORT  GPIOA
#define SD_CS_PIN   GPIO_PIN_4
#define SD_CS_LOW()  HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH() HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET)

/* ---- DSP module instances ---------------------------------- */
static BiquadQ15_t         g_bpf_x, g_bpf_y, g_bpf_z;
static StillnessDetector_t g_still;
static Kalman1D_t          g_kalman;
static Madgwick_t          g_madgwick;
static HilbertFIR_t        g_hilbert;
static HaarWavelet_t       g_haar;
HMM_t g_hmm;

/* ---- RMS / peak акумулятори для 1-секундного вікна --------- */
static float    g_rms_accum  = 0.0f;
static float    g_peak_g     = 0.0f;
static float    g_env_peak   = 0.0f;
static uint32_t g_sample_cnt = 0;

/* ---- Output record ----------------------------------------- */
static RoadRecord_t g_record;

/* ---- FatFS handles ----------------------------------------- */
static FATFS   g_fs;
static FIL     g_fil;
static uint8_t g_file_open = 0;

/* ============================================================
 * MPU6050 — I2C1 (PB6/PB7)
 * ============================================================ */

static HAL_StatusTypeDef MPU_Write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buf, 2, 10);
}

static HAL_StatusTypeDef MPU_ReadWhoAmI(uint8_t *id)
{
    uint8_t reg = 0x75;
    HAL_StatusTypeDef st;
    st = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 5);
    if (st != HAL_OK) return st;
    return HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, id, 1, 5);
}

/* ============================================================
 * FatFS / SD + UART
 * ============================================================ */

static void SD_Open(void)
{
    SD_CS_HIGH();

    // 1. ДАЄМО ФЛЕШЦІ ЧАС ПРОКИНУТИСЯ (Критично для павербанків!)
    HAL_Delay(500);

    // 2. Пробуємо змонтувати систему (цифра 1 означає "зробити це негайно")
    if (f_mount(&g_fs, "", 1) != FR_OK) {
        // Якщо не вийшло з першого разу — чекаємо ще пів секунди і пробуємо знову
        HAL_Delay(500);
        if (f_mount(&g_fs, "", 1) != FR_OK) {
            return; // Якщо і тепер глухо — значить картки тупо немає або вона згоріла
        }
    }

    // 3. Відкриваємо існуючий або СТВОРЮЄМО НОВИЙ файл
    if (f_open(&g_fil, "road_log.csv", FA_OPEN_ALWAYS | FA_WRITE) == FR_OK) {

        // Зміщуємо курсор у самий кінець файлу, щоб не перезаписати старі дані
        f_lseek(&g_fil, f_size(&g_fil));

        // Якщо файл щойно створений (розмір 0 байт) — пишемо йому шапку з назвами стовпців
        if (f_size(&g_fil) == 0) {
            f_puts("timestamp_ms,rms_z_g,max_impact_g,kalman_disp_mm,"
                   "envelope_peak,wavelet_e1,wavelet_e2,wavelet_e3,"
                   "surface_class,is_still\r\n", &g_fil);
            f_sync(&g_fil);
        }

        // Ставимо зелене світло для функції запису
        g_file_open = 1;
    }
}

static void SD_FlushRecord(RoadRecord_t *r)
{
    char line[160];
    snprintf(line, sizeof(line),
        "%lu,%.4f,%.4f,%.2f,%.4f,%.6f,%.6f,%.6f,%u,%u\r\n",
        (unsigned long)r->timestamp_ms,
        r->roughness_rms,
        r->max_impact_g,
        r->kalman_disp_mm,
        r->envelope_peak,
        r->wavelet_e1,
        r->wavelet_e2,
        r->wavelet_e3,
        r->surface_class,
        r->is_still);

    /* 1. Відправляємо дані на комп'ютер через UART */
    HAL_UART_Transmit(&huart1, (uint8_t*)line, strlen(line), 50);

    /* 2. Якщо SD-картка є і відкрилася — пишемо і на неї */
    if (g_file_open) {
        f_puts(line, &g_fil);
        f_sync(&g_fil);

        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
}

/* ============================================================
 * UP_Init
 * ============================================================ */

void UP_Init(void)
{
    HAL_Delay(100);
    MPU_Write(MPU6050_PWR_MGMT_1, 0x00);
    MPU_Write(MPU6050_SMPLRT_DIV, 0x09);
    MPU_Write(MPU6050_CONFIG_REG,  0x03);
    MPU_Write(MPU6050_GYRO_CFG,    0x00);
    MPU_Write(MPU6050_ACCEL_CFG,   0x00);
    HAL_Delay(10);

    uint8_t who = 0;
    MPU_ReadWhoAmI(&who);

    BPF_Init(&g_bpf_x);
    BPF_Init(&g_bpf_y);
    BPF_Init(&g_bpf_z);
    Stillness_Init(&g_still);
    Kalman_Init(&g_kalman);
    Madgwick_Init(&g_madgwick);
    Hilbert_Init(&g_hilbert);
    Haar_Init(&g_haar);
    HMM_Init(&g_hmm);

    memset(&g_record, 0, sizeof(g_record));

    SD_Open();
}

/* ============================================================
 * UP_ProcessSample
 * ============================================================ */

void UP_ProcessSample(int16_t ax_raw, int16_t ay_raw, int16_t az_raw,
                      int16_t gx_raw, int16_t gy_raw, int16_t gz_raw)
{
    float gx_rads = (float)gx_raw / 131.0f * 0.017453f;
    float gy_rads = (float)gy_raw / 131.0f * 0.017453f;
    float gz_rads = (float)gz_raw / 131.0f * 0.017453f;

    float ax_g = (float)ax_raw / ACCEL_SCALE_G;
    float ay_g = (float)ay_raw / ACCEL_SCALE_G;
    float az_g = (float)az_raw / ACCEL_SCALE_G;

    Madgwick_Update(&g_madgwick,
                    gx_rads, gy_rads, gz_rads,
                    ax_g, ay_g, az_g);
    float az_corrected_ms2 = Madgwick_GetVerticalAccel(
                                &g_madgwick, ax_g, ay_g, az_g);

    int32_t az_q15 = (int32_t)(az_corrected_ms2 / G_TO_MS2
                                * Q15_SCALE / 2.0f);
    if (az_q15 >  32767) az_q15 =  32767;
    if (az_q15 < -32768) az_q15 = -32768;

    int32_t az_filt_q15 = BPF_Process(&g_bpf_z, az_q15);
    float   az_filt_g   = (float)az_filt_q15 / Q15_SCALE * 2.0f;

    int32_t ax_q15  = (int32_t)(ax_g * Q15_SCALE / 2.0f);
    int32_t ay_q15  = (int32_t)(ay_g * Q15_SCALE / 2.0f);
    int32_t ax_filt = BPF_Process(&g_bpf_x, ax_q15);
    int32_t ay_filt = BPF_Process(&g_bpf_y, ay_q15);

    Stillness_Update(&g_still, ax_filt, ay_filt, az_filt_q15);

    float disp_m = Kalman_Update(&g_kalman,
                                 az_corrected_ms2,
                                 g_still.is_still);

    Hilbert_Update(&g_hilbert, az_filt_q15);
    Haar_AddSample(&g_haar, az_filt_q15);

    float az_abs = fabsf(az_filt_g);
    g_rms_accum += az_filt_g * az_filt_g;
    if (az_abs > g_peak_g)
        g_peak_g = az_abs;
    if (g_hilbert.envelope > g_env_peak)
        g_env_peak = g_hilbert.envelope;
    g_sample_cnt++;

    float obs[HMM_OBS_DIM] = {
        az_abs,
        g_hilbert.envelope,
        g_haar.energy_L3
    };
    HMM_Update(&g_hmm, obs);

    g_record.surface_class    = g_hmm.current_state;
    g_record.is_still         = g_still.is_still;
    g_record.kalman_disp_mm   = disp_m * 1000.0f;
    g_record.instant_z_abs    = az_abs;
    g_record.instant_envelope = g_hilbert.envelope;
    g_record.instant_w3       = g_haar.energy_L3;

    if (g_sample_cnt >= SAMPLE_RATE_HZ) {
        g_record.timestamp_ms  = HAL_GetTick();
        g_record.roughness_rms = sqrtf(g_rms_accum / g_sample_cnt);
        g_record.max_impact_g  = g_peak_g;
        g_record.envelope_peak = g_env_peak;
        g_record.wavelet_e1    = g_haar.energy_L1;
        g_record.wavelet_e2    = g_haar.energy_L2;
        g_record.wavelet_e3    = g_haar.energy_L3;

        SD_FlushRecord(&g_record);

        g_rms_accum  = 0.0f;
        g_peak_g     = 0.0f;
        g_env_peak   = 0.0f;
        g_sample_cnt = 0;
    }
}

RoadRecord_t* UP_GetRecord(void)
{
    return &g_record;
}

/* ============================================================
 * Ring buffer + state machine
 * ============================================================ */
#define RING_SIZE 128

typedef struct { int16_t ax, ay, az, gx, gy, gz; } ImuSample_t;

static volatile ImuSample_t g_ring[RING_SIZE];
static volatile uint8_t     g_ring_head = 0;
static volatile uint8_t     g_ring_tail = 0;

uint8_t Ring_Available(void)
{
    __disable_irq();
    uint8_t h = g_ring_head, t = g_ring_tail;
    __enable_irq();
    return (uint8_t)((h - t) & (RING_SIZE - 1));
}

ImuSample_t Ring_Pop(void)
{
    __disable_irq();
    ImuSample_t s = g_ring[g_ring_tail];
    g_ring_tail = (g_ring_tail + 1) & (RING_SIZE - 1);
    __enable_irq();
    return s;
}

void Ring_Push(int16_t ax, int16_t ay, int16_t az,
               int16_t gx, int16_t gy, int16_t gz)
{
    uint8_t next = (g_ring_head + 1) & (RING_SIZE - 1);
    if (next == g_ring_tail) return;
    g_ring[g_ring_head].ax = ax; g_ring[g_ring_head].ay = ay;
    g_ring[g_ring_head].az = az; g_ring[g_ring_head].gx = gx;
    g_ring[g_ring_head].gy = gy; g_ring[g_ring_head].gz = gz;
    g_ring_head = next;
}

static uint8_t mpu_dma_buf[14];
typedef enum { STATE_TAP_WAIT=0, STATE_CALIBRATE=1, STATE_RUN=2 } AppState_t;
static volatile AppState_t g_state = STATE_TAP_WAIT;

#define CALIB_SAMPLES (30 * SAMPLE_RATE_HZ)
static uint32_t g_calib_count   = 0;
static double   g_calib_sum_rms = 0.0;
static double   g_calib_sum_env = 0.0;
static double   g_calib_sum_w3  = 0.0;

static void Calib_AddSample(void)
{
    RoadRecord_t *r = &g_record;
    g_calib_sum_rms += r->instant_z_abs;
    g_calib_sum_env += r->instant_envelope;
    g_calib_sum_w3  += r->instant_w3;
    if (++g_calib_count < CALIB_SAMPLES) return;

    g_hmm.mu[0][0] = (float)(g_calib_sum_rms / g_calib_count);
    g_hmm.mu[0][1] = (float)(g_calib_sum_env / g_calib_count);
    g_hmm.mu[0][2] = (float)(g_calib_sum_w3  / g_calib_count);

    float b0=g_hmm.mu[0][0], b1=g_hmm.mu[0][1], b2=g_hmm.mu[0][2];
    g_hmm.mu[1][0]=b0*3.f;  g_hmm.mu[1][1]=b1*3.f;  g_hmm.mu[1][2]=b2*3.f;
    g_hmm.mu[2][0]=b0*6.f;  g_hmm.mu[2][1]=b1*5.f;  g_hmm.mu[2][2]=b2*6.f;
    g_hmm.mu[3][0]=b0*12.f; g_hmm.mu[3][1]=b1*15.f; g_hmm.mu[3][2]=b2*12.f;
    g_hmm.mu[4][0]=b0*8.f;  g_hmm.mu[4][1]=b1*12.f; g_hmm.mu[4][2]=b2*8.f;

    for (int s=0;s<HMM_STATES;s++)
        for (int d=0;d<HMM_OBS_DIM;d++)
            g_hmm.sigma[s][d] = g_hmm.mu[s][d]*0.4f + 1e-6f;

    g_state = STATE_RUN;
}

/* ============================================================
 * Індикація станів (Асинхронний світлодіод)
 * ============================================================ */
static void UP_UpdateLED(void)
{
    static uint32_t last_toggle = 0;
    uint32_t now = HAL_GetTick(); /* Беремо поточний час у мілісекундах */
    uint32_t interval = 0;

    /* Визначаємо, з якою швидкістю блимати залежно від стану */
    if (g_file_open == 0) {
        interval = 50;  /* ПОМИЛКА: Флешки немає або не відкрилась. "Паніка" */
    } else if (g_state == STATE_TAP_WAIT) {
        interval = 500; /* ОЧІКУВАННЯ: Спокійне блимання (чекаємо ляпаса) */
    } else if (g_state == STATE_CALIBRATE) {
        interval = 100; /* КАЛІБРУВАННЯ: Швидке блимання (збираємо базу) */
    } else if (g_state == STATE_RUN) {
        /* ЗАПИС: Тут нічого не робимо!
           Блимати раз на секунду буде функція SD_FlushRecord,
           щоб ти бачив реальний момент запису на флешку. */
        return;
    }

    /* Якщо потрібний інтервал часу пройшов — перемикаємо стан діода */
    if (interval > 0 && (now - last_toggle >= interval)) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        last_toggle = now;
    }
}

void UP_RunLoop(void)
{
	UP_UpdateLED();

    while (Ring_Available() > 0) {
        ImuSample_t s = Ring_Pop();
        switch (g_state) {
        case STATE_TAP_WAIT: {
            float az_g = (float)s.az / ACCEL_SCALE_G;
            if (fabsf(az_g) > TAP_THRESHOLD_G) {
                g_state = STATE_CALIBRATE;
                g_calib_count = 0;
                g_calib_sum_rms = g_calib_sum_env = g_calib_sum_w3 = 0.0;
            }
            break;
        }
        case STATE_CALIBRATE:
            UP_ProcessSample(s.ax,s.ay,s.az,s.gx,s.gy,s.gz);
            Calib_AddSample();
            break;
        case STATE_RUN:
            UP_ProcessSample(s.ax,s.ay,s.az,s.gx,s.gy,s.gz);
            break;
        }
    }
}

/* ============================================================
 * Таймерне переривання (100 Гц)
 * ============================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    /* Запускаємо апаратне фонове читання (читаємо 14 байт, починаючи з регістра ACCEL_XOUT) */
    /* Ця функція НЕ блокує мікроконтролер! */
    HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT, I2C_MEMADD_SIZE_8BIT, mpu_dma_buf, 14);
}

/* Ця функція викликається автоматично, коли DMA успішно завершив читання I2C */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        int16_t ax = (int16_t)((mpu_dma_buf[0]  << 8) | mpu_dma_buf[1]);
        int16_t ay = (int16_t)((mpu_dma_buf[2]  << 8) | mpu_dma_buf[3]);
        int16_t az = (int16_t)((mpu_dma_buf[4]  << 8) | mpu_dma_buf[5]);
        int16_t gx = (int16_t)((mpu_dma_buf[8]  << 8) | mpu_dma_buf[9]);
        int16_t gy = (int16_t)((mpu_dma_buf[10] << 8) | mpu_dma_buf[11]);
        int16_t gz = (int16_t)((mpu_dma_buf[12] << 8) | mpu_dma_buf[13]);

        Ring_Push(ax, ay, az, gx, gy, gz);
    }
}
