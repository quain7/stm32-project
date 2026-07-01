#ifndef URBANPULSE_H
#define URBANPULSE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ============================================================
 *  URBANPULSE — Road Quality Mapper
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
#define SAMPLE_DT           0.01f         /* seconds           */
#define G_TO_MS2            9.80665f

/* ---------- Fixed-point scaling ----------------------------- */
/* All Q15 values: 1.0 real = 32767 Q15                        */
#define Q15_ONE             32767
#define Q15_SCALE           32768.0f
#define ACCEL_SCALE_G       16384.0f      /* ±2G range          */

/* ---------- IIR Butterworth BPF (4–80 Hz @ 100 Hz Fs) ------ */
/* 2nd order, fixed-point Q15 coefficients                      */
/* Generated: scipy.signal.butter(2, [4,80], 'bandpass', fs=100)*/
/* b = [0.5765, 0, -0.5765]  a = [1, -0.2141, -0.1529]        */
#define BPF_STAGES          1             /* single biquad      */
typedef struct {
    int32_t b0, b1, b2;                  /* Q15 numerator      */
    int32_t a1, a2;                      /* Q15 denominator    */
    int32_t x1, x2;                      /* input delay line   */
    int32_t y1, y2;                      /* output delay line  */
} BiquadQ15_t;

/* ---------- 1D Kalman Filter (vertical axis Z) -------------- */
typedef struct {
    float x_pos;      /* estimated displacement [m]             */
    float x_vel;      /* estimated velocity [m/s]               */
    float P[2][2];    /* error covariance matrix                */
    float Q[2][2];    /* process noise covariance               */
    float R;          /* measurement noise variance             */
} Kalman1D_t;

/* ---------- Madgwick AHRS ----------------------------------- */
typedef struct {
    float q0, q1, q2, q3;  /* quaternion (w, x, y, z)          */
    float beta;             /* filter gain (≈ 0.1 for slow veh) */
} Madgwick_t;

/* ---------- Stillness Detector (ZUPT) ----------------------- */
#define STILLNESS_WINDOW    20            /* 200 ms @ 100 Hz    */
#define STILLNESS_THRESH    328           /* 0.02G in Q15 units */
typedef struct {
    int32_t buf_x[STILLNESS_WINDOW];
    int32_t buf_y[STILLNESS_WINDOW];
    int32_t buf_z[STILLNESS_WINDOW];
    uint8_t idx;
    uint8_t is_still;
} StillnessDetector_t;

/* ---------- Hilbert FIR (25-tap, odd-symmetric) ------------- */
/* Approximates Hilbert transform over 4–80 Hz                  */
#define HILBERT_TAPS        25
#define HILBERT_DELAY       12            /* (TAPS-1)/2 samples */
typedef struct {
    int32_t h[HILBERT_TAPS];             /* Q15 coefficients   */
    int32_t delay_x[HILBERT_TAPS];       /* input delay line   */
    int32_t delay_line[HILBERT_DELAY];   /* for real part delay */
    uint8_t idx;
    float   envelope;                    /* |z(t)| output      */
} HilbertFIR_t;

/* ---------- Haar Wavelet (3 levels) ------------------------- */
/* Window = 8 samples → levels: 4, 2, 1 detail coefficients    */
#define HAAR_WINDOW         8
typedef struct {
    int32_t buf[HAAR_WINDOW];
    uint8_t count;
    /* Energies per level (squared sum of detail coefficients)  */
    float   energy_L1;   /* 25–50 Hz: bруківка, рипа           */
    float   energy_L2;   /* 12–25 Hz: знос, тріщини            */
    float   energy_L3;   /* 6–12 Hz: яма, поріг                */
} HaarWavelet_t;

/* ---------- HMM — 5 surface classes ------------------------- */
/*  0=smooth  1=worn  2=cobblestone  3=pothole  4=speedbump    */
#define HMM_STATES          5
#define HMM_OBS_DIM         3   /* (rms_z, envelope, wavelet_E3)*/
typedef struct {
    /* Transition matrix A[from][to] — row-stochastic           */
    float A[HMM_STATES][HMM_STATES];
    /* Emission: Gaussian per state — mu and sigma per dim      */
    float mu[HMM_STATES][HMM_OBS_DIM];
    float sigma[HMM_STATES][HMM_OBS_DIM];
    /* Forward algorithm state                                  */
    float alpha[HMM_STATES];   /* scaled forward variable      */
    float scale;               /* normalisation constant        */
    uint8_t current_state;     /* MAP estimate                 */
} HMM_t;

/* ---------- Output record written to SD each second --------- */
typedef struct {
    uint32_t timestamp_ms;
    float    roughness_rms;     /* RMS of filtered Z over 1 s   */
    float    max_impact_g;      /* peak |accel| in window       */
    float    kalman_disp_mm;    /* vertical displacement [mm]   */
    float    envelope_peak;     /* Hilbert envelope peak        */
    float    wavelet_e1;        /* energy level 1               */
    float    wavelet_e2;        /* energy level 2               */
    float    wavelet_e3;        /* energy level 3               */
    uint8_t  surface_class;     /* HMM output 0-4               */
    uint8_t  is_still;          /* 1 = parked / stopped         */
    /* Миттєві значення @ 100 Hz — для HMM калібрування        */
    float    instant_z_abs;     /* |az_filt| поточного семплу   */
    float    instant_envelope;  /* Hilbert envelope поточного   */
    float    instant_w3;        /* wavelet E3 поточного вікна   */
} RoadRecord_t;

/* ---------- Calibration phase (first 5 s = tap detection) --- */
#define CALIB_WINDOW_MS     5000
#define TAP_THRESHOLD_G     1.5f          /* raw G for tap       */

/* ---------- RMS accumulator (1 second window = 100 samples) - */
#define RMS_WINDOW          100

/* ---------- Public API -------------------------------------- */
void UP_Init(void);
void UP_ProcessSample(int16_t ax, int16_t ay, int16_t az,
                      int16_t gx, int16_t gy, int16_t gz);
RoadRecord_t* UP_GetRecord(void);
void UP_RunLoop(void);

/* ---------- DSP Internal API -------------------------------- */
void BPF_Init(BiquadQ15_t *f);
int32_t BPF_Process(BiquadQ15_t *f, int32_t x);

void Stillness_Init(StillnessDetector_t *s);
void Stillness_Update(StillnessDetector_t *s, int32_t ax, int32_t ay, int32_t az);

void Kalman_Init(Kalman1D_t *k);
float Kalman_Update(Kalman1D_t *k, float accel_ms2, uint8_t is_still);

void Madgwick_Init(Madgwick_t *m);
void Madgwick_Update(Madgwick_t *m, float gx, float gy, float gz, float ax, float ay, float az);
float Madgwick_GetVerticalAccel(Madgwick_t *m, float ax, float ay, float az);

void Hilbert_Init(HilbertFIR_t *h);
void Hilbert_Update(HilbertFIR_t *hf, int32_t x_q15);

void Haar_Init(HaarWavelet_t *w);
void Haar_AddSample(HaarWavelet_t *w, int32_t x_q15);

void HMM_Init(HMM_t *h);
void HMM_Update(HMM_t *h, float obs[HMM_OBS_DIM]);

#endif /* URBANPULSE_H */
