/* ============================================================
 *  urbanpulse_dsp.c  —  All on-board DSP algorithms
 *  STM32F103C8T6, no FPU — fixed-point Q15 where possible
 * ============================================================ */

#include "urbanpulse.h"
#include <stdlib.h>

/* ============================================================
 *  SECTION 1: Q15 IIR BUTTERWORTH BANDPASS FILTER (4–80 Hz)
 *  Designed with scipy.signal.butter(2, [4,80], 'bandpass', fs=100)
 *  Converted to Q15 fixed-point for Cortex-M3 (no FPU)
 *
 *  Continuous fraction: b=[0.5765, 0, -0.5765], a=[1,-0.2141,-0.1529]
 *  Q15 repr: multiply by 32768, round to int32
 * ============================================================ */

void BPF_Init(BiquadQ15_t *f)
{
    /* Numerator b (Q15) */
    f->b0 =  18889;   /*  0.5765 * 32768 */
    f->b1 =      0;   /*  0               */
    f->b2 = -18889;   /* -0.5765 * 32768 */

    /* Denominator a (Q15, negated for difference equation) */
    f->a1 =   7014;   /*  0.2141 * 32768  (stored as +, subtracted)  */
    f->a2 =   5011;   /*  0.1529 * 32768 */

    f->x1 = f->x2 = 0;
    f->y1 = f->y2 = 0;
}

/*
 * Direct Form II Transposed biquad, Q15 fixed-point.
 * Input x: raw accel in Q15 units (int32 to avoid overflow mid-calc).
 * Returns filtered sample in same Q15 domain.
 *
 * y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
 *       - a1*y[n-1] - a2*y[n-2]
 * All products >> 15 to stay in Q15.
 */
int32_t BPF_Process(BiquadQ15_t *f, int32_t x)
{
    int32_t y;
    y  = ((f->b0 * x)     >> 15);
    y += ((f->b1 * f->x1) >> 15);
    y += ((f->b2 * f->x2) >> 15);
    y -= ((f->a1 * f->y1) >> 15);
    y -= ((f->a2 * f->y2) >> 15);

    f->x2 = f->x1;  f->x1 = x;
    f->y2 = f->y1;  f->y1 = y;
    return y;
}


/* ============================================================
 *  SECTION 2: STILLNESS DETECTOR (ZUPT — Zero Velocity Update)
 *
 *  Keeps a rolling 200 ms window of accel samples.
 *  Device is "still" if std-dev on ALL three axes < 0.02G.
 *  Uses integer arithmetic: variance = E[x²] - (E[x])²
 * ============================================================ */

void Stillness_Init(StillnessDetector_t *s)
{
    memset(s, 0, sizeof(*s));
}

void Stillness_Update(StillnessDetector_t *s,
                      int32_t ax, int32_t ay, int32_t az)
{
    s->buf_x[s->idx] = ax;
    s->buf_y[s->idx] = ay;
    s->buf_z[s->idx] = az;
    s->idx = (s->idx + 1) % STILLNESS_WINDOW;

    /* Compute variance per axis using integer math             */
    int64_t sum_x=0, sum_x2=0;
    int64_t sum_y=0, sum_y2=0;
    int64_t sum_z=0, sum_z2=0;

    for (int i = 0; i < STILLNESS_WINDOW; i++) {
        sum_x  += s->buf_x[i];
        sum_x2 += (int64_t)s->buf_x[i] * s->buf_x[i];
        sum_y  += s->buf_y[i];
        sum_y2 += (int64_t)s->buf_y[i] * s->buf_y[i];
        sum_z  += s->buf_z[i];
        sum_z2 += (int64_t)s->buf_z[i] * s->buf_z[i];
    }

    /* var = (N*sum_x2 - sum_x^2) / N^2  — compare vs threshold²*/
    /* STILLNESS_THRESH = 0.02G in Q15 = 328                    */
    /* threshold² in Q30 units = 328² * WINDOW² / WINDOW²...   */
    /* simpler: check (N*Σx² - (Σx)²) < THRESH² * N²           */
    int64_t N   = STILLNESS_WINDOW;
    int64_t thr2 = (int64_t)STILLNESS_THRESH * STILLNESS_THRESH;

    int64_t var_x = (N * sum_x2 - sum_x * sum_x);
    int64_t var_y = (N * sum_y2 - sum_y * sum_y);
    int64_t var_z = (N * sum_z2 - sum_z * sum_z);
    int64_t limit = thr2 * N * N;

    s->is_still = (var_x < limit) && (var_y < limit) && (var_z < limit);
}


/* ============================================================
 *  SECTION 3: 1D KALMAN FILTER — vertical displacement (Z axis)
 *
 *  State vector: [position, velocity]
 *  Observation:  acceleration (double-integrated to position)
 *
 *  When stillness detector fires → ZUPT: force vel=0 & reset P
 *  This eliminates the classic quadratic drift of cheap IMUs.
 *
 *  Gauss-Markov process noise (from Allan Deviation analysis):
 *    Q_pos ≈ 1e-6 m²/s^4 * dt²
 *    Q_vel ≈ 1e-4 m²/s² (bias instability)
 *    R     ≈ 4e-4 m²  (accel noise integrated twice)
 * ============================================================ */

void Kalman_Init(Kalman1D_t *k)
{
    k->x_pos = 0.0f;
    k->x_vel = 0.0f;

    /* Initial covariance — high uncertainty */
    k->P[0][0] = 1.0f;  k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;  k->P[1][1] = 1.0f;

    /* Process noise (tuned for bus vibration environment) */
    float dt = SAMPLE_DT;
    k->Q[0][0] = 1e-6f * dt * dt;
    k->Q[0][1] = 0.0f;
    k->Q[1][0] = 0.0f;
    k->Q[1][1] = 1e-4f;

    /* Measurement noise */
    k->R = 4e-4f;
}

/*
 * Predict + Update step.
 * accel_ms2: filtered acceleration in m/s²  (float, after BPF)
 * Returns estimated vertical displacement in meters.
 */
float Kalman_Update(Kalman1D_t *k, float accel_ms2, uint8_t is_still)
{
    float dt = SAMPLE_DT;

    /* ---- ZUPT: if stopped, reset velocity & shrink P ---- */
    if (is_still) {
        k->x_vel   = 0.0f;
        k->P[1][0] = 0.0f;
        k->P[0][1] = 0.0f;
        k->P[1][1] = 1e-6f;  /* tight uncertainty on velocity */
    }

    /* ---- PREDICT ---------------------------------------- */
    /* State transition: pos += vel*dt + 0.5*a*dt²           */
    float x_pos_pred = k->x_pos + k->x_vel * dt + 0.5f * accel_ms2 * dt * dt;
    float x_vel_pred = k->x_vel + accel_ms2 * dt;

    /* Covariance prediction: P = F*P*F' + Q               */
    /* F = [[1, dt],[0, 1]]                                 */
    float P00 = k->P[0][0] + dt*(k->P[1][0] + k->P[0][1]) + dt*dt*k->P[1][1] + k->Q[0][0];
    float P01 = k->P[0][1] + dt * k->P[1][1] + k->Q[0][1];
    float P10 = k->P[1][0] + dt * k->P[1][1] + k->Q[1][0];
    float P11 = k->P[1][1] + k->Q[1][1];

    /* ---- UPDATE (position observation from integration) - */
    /* Innovation: z = pos_from_accel - predicted_pos        */
    /* We use the double-integrated accel as measurement     */
    float z = 0.5f * accel_ms2 * dt * dt;   /* Δpos from accel */
    float S = P00 + k->R;                   /* innovation covariance */

    /* Kalman gain: K = P*H' / S  (H=[1,0]) */
    float K0 = P00 / S;
    float K1 = P10 / S;

    float innov = z - x_pos_pred;
    k->x_pos = x_pos_pred + K0 * innov;
    k->x_vel = x_vel_pred + K1 * innov;

    /* Update covariance: P = (I - K*H)*P */
    k->P[0][0] = (1.0f - K0) * P00;
    k->P[0][1] = (1.0f - K0) * P01;
    k->P[1][0] = P10 - K1 * P00;
    k->P[1][1] = P11 - K1 * P01;

    return k->x_pos;
}


/* ============================================================
 *  SECTION 4: MADGWICK AHRS
 *
 *  Full quaternion orientation estimator.
 *  Fuses accel + gyro. Gives us gravity-corrected vertical Z,
 *  so we can subtract 1G properly even when bus is tilted.
 *
 *  beta ≈ 0.033 for pedestrian/vehicle applications.
 *  Reference: Madgwick (2010), "An efficient orientation filter..."
 * ============================================================ */

void Madgwick_Init(Madgwick_t *m)
{
    m->q0   = 1.0f;
    m->q1   = 0.0f;
    m->q2   = 0.0f;
    m->q3   = 0.0f;
    m->beta = 0.033f;
}

void Madgwick_Update(Madgwick_t *m,
                     float gx, float gy, float gz,   /* rad/s  */
                     float ax, float ay, float az)   /* any G  */
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;
    float _2q0, _2q1, _2q2, _2q3;
    float _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    /* Rate of change from gyro */
    qDot0 = 0.5f * (-m->q1*gx - m->q2*gy - m->q3*gz);
    qDot1 = 0.5f * ( m->q0*gx + m->q2*gz - m->q3*gy);
    qDot2 = 0.5f * ( m->q0*gy - m->q1*gz + m->q3*gx);
    qDot3 = 0.5f * ( m->q0*gz + m->q1*gy - m->q2*gx);

    /* Gradient descent from accelerometer */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        _2q0 = 2.0f * m->q0; _2q1 = 2.0f * m->q1;
        _2q2 = 2.0f * m->q2; _2q3 = 2.0f * m->q3;
        _4q0 = 4.0f * m->q0; _4q1 = 4.0f * m->q1; _4q2 = 4.0f * m->q2;
        _8q1 = 8.0f * m->q1; _8q2 = 8.0f * m->q2;
        q0q0 = m->q0*m->q0; q1q1 = m->q1*m->q1;
        q2q2 = m->q2*m->q2; q3q3 = m->q3*m->q3;

        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        s1 = _4q1*q3q3 - _2q3*ax + 4.0f*(q0q0*m->q1) - _2q0*ay
             - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        s2 = 4.0f*(q0q0*m->q2) + _2q0*ax + _4q2*q3q3
             - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        s3 = 4.0f*(q1q1*m->q3) - _2q1*ax + 4.0f*(q2q2*m->q3) - _2q2*ay;

        recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm;
        s2 *= recipNorm; s3 *= recipNorm;

        qDot0 -= m->beta * s0;
        qDot1 -= m->beta * s1;
        qDot2 -= m->beta * s2;
        qDot3 -= m->beta * s3;
    }

    m->q0 += qDot0 * SAMPLE_DT;
    m->q1 += qDot1 * SAMPLE_DT;
    m->q2 += qDot2 * SAMPLE_DT;
    m->q3 += qDot3 * SAMPLE_DT;

    recipNorm = 1.0f / sqrtf(m->q0*m->q0 + m->q1*m->q1 +
                              m->q2*m->q2 + m->q3*m->q3);
    m->q0 *= recipNorm; m->q1 *= recipNorm;
    m->q2 *= recipNorm; m->q3 *= recipNorm;
}

/*
 * Extract world-frame vertical acceleration (gravity removed).
 * Returns accel_z_world in m/s²  (pure road vibration component).
 */
float Madgwick_GetVerticalAccel(Madgwick_t *m,
                                float ax, float ay, float az)
{
    /* Rotate body-frame accel to world frame using quaternion  */
    /* World Z component: 2*(q1*q3 - q0*q2)*ax                 */
    /*                  + 2*(q2*q3 + q0*q1)*ay                 */
    /*                  + (q0²-q1²-q2²+q3²)*az                 */
    /* Then subtract 1G                                         */
    float az_world =
        2.0f*(m->q1*m->q3 - m->q0*m->q2)*ax +
        2.0f*(m->q2*m->q3 + m->q0*m->q1)*ay +
        (m->q0*m->q0 - m->q1*m->q1 - m->q2*m->q2 + m->q3*m->q3)*az;

    return (az_world - 1.0f) * G_TO_MS2;  /* subtract 1G, to m/s² */
}


/* ============================================================
 *  SECTION 5: HILBERT TRANSFORM FIR (25-tap)
 *
 *  Computes analytic signal z(t) = x(t) + j*H{x(t)}
 *  Envelope = |z(t)| — instantaneous impact amplitude.
 *  Coefficients: standard 25-tap Hilbert FIR (odd-symmetric,
 *  zero at even taps, scaled to Q15).
 *  h[k] = 2/(pi*k) * sin²(pi*k/2)  for k odd, else 0
 * ============================================================ */

static const int32_t hilbert_h[HILBERT_TAPS] = {
/*  h[-12]..h[0]..h[12]  (antisymmetric, center=0) */
    -831,    0, -1028,    0, -1415,    0, -2255,    0, -4817,
       0, -21306,  0,
       0,   /* center tap — always 0 for Hilbert */
       0, 21306,   0,  4817,    0,  2255,    0,  1415,
       0,  1028,   0,   831
};

void Hilbert_Init(HilbertFIR_t *h)
{
    memset(h, 0, sizeof(*h));
    memcpy(h->h, hilbert_h, sizeof(hilbert_h));
}

/*
 * Feed one filtered sample (Q15 int32).
 * Updates h->envelope with |z(t)| in Q15 units.
 * Real part is delayed by HILBERT_DELAY samples to align with imag.
 */
void Hilbert_Update(HilbertFIR_t *hf, int32_t x_q15)
{
    /* Shift delay lines */
    for (int i = HILBERT_TAPS-1; i > 0; i--)
        hf->delay_x[i] = hf->delay_x[i-1];
    hf->delay_x[0] = x_q15;

    /* Imaginary part: convolution with h[] */
    int64_t imag = 0;
    for (int i = 0; i < HILBERT_TAPS; i++)
        imag += (int64_t)hf->h[i] * hf->delay_x[i];
    int32_t imag_q15 = (int32_t)(imag >> 15);

    /* Real part: same signal delayed by (TAPS-1)/2 = 12 samples */
    int32_t real_q15 = hf->delay_line[HILBERT_DELAY-1];
    for (int i = HILBERT_DELAY-1; i > 0; i--)
        hf->delay_line[i] = hf->delay_line[i-1];
    hf->delay_line[0] = x_q15;

    /* Envelope = sqrt(real² + imag²) in Q15                   */
    /* Use float for sqrt — called 100×/s, acceptable overhead  */
    float r = (float)real_q15 / Q15_SCALE;
    float im = (float)imag_q15 / Q15_SCALE;
    hf->envelope = sqrtf(r*r + im*im);
}


/* ============================================================
 *  SECTION 6: HAAR WAVELET DECOMPOSITION (3 levels, 8-sample window)
 *
 *  Input: 8 filtered samples (Q15 int32)
 *  Output: energy at 3 detail levels
 *    Level 1 detail: 25–50 Hz → cobblestone, high-freq chatter
 *    Level 2 detail: 12–25 Hz → worn asphalt, cracks
 *    Level 3 detail: 6–12 Hz  → potholes, speed bumps
 *
 *  In-place lifting: split into even/odd, compute (odd-even)/2
 *  as detail, (odd+even)/2 as approximation. Repeat on approx.
 * ============================================================ */

void Haar_Init(HaarWavelet_t *w)
{
    memset(w, 0, sizeof(*w));
}

void Haar_AddSample(HaarWavelet_t *w, int32_t x_q15)
{
    w->buf[w->count++] = x_q15;
    if (w->count < HAAR_WINDOW) return;

    /* Have 8 samples — run 3-level Haar DWT */
    int32_t a[8];
    memcpy(a, w->buf, sizeof(a));
    w->count = 0;

    float e1 = 0, e2 = 0, e3 = 0;

    /* Level 1: length-8 → 4 approx + 4 detail              */
    int32_t approx[4], detail_L1[4];
    for (int i = 0; i < 4; i++) {
        approx[i]    = (a[2*i] + a[2*i+1]) >> 1;  /* avg  */
        detail_L1[i] = (a[2*i] - a[2*i+1]) >> 1;  /* diff */
        float d = (float)detail_L1[i] / Q15_SCALE;
        e1 += d * d;
    }

    /* Level 2: length-4 → 2 approx + 2 detail              */
    int32_t approx2[2], detail_L2[2];
    for (int i = 0; i < 2; i++) {
        approx2[i]    = (approx[2*i] + approx[2*i+1]) >> 1;
        detail_L2[i]  = (approx[2*i] - approx[2*i+1]) >> 1;
        float d = (float)detail_L2[i] / Q15_SCALE;
        e2 += d * d;
    }

    /* Level 3: length-2 → 1 approx + 1 detail              */
    int32_t detail_L3 = (approx2[0] - approx2[1]) >> 1;
    float d3 = (float)detail_L3 / Q15_SCALE;
    e3 = d3 * d3;

    /* Accumulate energies (exponential moving average)      */
    float alpha = 0.2f;
    w->energy_L1 = (1.0f - alpha)*w->energy_L1 + alpha*e1;
    w->energy_L2 = (1.0f - alpha)*w->energy_L2 + alpha*e2;
    w->energy_L3 = (1.0f - alpha)*w->energy_L3 + alpha*e3;
}


/* ============================================================
 *  SECTION 7: HIDDEN MARKOV MODEL — 5-class surface classifier
 *
 *  States: 0=smooth, 1=worn, 2=cobblestone, 3=pothole, 4=speedbump
 *  Observation vector: [rms_z, envelope_peak, wavelet_E3]
 *  Algorithm: Forward pass (scaled) → MAP state estimate.
 *
 *  Emission: Diagonal Gaussian per state
 *    p(o|s) = Π_d  N(o_d; mu[s][d], sigma[s][d])
 *
 *  Parameters below are physically motivated priors —
 *  tune from real Lviv road data after first test drive.
 * ============================================================ */

void HMM_Init(HMM_t *h)
{
    /*  Transition matrix — roads change state slowly          */
    float A_init[HMM_STATES][HMM_STATES] = {
    /* to:  smooth  worn    cobble  pothole speedbump */
    /*smooth*/  {0.90f,  0.08f,  0.01f,  0.005f, 0.005f},
    /*worn*/    {0.05f,  0.88f,  0.05f,  0.015f, 0.005f},
    /*cobble*/  {0.01f,  0.05f,  0.90f,  0.03f,  0.01f },
    /*pothole*/ {0.01f,  0.05f,  0.10f,  0.80f,  0.04f },
    /*speedbmp*/{0.10f,  0.20f,  0.10f,  0.05f,  0.55f }
    };
    memcpy(h->A, A_init, sizeof(A_init));

    /* Emission means [rms_z(G), envelope, wavelet_E3]         */
    float mu_init[HMM_STATES][HMM_OBS_DIM] = {
    /* smooth    */ {0.05f,  0.03f,  0.0002f},
    /* worn      */ {0.15f,  0.12f,  0.002f },
    /* cobblest. */ {0.30f,  0.25f,  0.010f },
    /* pothole   */ {0.60f,  0.80f,  0.040f },
    /* speedbump */ {0.40f,  0.60f,  0.020f }
    };
    memcpy(h->mu, mu_init, sizeof(mu_init));

    /* Emission std deviations                                 */
    float sigma_init[HMM_STATES][HMM_OBS_DIM] = {
    /* smooth    */ {0.03f,  0.02f,  0.0002f},
    /* worn      */ {0.06f,  0.05f,  0.001f },
    /* cobblest. */ {0.10f,  0.10f,  0.005f },
    /* pothole   */ {0.20f,  0.25f,  0.015f },
    /* speedbump */ {0.15f,  0.20f,  0.010f }
    };
    memcpy(h->sigma, sigma_init, sizeof(sigma_init));

    /* Initial distribution — uniform */
    for (int s = 0; s < HMM_STATES; s++)
        h->alpha[s] = 1.0f / HMM_STATES;

    h->scale = 1.0f;
    h->current_state = 0;
}

/* Log-Gaussian emission probability (numerically stable)      */
static float HMM_Emission(HMM_t *h, int state, float obs[HMM_OBS_DIM])
{
    float log_p = 0.0f;
    for (int d = 0; d < HMM_OBS_DIM; d++) {
        float diff  = obs[d] - h->mu[state][d];
        float sig   = h->sigma[state][d];
        /* log N = -0.5*(diff/sig)² - log(sig) - 0.5*log(2π) */
        log_p += -0.5f*(diff/sig)*(diff/sig) - logf(sig) - 0.9189f;
    }
    return expf(log_p);
}

/*
 * Forward algorithm — one time step.
 * obs: [rms_z, envelope_peak, wavelet_E3]
 * Updates h->alpha and h->current_state.
 */
void HMM_Update(HMM_t *h, float obs[HMM_OBS_DIM])
{
    float alpha_new[HMM_STATES];
    float total = 0.0f;

    for (int j = 0; j < HMM_STATES; j++) {
        float sum = 0.0f;
        for (int i = 0; i < HMM_STATES; i++)
            sum += h->alpha[i] * h->A[i][j];
        alpha_new[j] = sum * HMM_Emission(h, j, obs);
        total += alpha_new[j];
    }

    /* Scale to avoid underflow */
    h->scale = (total > 1e-30f) ? total : 1e-30f;
    float best = -1.0f;
    for (int j = 0; j < HMM_STATES; j++) {
        h->alpha[j] = alpha_new[j] / h->scale;
        if (h->alpha[j] > best) {
            best = h->alpha[j];
            h->current_state = (uint8_t)j;
        }
    }
}
