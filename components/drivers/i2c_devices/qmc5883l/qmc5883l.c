/*
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file qmc5883l.c
 *
 * ESP-IDF Driver for 3-axis magnetic sensor QMC5883L
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include "qmc5883l.h"

#define REG_XOUT_L 0x00
#define REG_XOUT_H 0x01
#define REG_YOUT_L 0x02
#define REG_YOUT_H 0x03
#define REG_ZOUT_L 0x04
#define REG_ZOUT_H 0x05
#define REG_STATE  0x06
#define REG_TOUT_L 0x07
#define REG_TOUT_H 0x08
#define REG_CTRL1  0x09
#define REG_CTRL2  0x0a
#define REG_FBR    0x0b
#define REG_ID     0x0d

#define MASK_MODE  0xfe
#define MASK_ODR   0xf3

static bool write_reg(qmc5883l_t *dev, uint8_t reg, uint8_t val) {
    return i2cdevWriteReg8(dev->i2c_dev, dev->addr, reg, 1, &val);
}

static bool read_reg(qmc5883l_t *dev, uint8_t reg, uint8_t *val) {
    return i2cdevReadReg8(dev->i2c_dev, dev->addr, reg, 1, val);
}

bool qmc5883l_init_desc(qmc5883l_t *dev, I2C_Dev *i2c_dev, uint8_t addr) {
    if (!dev || !i2c_dev) return false;
    dev->i2c_dev = i2c_dev;
    i2cdevInit(dev->i2c_dev);
    dev->addr = addr;
    dev->range = QMC5883L_RNG_2;
    return true;
}

bool qmc5883l_reset(qmc5883l_t *dev) {
    if (!dev) return false;
    if (!write_reg(dev, REG_CTRL2, 0x80)) return false;
    dev->range = QMC5883L_RNG_2;
    return true;
}

bool qmc5883l_get_chip_id(qmc5883l_t *dev, uint8_t *id) {
    if (!dev || !id) return false;
    return read_reg(dev, REG_ID, id);
}

bool qmc5883l_set_mode(qmc5883l_t *dev, qmc5883l_mode_t mode) {
    if (!dev || mode > QMC5883L_MODE_CONTINUOUS) return false;
    uint8_t v;
    if (!read_reg(dev, REG_CTRL1, &v)) return false;
    v = (v & 0xfe) | mode;
    return write_reg(dev, REG_CTRL1, v);
}

bool qmc5883l_get_mode(qmc5883l_t *dev, qmc5883l_mode_t *mode) {
    if (!dev || !mode) return false;
    uint8_t v;
    if (!read_reg(dev, REG_CTRL1, &v)) return false;
    *mode = v & 1;
    return true;
}

bool qmc5883l_set_config(qmc5883l_t *dev, qmc5883l_odr_t odr, qmc5883l_osr_t osr, qmc5883l_range_t rng) {
    if (!dev || odr > QMC5883L_DR_200 || osr > QMC5883L_OSR_512 || rng > QMC5883L_RNG_8) return false;
    uint8_t v;
    if (!read_reg(dev, REG_CTRL1, &v)) return false;
    dev->range = rng;
    if (!write_reg(dev, REG_FBR, 1)) return false;
    v = (v & 0x03) | ((odr & 3) << 2) | ((rng & 1) << 4) | ((osr & 3) << 6);
    return write_reg(dev, REG_CTRL1, v);
}

bool qmc5883l_get_config(qmc5883l_t *dev, qmc5883l_odr_t *odr, qmc5883l_osr_t *osr, qmc5883l_range_t *rng) {
    if (!dev || !odr || !osr || !rng) return false;
    uint8_t v;
    if (!read_reg(dev, REG_CTRL1, &v)) return false;
    *odr = (v >> 2) & 3;
    *osr = (v >> 6) & 3;
    *rng = (v >> 4) & 1;
    return true;
}

bool qmc5883l_set_int(qmc5883l_t *dev, bool enable) {
    if (!dev) return false;
    return write_reg(dev, REG_CTRL2, enable ? 1 : 0);
}

bool qmc5883l_get_int(qmc5883l_t *dev, bool *enable) {
    if (!dev || !enable) return false;
    uint8_t v;
    if (!read_reg(dev, REG_CTRL2, &v)) return false;
    *enable = v & 1;
    return true;
}

bool qmc5883l_data_ready(qmc5883l_t *dev, bool *ready) {
    if (!dev || !ready) return false;
    uint8_t v;
    if (!read_reg(dev, REG_STATE, &v)) return false;
    *ready = v & 1;
    return true;
}

bool qmc5883l_get_raw_data(qmc5883l_t *dev, qmc5883l_raw_data_t *raw) {
    if (!dev || !raw) return false;
    return i2cdevReadReg8(dev->i2c_dev, dev->addr, REG_XOUT_L, 6, (uint8_t *)raw);
}

bool qmc5883l_raw_to_mg(qmc5883l_t *dev, qmc5883l_raw_data_t *raw, qmc5883l_data_t *data) {
    if (!dev || !raw || !data) return false;
    float f = (dev->range == QMC5883L_RNG_2 ? 2000.0f : 8000.0f) / 32768.0f;
    data->x = raw->x * f;
    data->y = raw->y * f;
    data->z = raw->z * f;
    return true;
}

bool qmc5883l_get_data(qmc5883l_t *dev, qmc5883l_data_t *data) {
    qmc5883l_raw_data_t raw;
    if (!qmc5883l_get_raw_data(dev, &raw)) return false;
    return qmc5883l_raw_to_mg(dev, &raw, data);
}

bool qmc5883l_get_raw_temp(qmc5883l_t *dev, int16_t *temp) {
    if (!dev || !temp) return false;
    return i2cdevReadReg8(dev->i2c_dev, dev->addr, REG_TOUT_L, 2, (uint8_t *)temp);
}

/* ---------- calibration fuctions ---------- */
// Calibration routines for QMC5883L
#include <math.h>
void qmc5883l_hardiron_calibration(qmc5883l_t *dev, void (*printfn)(const char*, ...), uint16_t seconds) {
    if (!dev) return;
    if (printfn) printfn("QMC5883L standalone test: starting 30s Hard-Iron calibration...\n");
    // Sleep for 3 seconds before calibration
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    int32_t x_min = 32767, x_max = -32768;
    int32_t y_min = 32767, y_max = -32768;
    int32_t z_min = 32767, z_max = -32768;
    const int calib_ms = seconds * 1000;
    int elapsed = 0;
    while (elapsed < calib_ms) {
        bool ready = false;
        if (qmc5883l_data_ready(dev, &ready) && ready) {
            qmc5883l_raw_data_t raw = {0};
            if (qmc5883l_get_raw_data(dev, &raw)) {
                if (raw.x < x_min) x_min = raw.x;
                if (raw.x > x_max) x_max = raw.x;
                if (raw.y < y_min) y_min = raw.y;
                if (raw.y > y_max) y_max = raw.y;
                if (raw.z < z_min) z_min = raw.z;
                if (raw.z > z_max) z_max = raw.z;
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
        elapsed += 50;
    }
    dev->x_offset = (x_max + x_min) / 2.0f;
    dev->y_offset = (y_max + y_min) / 2.0f;
    dev->z_offset = (z_max + z_min) / 2.0f;
    if (printfn) printfn("Calibration SI: X=%.1f Y=%.1f Z=%.1f\n", dev->x_offset, dev->y_offset, dev->z_offset);
}

int qmc5883l_softiron_calibration(qmc5883l_t *dev, void (*printfn)(const char*, ...), uint16_t seconds) {
    if (!dev) return -1;
    if (printfn) printfn("QMC5883L: starting soft-iron calib (ellipse fit)...\n");
    // Sleep for 3 seconds before calibration
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    const int max_samples = 512;
    int16_t x_samples[max_samples];
    int16_t y_samples[max_samples];
    int n = 0;
    const int calib_ms = seconds * 1000;
    int elapsed = 0;
    while (elapsed < calib_ms && n < max_samples) {
        bool ready = false;
        if (qmc5883l_data_ready(dev, &ready) && ready) {
            qmc5883l_raw_data_t raw = {0};
            if (qmc5883l_get_raw_data(dev, &raw)) {
                x_samples[n] = raw.x;
                y_samples[n] = raw.y;
                n++;
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
        elapsed += 50;
    }
    if (n < 10) {
        if (printfn) printfn("Not enough samples for soft-iron calibration!\n");
        dev->scale_x = 1.0f;
        dev->scale_y = 1.0f;
        dev->theta = 0.0f;
        return -1;
    }

    float x_mean = 0, y_mean = 0;
    for (int i = 0; i < n; i++) {
        x_mean += x_samples[i];
        y_mean += y_samples[i];
    }
    x_mean /= n;
    y_mean /= n;

    float Sxx = 0, Syy = 0, Sxy = 0;
    for (int i = 0; i < n; i++) {
        float dx = x_samples[i] - x_mean;
        float dy = y_samples[i] - y_mean;
        Sxx += dx * dx;
        Syy += dy * dy;
        Sxy += dx * dy;
    }
    Sxx /= n;
    Syy /= n;
    Sxy /= n;

    float theta_est = 0.5f * atan2f(2 * Sxy, Sxx - Syy);

    float ct = cosf(theta_est);
    float st = sinf(theta_est);
    float S1 = Sxx * ct * ct + 2 * Sxy * ct * st + Syy * st * st;
    float S2 = Sxx * st * st - 2 * Sxy * ct * st + Syy * ct * ct;
    float a = sqrtf(S1);
    float b = sqrtf(S2);
    if (a < b) { float tmp = a; a = b; b = tmp; theta_est += (float)M_PI_2; }
    if (b < 1e-6f) b = 1.0f;

    dev->scale_x = 1.0f;
    dev->scale_y = a / b;
    dev->theta = theta_est;
    // if (printfn) printfn("Soft-iron. Scale X=%.3f, Y=%.3f, %.2f\n", dev->scale_x, dev->scale_y, dev->theta * 180.0f / (float)M_PI);
    return 0;
}

// Read and apply calibration to QMC5883L magnetometer
// Returns true if new data was read and outputs are updated
bool qmc5883l_read_mag(qmc5883l_t *dev, float *x_cal, float *y_cal, float *z_cal) {
    if (!dev || !x_cal || !y_cal || !z_cal) return false;
    bool mag_ready = false;
    if (qmc5883l_data_ready(dev, &mag_ready) && mag_ready) {
        qmc5883l_raw_data_t mag_raw = {0};
        if (qmc5883l_get_raw_data(dev, &mag_raw)) {
            // Hard-iron correction
            float x_corr = mag_raw.x - dev->x_offset;
            float y_corr = mag_raw.y - dev->y_offset;
            float z_corr = mag_raw.z - dev->z_offset;
            // Soft-iron correction (2D)
            float cos_t = cosf(dev->theta), sin_t = sinf(dev->theta);
            *x_cal = dev->scale_x * (cos_t * x_corr + sin_t * y_corr);
            *y_cal = dev->scale_y * (-sin_t * x_corr + cos_t * y_corr);
            *z_cal = z_corr; // No soft-iron correction for Z
            return true;
        }
    }
    return false;
}