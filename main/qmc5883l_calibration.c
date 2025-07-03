#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "qmc5883l.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern qmc5883l_t qmc_dev;

void qmc5883l_hardiron_calibration(float *x_offset, float *y_offset, float *z_offset) {
    printf("QMC5883L standalone test: starting 30s calibration...\n");
    int32_t x_min = 32767, x_max = -32768;
    int32_t y_min = 32767, y_max = -32768;
    int32_t z_min = 32767, z_max = -32768;
    const int calib_ms = 30000;
    int elapsed = 0;
    while (elapsed < calib_ms) {
        bool ready = false;
        if (qmc5883l_data_ready(&qmc_dev, &ready) && ready) {
            qmc5883l_raw_data_t raw = {0};
            if (qmc5883l_get_raw_data(&qmc_dev, &raw)) {
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
    *x_offset = (x_max + x_min) / 2.0f;
    *y_offset = (y_max + y_min) / 2.0f;
    *z_offset = (z_max + z_min) / 2.0f;
    printf("Calibration done. Offsets: X=%.1f Y=%.1f Z=%.1f\n", *x_offset, *y_offset, *z_offset);
}
