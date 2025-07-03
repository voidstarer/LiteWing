#ifndef QMC5883L_CALIBRATION_H
#define QMC5883L_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

void qmc5883l_hardiron_calibration(float *x_offset, float *y_offset, float *z_offset);

#endif // QMC5883L_CALIBRATION_H
