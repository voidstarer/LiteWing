#ifndef MAG_UTILS_H
#define MAG_UTILS_H

#include <math.h>
#include "sdkconfig.h"

/**
 * Calculate tilt-compensated heading from magnetometer and accelerometer data
 * @param mx, my, mz: Magnetometer readings
 * @param ax, ay, az: Accelerometer readings  
 * @param mounting_angle_deg: Magnetometer mounting angle in degrees
 * @return Heading in degrees
 */
float mag_calculate_heading(float mx, float my, float mz, float ax, float ay, float az, float mounting_angle_deg);

/**
 * Get the configured magnetometer mounting angle from sdkconfig
 * @return Mounting angle in degrees
 */
static inline float mag_get_mounting_angle(void) {
    return CONFIG_MAG_MOUNTING_ANGLE_DEG;
}

#endif // MAG_UTILS_H
