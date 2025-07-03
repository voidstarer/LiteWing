#include "mag_utils.h"

float mag_calculate_heading(float mx, float my, float mz, float ax, float ay, float az, float mounting_angle_deg) {
    // Apply magnetometer mounting angle rotation
    float angle_rad = mounting_angle_deg * (float)M_PI / 180.0f;
    float cos_angle = cosf(angle_rad);
    float sin_angle = sinf(angle_rad);
    
    // Rotate magnetometer readings to align with drone coordinate system
    float drone_mx = mx * cos_angle - my * sin_angle;
    float drone_my = mx * sin_angle + my * cos_angle;
    float drone_mz = mz;  // Z-axis typically unchanged
    
    // Calculate pitch and roll (in radians)
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    // Tilt compensation using rotated magnetometer values
    float Xh = drone_mx * cosf(pitch) + drone_mz * sinf(pitch);
    float Yh = drone_mx * sinf(roll) * sinf(pitch) + drone_my * cosf(roll) - drone_mz * sinf(roll) * cosf(pitch);

    float headingRad = atan2f(Yh, Xh);
    float headingDeg = headingRad * 180.0f / (float)M_PI;
    
    return headingDeg;
}
