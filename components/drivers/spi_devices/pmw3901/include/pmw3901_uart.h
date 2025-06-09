#pragma once

#include <stdint.h>
#include <stdbool.h>

// Structure should match what your code expects
typedef struct {
    uint8_t motion;
    int16_t deltaX;
    int16_t deltaY;
    uint16_t shutter; // Not used in UART, set to 0
    uint8_t maxRawData; // Not used in UART, set to 0
    uint8_t minRawData; // Not used in UART, set to 0
    uint8_t rawDataSum; // Not used in UART, set to 0
    uint8_t squal;
} motionBurst_t;

// Initialize UART for PMW3901
bool pmw3901UartInit(void);

// Read a motion packet from UART
bool pmw3901UartReadMotion(motionBurst_t *motion);
