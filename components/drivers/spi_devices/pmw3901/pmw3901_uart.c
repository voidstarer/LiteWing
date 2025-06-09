/*
 * pmw3901_uart.c: PMW3901 UART packet parser for motion data
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "pmw3901.h"
#include <string.h>
#include <stdbool.h>

#define PMW3901_UART_BAUDRATE 19200
#define PMW3901_UART_PORT     UART_NUM_1
#define PMW3901_UART_RX_PIN   GPIO_NUM_16  // Change this to your actual RX pin

static bool isInit = false;

bool pmw3901UartInit(void)
{
    if (isInit) return true;

    uart_config_t uart_config = {
        .baud_rate = PMW3901_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(PMW3901_UART_PORT, &uart_config);

    // Set only RX pin, TX is unused
    uart_set_pin(PMW3901_UART_PORT, UART_PIN_NO_CHANGE, PMW3901_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install UART driver with RX buffer only
    uart_driver_install(PMW3901_UART_PORT, 256, 0, 0, NULL, 0);

    isInit = true;
    return true;
}

static uint8_t calcChecksum(const uint8_t *data, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

bool pmw3901UartReadMotion(motionBurst_t *motion)
{
    uint8_t buf[16];
    int idx = 0;

    while (1) {
        int n = uart_read_bytes(PMW3901_UART_PORT, &buf[idx], 1, pdMS_TO_TICKS(20));
        if (n == 1) {
            if (idx == 0 && buf[0] != 0xFE) continue; // Wait for header
            idx++;
            if (idx == 9) {
                if (buf[8] == 0xAA && buf[1] == 0x04) {
                    uint8_t checksum = calcChecksum(&buf[2], 4);
                    if (checksum == buf[6]) {
                        int16_t dx = (buf[2] << 8) | buf[3];
                        int16_t dy = (buf[4] << 8) | buf[5];
                        memset(motion, 0, sizeof(*motion));
                        motion->deltaX = dx;
                        motion->deltaY = dy;
                        motion->squal = buf[7];
                        return true;
                    }
                }
                // Shift buffer to try again
                memmove(buf, buf + 1, 8);
                idx = 8;
                if (buf[0] != 0xFE) idx = 0; // Resync if header lost
            }
        } else {
            break; // Timeout
        }
    }
    return false;
}

