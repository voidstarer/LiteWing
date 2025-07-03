#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "qmc5883l.h"
#include "i2cdev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h" // Correct include for MPU6050 support
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_netif.h"
#include "esp_timer.h"

#define WIFI_SSID      CONFIG_WIFI_BASE_SSID
#define WIFI_PASS      CONFIG_WIFI_PASSWORD

// #define WIFI_SSID      "Subnet Labs 2.4g"
// #define WIFI_PASS      "Vinod@10"
#define UDP_REMOTE_IP  "192.168.1.6"
// #define UDP_REMOTE_IP "192.168.2.187"
#define UDP_REMOTE_PORT 2000
#define UDP_LOCAL_PORT  2000

static int udp_sock = -1;
static struct sockaddr_in udp_dest_addr;

void udp_printf(const char *fmt, ...) {
    char buf[150];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len < 0) return;
    buf[sizeof(buf) - 1] = 0; // Always null-terminate
    // Print to console
    printf("%s", buf);
    fflush(stdout);
    // Send over UDP if socket is valid
    if (udp_sock >= 0) {
        sendto(udp_sock, buf, len, 0, (struct sockaddr *)&udp_dest_addr, sizeof(udp_dest_addr));
    }
}

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        udp_printf("WiFi disconnected, restarting...\n");
        esp_restart();
    }
}

static void wifi_init_and_wait(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    udp_printf("Connecting to WiFi SSID: %s...\n", WIFI_SSID);
    // Wait for IP
    esp_netif_ip_info_t ip_info;
    const int max_wait_ms = 10000; // 10 seconds
    int waited = 0;
    bool got_ip = false;
    while (waited < max_wait_ms) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        waited += 500;
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
            char ip_str[16];
            snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d",
                ip4_addr1(&ip_info.ip), ip4_addr2(&ip_info.ip),
                ip4_addr3(&ip_info.ip), ip4_addr4(&ip_info.ip));
            udp_printf("WiFi connected, IP: %s\n", ip_str);
            got_ip = true;
            break;
        }
    }
    if (!got_ip) {
        udp_printf("WiFi connection timeout, restarting...\n");
        esp_restart();
    }
}

static void udp_socket_init(void) {
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_sock < 0) {
        udp_printf("Unable to create UDP socket\n");
        esp_restart();
    }
    struct sockaddr_in local_addr = {0};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(UDP_LOCAL_PORT);
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(udp_sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        udp_printf("UDP bind failed\n");
        close(udp_sock);
        esp_restart();
    }
    udp_dest_addr.sin_family = AF_INET;
    udp_dest_addr.sin_port = htons(UDP_REMOTE_PORT);
    udp_dest_addr.sin_addr.s_addr = inet_addr(UDP_REMOTE_IP);
    udp_printf("UDP socket ready, sending to %s:%d\n", UDP_REMOTE_IP, UDP_REMOTE_PORT);
}

// #define QMC5883L_I2C_ADDR_DEF 0x0D

static qmc5883l_t qmc_dev;

// Simple test to help identify which physical axis corresponds to accel X, Y, Z
// Move the board so that one axis points up (+1g), others should be near 0
static void accel_axis_identification_task(void) {
    udp_printf("\n=== Accelerometer Axis Identification ===\n");
    udp_printf("Move the board so that each axis points up (+1g), then down (-1g).\n");
    udp_printf("Observe which value (ax, ay, az) changes.\n");
    udp_printf("Press reset or power cycle to exit this test.\n\n");
    int16_t ax, ay, az;
    while (1) {
        mpu6050GetAcceleration(&ax, &ay, &az);
        float fax = ax / 16384.0f;
        float fay = ay / 16384.0f;
        float faz = az / 16384.0f;
        udp_printf("Accel: ax=%.2f g\t ay=%.2f g\t az=%.2f g\r", fax, fay, faz);
        fflush(stdout);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void test_qmc5883l(void)
{
    // Connect to WiFi and start UDP socket before any initialization
    wifi_init_and_wait();
    udp_socket_init();
    // Init I2C and QMC5883L
    i2cdevInit(I2C0_DEV);
    if (!qmc5883l_init_desc(&qmc_dev, I2C0_DEV, QMC5883L_I2C_ADDR_DEF)) {
    udp_printf("QMC5883L init failed!\n");
        return;
    }
    qmc5883l_reset(&qmc_dev);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    qmc5883l_set_config(&qmc_dev, QMC5883L_DR_50, QMC5883L_OSR_512, QMC5883L_RNG_8);
    qmc5883l_set_mode(&qmc_dev, QMC5883L_MODE_CONTINUOUS);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Init MPU6050 (no struct, just I2C port)
    mpu6050Init(I2C0_DEV);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // Wake up MPU6050 (clear sleep bit if needed)
    if (mpu6050GetSleepEnabled()) {
        mpu6050SetSleepEnabled(false);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        udp_printf("MPU6050 was in sleep mode, now woken up.\n");
    }
    // Check MPU6050 connection
    if (!mpu6050TestConnection()) {
        udp_printf("MPU6050 connection failed! Check wiring and address.\n");
        udp_printf("Rebooting\n");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        return;
    } else {
        udp_printf("MPU6050 connection OK.\n");
    }
    // accel_axis_identification_task();

    bool enable_calibration = false; // Set to true to perform calibration
    if (enable_calibration) {
        qmc5883l_hardiron_calibration(&qmc_dev, udp_printf, 30);
        qmc5883l_softiron_calibration(&qmc_dev, udp_printf, 30);
        udp_printf("Soft-iron. Scale X=%.3f, Y=%.3f, Z=%.2f\n", qmc_dev.scale_x, qmc_dev.scale_y, qmc_dev.theta * 180.0f / (float)M_PI);
        udp_printf("Restarting...\n");
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait for calibration to finish
        esp_restart();
    } else {

        // Hardcoded calibration values from previous run
        /*
        // 203 - earlier
        qmc_dev.x_offset = -8386.0f;
        qmc_dev.y_offset = -4650.5f;
        qmc_dev.z_offset = -183.0f;
        qmc_dev.scale_x = 1.0f;
        qmc_dev.scale_y = 1.297f;
        qmc_dev.theta = 45.75f * (float)M_PI / 180.0f; // Convert degrees to radians
        //203 drone
        qmc_dev.x_offset = -8413.5f;
        qmc_dev.y_offset = -4386.5f;
        qmc_dev.z_offset = -265.0f;
        qmc_dev.scale_x = 1.0f;
        qmc_dev.scale_y = 1.917f;
        qmc_dev.theta = 0.54f * (float)M_PI / 180.0f; // Convert degrees to radians
        */
        //202 drone
        qmc_dev.x_offset = 1832.0f;
        qmc_dev.y_offset = -3054.5f;
        qmc_dev.z_offset = 1543.0f;
        qmc_dev.scale_x = 1.000f;
        qmc_dev.scale_y = 1.130f;
        qmc_dev.theta = -73.09f * (float)M_PI / 180.0f; // Convert degrees to radians
        udp_printf("Hardcoded offsets: X=%.1f Y=%.1f Z=%.1f, scale_x=%.3f, scale_y=%.3f, theta=%.2f deg\n",
            qmc_dev.x_offset, qmc_dev.y_offset, qmc_dev.z_offset, qmc_dev.scale_x, qmc_dev.scale_y, qmc_dev.theta * 180.0f / (float)M_PI);
    }

    udp_printf("QMC5883L+MPU6050 standalone test: reading tilt-compensated heading...\n");
    TickType_t lastPrintTick = xTaskGetTickCount();
    int16_t latest_ax = 0, latest_ay = 0, latest_az = 0;
    float latest_x_cal = 0, latest_y_cal = 0, latest_z_cal = 0;
    while (1) {
        // Update magnetometer and get calibrated values
        qmc5883l_read_mag(&qmc_dev, &latest_x_cal, &latest_y_cal, &latest_z_cal);
        // Always update accelerometer
        mpu6050GetAcceleration(&latest_ax, &latest_ay, &latest_az);

        TickType_t now = xTaskGetTickCount();
        if ((now - lastPrintTick) * portTICK_PERIOD_MS >= 250) {
            // Convert to g (assuming 16,384 LSB/g for +/-2g)
            float ax = latest_ax / 16384.0f;
            float ay = latest_ay / 16384.0f;
            float az = latest_az / 16384.0f;

            // Calculate pitch and roll (in radians)
            float roll = atan2f(ay, az);
            float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

            // Tilt compensation
            float Xh = latest_x_cal * cosf(pitch) + latest_z_cal * sinf(pitch);
            float Yh = latest_x_cal * sinf(roll) * sinf(pitch) + latest_y_cal * cosf(roll) - latest_z_cal * sinf(roll) * cosf(pitch);

            float headingRad = atan2f(Yh, Xh);
            float headingDeg = headingRad * 180.0f / (float)M_PI;
            if (headingDeg < 0) headingDeg += 360.0f;

            udp_printf("X=%f Y=%f Z=%f|ax=%.2f ay=%.2f az=%.2f|head: %.2f deg\n",
                latest_x_cal, latest_y_cal, latest_z_cal, ax, ay, az, headingDeg);
            lastPrintTick = now;
        }
        // No vTaskDelay here: run as fast as possible, only print every 250ms
    }
}
