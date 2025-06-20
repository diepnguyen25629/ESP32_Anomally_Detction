#ifndef _WIFI_BT_H
#define _WIFI_BT_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "mpu6500.h"

typedef struct {
    char ssid[32];
    char password[64];
    char ip[16];
} wifi_gui_config_t;

typedef struct {
    accel_fs_t accel_fs;
    gyro_fs_t gyro_fs;

    mpu6500_calibration_t calibration;
    uint16_t imu_sample_rate;
    int audio_sample_rate;
} sensor_config_t;

void bluetooth_init(void);
void bluetooth_disconnect(void);

void wifi_connect(const char *ssid, const char *password);
void wifi_disconnect(void); 

int tcp_socket_connect(const char *ip, uint16_t port);

void wait_wifi_config(wifi_gui_config_t *config);
esp_err_t wait_sensor_config(sensor_config_t *config, int sock);
#endif