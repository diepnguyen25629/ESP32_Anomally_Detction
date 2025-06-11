#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include "driver/i2c.h"

#include "my_i2c.h"
#include "mpu6500.h"
#include "sd_card_spi.h"

static const char *TAG = "MAIN"; 

// Khởi tạo các chân giao tiếp I2C
#define I2C_MASTER_NUM I2C_NUM_0  
#define I2C_SDA_GPIO GPIO_NUM_17
#define I2C_SCL_GPIO GPIO_NUM_16

// Khởi tạo các chân button, led ghi dũ liệu và led kết nối đến GUI
#define BUTTON_GPIO GPIO_NUM_0
#define LED_RECORD_GPIO GPIO_NUM_15

#define imu_BUF_LEN 32

mpu6500_config_t mpu6500_cfg = {
    .calibration = {
        .gyro_bias_offset = {.x = -2.0721499, .y = 9.80414906, .z = 0.37228756},
        .accel_bias_offset = {.x = 0.00153134  , .y = 0.00623305, .z = -0.01285148},
        .accel_cal_matric = {
            {1.00048822, 0.00636324, -0.00751618},
            {-0.00930936, 0.99997852, 0.00544903},
            { 0.03680844 , -0.0051126 , 0.98680229}
        }
    },
    .imu_sample_rate = 20,
    .accel_fs = ACCEL_FS_16G,
    .gyro_fs = GYRO_FS_2500DPS

};

// mpu6500_config_t mpu6500_cfg = {
//     .calibration = {
//         .gyro_bias_offset = {.x = -2.07176219, .y = 9.80366151, .z = 0.37255191},
//         .accel_bias_offset = {.x = 0.00132555 , .y = 0.00647594, .z = -0.01120966},
//         .accel_cal_matric = {
//             {1.00089023, 0.00616087, -0.007825},
//             {-0.00973511, 1.00038651, 0.00564757},
//             { 0.035241 , -0.00492608,  0.98840621}
//         }
//     },
//     .imu_sample_rate = 20,
//     .accel_fs = ACCEL_FS_16G,
//     .gyro_fs = GYRO_FS_2500DPS
// };


vector_t accel_data, gyro_data;
bool is_read_data = false;

void app_main(void)
{
    gpio_config_t button_io_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&button_io_cfg);

    gpio_config_t led_io_cfg = {
        .pin_bit_mask = (1ULL << LED_RECORD_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };

    gpio_config(&led_io_cfg);

    gpio_set_level(LED_RECORD_GPIO, 1); // LED OFF (nguồn ngoài, LOW là bật)

    // Khởi tạo giao tiếp I2C
    esp_err_t ret = i2c_master_init(I2C_MASTER_NUM, I2C_SDA_GPIO, I2C_SCL_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
        return;
    }

    // Khởi tạo MPU6500
    ret = i2c_mpu6500_init(&mpu6500_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6500: %s", esp_err_to_name(ret));
        return;
    }


    bool last_button_state = true;
    while (1) {
        bool button_state = gpio_get_level(BUTTON_GPIO);

        // Nút được nhấn (LOW)
        if (!button_state && last_button_state) {
            is_read_data = !is_read_data;
            printf("Read accel and gyro mpu6500: %s\n", is_read_data ? "ON" : "OFF");
            vTaskDelay(pdMS_TO_TICKS(300));  // Chống dội nút
            gpio_set_level(LED_RECORD_GPIO, !is_read_data);  // LED ON (kéo xuống)

        }

        if(is_read_data){
        // Ghi log ra serial
        ret = get_accel_gyro(&accel_data, &gyro_data);
        if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data MPU6500: %s", esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(200)); 
        return;
        }
        printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    accel_data.x, accel_data.y, accel_data.z,
                    gyro_data.x, gyro_data.y, gyro_data.z);
        }
        last_button_state = button_state;
        vTaskDelay(pdMS_TO_TICKS(20));
    }    
}
    