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
#include "driver/i2s_std.h"
#include "nvs_flash.h"

#include "my_i2c.h"
#include "mpu6500.h"
#include "bmp280.h"
#include "inmp441_i2s.h"
#include "ds18b20_1_wire.h"
#include "sd_card_spi.h"
#include "wifi_bt.h"

#include "esp_spp_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"

#include <sys/socket.h>

static const char *TAG = "MAIN"; 

// Khởi tạo các chân giao tiếp I2C
#define I2C_MASTER_NUM I2C_NUM_0  
#define I2C_SDA_GPIO GPIO_NUM_17
#define I2C_SCL_GPIO GPIO_NUM_16

// Khởi tạo chân giao tiếp 1-Wire
#define ONE_WIRE_GPIO 4

// Khởi tạo các chân button, led ghi dũ liệu và led kết nối đến GUI
#define BUTTON_GPIO GPIO_NUM_0
#define LED_RECORD_GPIO GPIO_NUM_15
#define LED_CONNECT_GPIO GPIO_NUM_22

// Số lượng lô dữ liệu cảm biến ghi vào thẻ SD  
#define IMUPT_WRITE_BATCH 75
#define AUDIO_BUF_LEN 512
#define AUDIO_WRITE_BATCH 16 // Ghi 16 buffer 512 mẫu mỗi lần 

// Số lượng packet gửi đến cho GUI 
#define MAX_SEND_IMUPT 25
#define MAX_SEND_AUDIO 2

#define PACKET_MAGIC            0xABCD      // Vị trí bắt đầu của packet
#define PACKET_TYPE_IMUPT       0x01
#define PACKET_TYPE_AUDIO       0x02

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

// Cấu hình các tham số hiệu chỉnh, phạm vi đo và tần số lấy mẫu cho cảm biến mpu6500
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
    .imu_sample_rate = 100,
    .accel_fs = ACCEL_FS_16G,
    .gyro_fs = GYRO_FS_2500DPS

};

// Cấu hình các tham số và tần số lấy mẫu cho cảm biến bmp280
bmp280_config_t bmp280_cfg = {
        .mode = BMP280_MODE_NORMAL,
        .filter = BMP280_FILTER_8,
        .oversampling_pressure = BMP280_ULTRA_HIGH_RES,     
        .oversampling_temperature = BMP280_LOW_POWER,  
        .stanby = BMP280_STANDBY_250, 
        .pre_sample_rate = 5
};

// Cấu hình độ phân giải và tần số lấy mẫu của cảm biến ds18b20
ds18b20_config_t ds18b20_cfg = {
        .temp_resol = RESOLUTION_10_BIT,
        .temp_sample_rate = 1
};

// Cấu hình các chân giao tiếp I2S của cảm biến inmp441
inmp441_config_t inmp441_cfg = {
    .audio_sample_rate = 16000,
    .bclk_io_num = 26,
    .ws_io_num = 27,
    .data_in_io_num = 25,
    .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT
};

// Cấu hình các chân giao tiếp SPI của sd card 
sd_spi_config_t sd_cfg = {
    .pin_miso = 32,
    .pin_mosi = 23,
    .pin_clk = 33,
    .pin_cs = 5,
    .mount_point = "/data"
};

typedef struct {
    vector_t accel_data, gyro_data;
    float temp_data, pre_data;
}imupt_data_t;

typedef struct {
    int32_t audio_buf[AUDIO_BUF_LEN];
}audio_data_t;

// Bộ đệm và biến đếm để ghi dữ liệu imupt theo batch vào sd card
imupt_data_t imupt_batch[IMUPT_WRITE_BATCH];
int imupt_batch_index = 0;

// Bộ đệm và biến đếm để ghi dữ liệu audio theo batch vào sd card
audio_data_t audio_batch[AUDIO_WRITE_BATCH];
int audio_batch_index = 0; 

// Bộ đệm và biến đếm để gửi dữ liệu imupt theo batch đến GUI
imupt_data_t imupt_batch_gui[IMUPT_WRITE_BATCH];
int imupt_batch_gui_index = 0;

// Bộ đệm và biến đếm để gửi dữ liệu audio theo batch đến GUI
audio_data_t audio_batch_gui[AUDIO_WRITE_BATCH];
int audio_batch_gui_index = 0; 

// Phần header dữ liệu nhị phân ghi vào sd và gửi đến GUI 
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint16_t type;
    uint16_t length;
} packet_header_t;

QueueHandle_t imupt_queue;                              // Hàng đợi chứa dữ liệu cảm biến để ghi dữ liệu vào thẻ sd 
QueueHandle_t audio_queue;
QueueHandle_t imupt_queue_gui;                           // Hàng đợi chứa dữ liệu cảm biến để gửi đến GUI
QueueHandle_t audio_queue_gui;

volatile bool is_recording = false;                     // Trạng thái record vào sd card 
volatile bool is_connect_gui = false;                   // Trạng thái kêt nối đến GUI

// Khởi tạo các cảm biến accelerometer, gyroscope, pressure, temperate rồi đưa dữ liệu vào hàng đợi
void read_imupt_task(void *pvParameter)
{   
    imupt_data_t imupt_data; 

    // Khởi tạo giao tiếp I2C
    esp_err_t ret = i2c_master_init(I2C_MASTER_NUM, I2C_SDA_GPIO, I2C_SCL_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        vTaskDelete(NULL);
        return;
    }

    // Khởi tạo MPU6500 
    ret = i2c_mpu6500_init(&mpu6500_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6500");
        vTaskDelete(NULL);
        return;
    }

    // Khởi tạo BMP280
    ret = bmp280_init(&bmp280_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMP280");
        vTaskDelete(NULL);
        return;
    }

    // Khởi tạo DS18B20
    ret = ds18b20_init(ONE_WIRE_GPIO);
    if(ret != ESP_OK){
        ESP_LOGE(TAG, " Failed to initialize DS18B20");
        vTaskDelete(NULL);
        return;
    }

    DeviceAddress sensorAddr;
    ds18b20_reset_search();                     // Reset trạng thái tìm kiếm cảm biến DS18B20
    // Tìm kiếm 1 thiết bị DS18B20
    if(!ds18b20_search(sensorAddr, true)){
        ESP_LOGE(TAG, " Failed to initialize DS18B20");
        vTaskDelete(NULL);
        return;
    }
    ds18b20_setResolution(&sensorAddr, 1, ds18b20_cfg.temp_resol);                         // Cấu hình độ phân giải cho cảm biến DS18B20 
 
    // Khoảng thời gian giữa các lần lặp dựa theo tần số của IMU
    int imu_rate_hz = mpu6500_cfg.imu_sample_rate;
    TickType_t loop_interval = pdMS_TO_TICKS(1000 / imu_rate_hz);
    // Số vòng cần đợi trước khi cập nhật giá trị pressure và temperate
    int pre_update_interval = imu_rate_hz / bmp280_cfg.pre_sample_rate;
    int temp_update_interval = imu_rate_hz / ds18b20_cfg.temp_sample_rate;

    int pre_count = 0;
    int temp_count = 0;

    bool temp_conversion_started = false;
    TickType_t temp_conversion_start_time = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        if (get_accel_gyro(&imupt_data.accel_data, &imupt_data.gyro_data) != ESP_OK)
            ESP_LOGE(TAG, "Failed to read MPU6500");

        if (pre_count >= pre_update_interval) {
            if (bmp280_read_pre(&imupt_data.pre_data) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read BMP280");
            }
            pre_count = 0;
        }

        // Bắt đầu chuyển đổi nếu đến thời điểm
        if (!temp_conversion_started && temp_count >= temp_update_interval) {
            ds18b20_requestTemperatures();
            temp_conversion_start_time = xTaskGetTickCount();
            temp_conversion_started = true;
            temp_count = 0;
        }

        // Sau 750ms kể từ khi bắt đầu conversion, thì đọc dữ liệu
        if (temp_conversion_started &&
            (xTaskGetTickCount() - temp_conversion_start_time >= pdMS_TO_TICKS(1000))) {

            if (ds18b20_getTempC(&sensorAddr, &imupt_data.temp_data) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read DS18B20");
            }
            temp_conversion_started = false; // Reset trạng thái
        }

        if (xQueueSend(imupt_queue, &imupt_data, pdMS_TO_TICKS(10)) != pdTRUE) {
           // printf("imupt_queue full ");
        } 

        if (imupt_queue_gui != NULL){
            if (xQueueSend(imupt_queue_gui, &imupt_data, pdMS_TO_TICKS(10)) != pdTRUE) {
               //printf(" imupt_queue_gui full ");
            }
        }
        pre_count++;
        temp_count++;

        vTaskDelayUntil(&xLastWakeTime, loop_interval);
    }
}

// Khởi tạo cảm biến audio rồi đưa dữ liệu vào hàng đợi
void read_audio_task(void *pvParameter)
{
    audio_data_t audio_data;
    //Khởi tạo INMP441
    esp_err_t ret = inmp441_i2s_init(&inmp441_cfg);
    if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize INMP441");
    vTaskDelete(NULL);
    return;
    }

    while (1) {
        int bytes = inmp441_i2s_read(audio_data.audio_buf, sizeof(audio_data.audio_buf));

        if (bytes > 0 ) {
            if (xQueueSend(audio_queue, &audio_data, pdMS_TO_TICKS(10)) != pdTRUE) {
                //printf("audio_queue full ");
            }
            if (audio_queue_gui != NULL) {
                if (xQueueSend(audio_queue_gui, &audio_data, pdMS_TO_TICKS(10)) != pdTRUE) {
                   // printf("audio_queue_gui full");
                }
            }    
        }
    }
}

// Lấy dữ liệu từ hàng đợi rồi đưa vào buffer để truyền theo batch 
//    Sau khi buffer đầy thì ghi xuóng file của sd card
void write_data_to_sd_task(void *pvParameter) {
    FILE *data_file = NULL;
    static char filepath[64];
    imupt_data_t imupt_data_recv;
    audio_data_t audio_data_recv;
    int64_t last_reopen_us = 0;
    const int64_t REOPEN_INTERVAL_US = 120 * 1000 * 1000;

    while(1) {
        int now = esp_timer_get_time();

        // Nếu button được nhấn thì bắt đầu ghi vào sd card 
        if (is_recording && data_file == NULL) {
            // Tạo các file không trùng nhau trong sd card  
            sd_spi_generate_unique_filename("/data", "data", "bin", filepath, sizeof(filepath));
            data_file = sd_spi_start_write(filepath);
            if (!data_file) {
                ESP_LOGE(TAG, "Failed to open data file");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
        }

        // Nếu button được nhấn lần nữa thì kết thúc ghi        
        if (!is_recording && data_file != NULL) {
            sd_spi_stop_write(data_file);
            data_file = NULL;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Fflush file theo chu kì 
        if (data_file != NULL && now - last_reopen_us > REOPEN_INTERVAL_US) {
            ESP_LOGI(TAG, "Cycling file: flushing and reopening to prevent data loss");

            fflush(data_file);
            last_reopen_us = now;
            continue;
        }

        if (is_recording && data_file != NULL) {
            while (xQueueReceive(imupt_queue, &imupt_data_recv, 0) == pdTRUE) {
                imupt_batch[imupt_batch_index++] = imupt_data_recv;

                if (imupt_batch_index >= IMUPT_WRITE_BATCH) {
                    packet_header_t header = {
                    .magic = PACKET_MAGIC,
                    .type = PACKET_TYPE_IMUPT,
                    .length = imupt_batch_index * sizeof(imupt_data_t),
                };
                sd_spi_write_binary_buffer(data_file, &header, sizeof(header), 1);
                sd_spi_write_binary_buffer(data_file, imupt_batch, sizeof(imupt_data_t), imupt_batch_index);
                imupt_batch_index = 0;
                }
            }

            while (xQueueReceive(audio_queue, &audio_data_recv, 0) == pdTRUE) {
                audio_batch[audio_batch_index++] = audio_data_recv;

                if (audio_batch_index >= AUDIO_WRITE_BATCH) {
                packet_header_t header = {
                    .magic = PACKET_MAGIC,
                    .type = PACKET_TYPE_AUDIO,
                    .length = audio_batch_index * sizeof(audio_data_t),
                };
                sd_spi_write_binary_buffer(data_file, &header, sizeof(header), 1);
                sd_spi_write_binary_buffer(data_file, audio_batch, sizeof(audio_data_t), audio_batch_index);
                audio_batch_index = 0;
                }
            }                   
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// Khởi tạo các chân gpio của LED và button 
void gpio_init()
{
    gpio_config_t button_io_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&button_io_cfg);

    gpio_config_t led_record_io_cfg = {
        .pin_bit_mask = (1ULL << LED_RECORD_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };

    gpio_config(&led_record_io_cfg);

    gpio_set_level(LED_CONNECT_GPIO, 1); // LED OFF (nguồn ngoài, LOW là bật)

        gpio_config_t led_connect_io_cfg = {
        .pin_bit_mask = (1ULL << LED_CONNECT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };

    gpio_config(&led_connect_io_cfg);

    gpio_set_level(LED_CONNECT_GPIO, 1); // LED OFF (nguồn ngoài, LOW là bật)
}

// Kiểm tra trạng thái của button 
void button_monitor_task(void *pvParameter)
{
    bool last_button_state = true;
    while (1) {
        bool button_state = gpio_get_level(BUTTON_GPIO);

        // Nút được nhấn (LOW)
        if (!button_state && last_button_state) {
            is_recording = !is_recording;
            xQueueReset(imupt_queue);   // Xoá dữ liệu cũ
            xQueueReset(audio_queue);
            ESP_LOGI(TAG, "Recording: %s", is_recording ? "ON" : "OFF");
            vTaskDelay(pdMS_TO_TICKS(200));  // Chống dội nút
        }

        last_button_state = button_state;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void led_blink_task(void *pvParameter)
{
    while (1) {
        if (is_recording) {
            gpio_set_level(LED_RECORD_GPIO, 0);  // LED ON (kéo xuống)
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            gpio_set_level(LED_RECORD_GPIO, 1);  // LED luôn OFF khi không ghi
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (is_connect_gui) {
            gpio_set_level(LED_CONNECT_GPIO, 0);  // LED ON (kéo xuống)
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            gpio_set_level(LED_CONNECT_GPIO, 1);  // LED luôn OFF khi không ghi
            vTaskDelay(pdMS_TO_TICKS(100));
        }       
    }
}

void gui_send_task(void *pvParameter) {
    int sock = *((int *)pvParameter);
    free(pvParameter);

    imupt_data_t imupt_data_gui_recv;
    audio_data_t audio_data_gui_recv;

    while(1) {
        if (!is_connect_gui) {
            break;
        }
        while (xQueueReceive(imupt_queue_gui, &imupt_data_gui_recv, 0) == pdTRUE) {
            imupt_batch_gui[imupt_batch_gui_index++] = imupt_data_gui_recv;
            if (imupt_batch_gui_index >= IMUPT_WRITE_BATCH) {
                packet_header_t header = {
                .magic = PACKET_MAGIC,
                .type = PACKET_TYPE_IMUPT,
                .length = imupt_batch_gui_index * sizeof(imupt_data_t),
            };
            if (send(sock, &header, sizeof(header), 0) < 0 ||
            send(sock, imupt_batch_gui, imupt_batch_gui_index * sizeof(imupt_data_t), 0) < 0) {
                ESP_LOGE(TAG, "Send imupt to GUI failed");
                is_connect_gui = false;
            }
            imupt_batch_gui_index = 0;
            }
        }

        while (xQueueReceive(audio_queue_gui, &audio_data_gui_recv, 0) == pdTRUE) {
            audio_batch_gui[audio_batch_gui_index++] = audio_data_gui_recv;
            if (audio_batch_gui_index >= AUDIO_WRITE_BATCH) {
            packet_header_t header = {
                .magic = PACKET_MAGIC,
                .type = PACKET_TYPE_AUDIO,
                .length = audio_batch_gui_index * sizeof(audio_data_t),
            };

            if (send(sock, &header, sizeof(header), 0) < 0 ||
            send(sock, audio_batch_gui, audio_batch_gui_index * sizeof(audio_data_t), 0) < 0) {
                ESP_LOGE(TAG, "Send audio to GUI failed");
                is_connect_gui = false;
            }
            audio_batch_gui_index = 0;
            }          
        }
        vTaskDelay(pdMS_TO_TICKS(5));     
    }
    close(sock);
    vTaskDelete(NULL);
}

void bt_wifi_config_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Waiting for Bluetooth WiFi config...");
    
    // Chờ semaphore báo đã nhận được cấu hình mới thực hiện cấu hình wifi 
    xSemaphoreTake(wifi_bt_get_semaphore(), portMAX_DELAY);
    wifi_bt_config_t config = wifi_bt_get_config();

    // Hủy kết nối Bluetooth  
    ESP_LOGI(TAG, "Disabling Bluetooth...");

    esp_spp_deinit();
    vTaskDelay(pdMS_TO_TICKS(100));   // Cho SPP thoát hẳn
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Bluetooth disabled");

    // Kết nối WiFi
    wifi_bt_start_wifi(config.ssid, config.password);

    // Kết nối TCP đến GUI 
    int sock = wifi_bt_tcp_connect(config.gui_ip, 12345);
    if (sock < 0) {
        vTaskDelete(NULL);
        return;
    } 
    int *sock_ptr = malloc(sizeof(int));
    if (sock_ptr == NULL) {
        ESP_LOGE(TAG, "Malloc failed for socket ptr");
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    *sock_ptr = sock;

    imupt_queue_gui = xQueueCreate(50, sizeof(imupt_data_t));
    audio_queue_gui = xQueueCreate(20, sizeof(audio_data_t));

    if(!imupt_queue_gui || !audio_queue_gui){
        ESP_LOGE(TAG, "Fail to create queue GUI");
        return;
    }

    is_connect_gui = true;   
    // Truyền socket cho task con 
    xTaskCreate(gui_send_task, "Send To GUI", 8192, (void *)sock_ptr, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Khởi tạo NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Xoá NVS và khởi tạo lại nếu gặp lỗi
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Bật bluetooth 
    wifi_bt_init();

    if (sd_spi_mount(&sd_cfg) != ESP_OK){
        return;
    }

    imupt_queue = xQueueCreate(50, sizeof(imupt_data_t));
    audio_queue = xQueueCreate(20, sizeof(audio_data_t));

    if (!imupt_queue || !audio_queue) {
    ESP_LOGE(TAG, "Failed to create queues!");
    return;
    }

    gpio_init();
    xTaskCreate(button_monitor_task, "Button", 2048, NULL, 4, NULL);
    xTaskCreate(led_blink_task, "LED", 1024, NULL, 4, NULL);

    xTaskCreate(read_imupt_task, "ReadIMUPT", 6000, NULL, 5, NULL);
    xTaskCreate(read_audio_task, "ReadAudio", 6000, NULL, 6, NULL);

    xTaskCreate(write_data_to_sd_task, "WriteIMUPT", 8192, NULL, 8, NULL);

    xTaskCreate(bt_wifi_config_task, "bt_wifi_task", 8192, NULL, 4, NULL);
}
