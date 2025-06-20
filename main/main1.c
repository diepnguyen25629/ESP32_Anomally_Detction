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
#include "esp_wifi.h"

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

// Số lượng lô dữ liệu cảm biến ghi vào thẻ SD và gửi đến GUI 
#define IMU_WRITE_BATCH 35
#define AUDIO_BUF_LEN 512
#define AUDIO_WRITE_BATCH 10 // Ghi 16 buffer 512 mẫu mỗi lần 

#define PACKET_MAGIC                0xABCD      // Vị trí bắt đầu của packet
#define PACKET_TYPE_IMU             0x01
#define PACKET_TYPE_AUDIO           0x02
#define PACKET_TYPE_PRE_TEMP        0x03

#define IMU_CONFIG_UPDATED_EVENT                BIT0
#define PRE_TEMP_CONFIG_UPDATED_EVENT           BIT1
#define AUDIO_CONFIG_UPDATE_EVENT               BIT2

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
    .gyro_fs = GYRO_FS_2000DPS

};

// Cấu hình các tham số và tần số lấy mẫu cho cảm biến bmp280
bmp280_config_t bmp280_cfg = {
        .mode = BMP280_MODE_NORMAL,
        .filter = BMP280_FILTER_8,
        .oversampling_pressure = BMP280_ULTRA_HIGH_RES,     
        .oversampling_temperature = BMP280_LOW_POWER,  
        .stanby = BMP280_STANDBY_125, 
        .pre_sample_rate = 1
};

// Cấu hình độ phân giải và tần số lấy mẫu của cảm biến ds18b20
ds18b20_config_t ds18b20_cfg = {
        .temp_resol = RESOLUTION_9_BIT,
        .temp_sample_rate = 1
};

// Cấu hình các chân giao tiếp I2S của cảm biến inmp441
inmp441_config_t inmp441_cfg = {
    .audio_sample_rate = 16000,
    .bclk_io_num = 26,
    .ws_io_num = 27,
    .data_in_io_num = 25,
    .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT
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
}imu_data_t;

typedef struct {
    float temp_data, pre_data;
}pre_temp_data_t;

typedef struct {
    int16_t audio_buf[AUDIO_BUF_LEN];
}audio_data_t;

// Bộ đệm và biến đếm để ghi dữ liệu imupt theo batch vào sd card
imu_data_t imu_batch[IMU_WRITE_BATCH];
int imu_batch_index = 0;

// Bộ đệm và biến đếm để ghi dữ liệu audio theo batch vào sd card
audio_data_t audio_batch[AUDIO_WRITE_BATCH];
int audio_batch_index = 0; 

// Bộ đệm và biến đếm để gửi dữ liệu imupt theo batch đến GUI
imu_data_t imu_batch_gui[IMU_WRITE_BATCH];
int imu_batch_gui_index = 0;

// Bộ đệm và biến đếm để gửi dữ liệu audio theo batch đến GUI
audio_data_t audio_batch_gui[AUDIO_WRITE_BATCH];
int audio_batch_gui_index = 0;

// Phần header dữ liệu nhị phân ghi vào sd và gửi đến GUI 
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint16_t type;
    uint16_t length;
} packet_header_t;

esp_timer_handle_t imu_timer;
esp_timer_handle_t pre_temp_timer;

TaskHandle_t imu_task_handle;
TaskHandle_t pre_temp_task_handle;
TaskHandle_t audio_task_handle;

QueueHandle_t imu_queue;                              // Hàng đợi chứa dữ liệu cảm biến để ghi dữ liệu vào thẻ sd 
QueueHandle_t audio_queue;
QueueHandle_t pre_temp_queue;
QueueHandle_t imu_queue_gui;                           // Hàng đợi chứa dữ liệu cảm biến để gửi đến GUI
QueueHandle_t audio_queue_gui;
QueueHandle_t pre_temp_queue_gui;

volatile bool is_recording = false;                     // Trạng thái record vào sd card 
volatile bool is_connect_gui = false;                   // Trạng thái kêt nối đến GUI
volatile bool is_bluetooth = false;                     // Trạng thái bất/tắt bluetooth 
volatile bool is_sensor_config = false;
wifi_gui_config_t wifi_config;                          // Cấu hình wifi gửi từ GUI 

int gui_socket = -1;

void imu_timer_callback(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(imu_task_handle, 0, eNoAction, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void pre_temp_timer_callback(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(pre_temp_task_handle, 0, eNoAction, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Khởi tạo các cảm biến accelerometer, gyroscope, pressure, temperate rồi đưa dữ liệu vào hàng đợi
void read_imu_task(void *pvParameter)
{   
    imu_data_t imu_data;

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

    while (1)
    {
        if (is_sensor_config) i2c_mpu6500_init(&mpu6500_cfg);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        get_accel_gyro(&imu_data.accel_data, &imu_data.gyro_data);
        if (is_recording) {
            xQueueSend(imu_queue, &imu_data, 0);
        }
        if (is_connect_gui && imu_queue_gui != NULL) {
            xQueueSend(imu_queue_gui, &imu_data, 0);
        }
    }
}

void read_pre_temp_task(void *pvParameter) 
{
    pre_temp_data_t pre_temp_data;

    // Khởi tạo DS18B20
    esp_err_t ret = ds18b20_init(ONE_WIRE_GPIO);
    if(ret != ESP_OK){
        ESP_LOGE(TAG, " Failed to initialize 1-Wire");
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

    // Khởi tạo BMP280
    ret = bmp280_init(&bmp280_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMP280");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);        
        bmp280_read_pre(&pre_temp_data.pre_data);
        ds18b20_start_conversion();
        // Đợi để ds18b20 chuyển đổi xong 
        vTaskDelay(pdMS_TO_TICKS(millisToWaitForConversion(ds18b20_cfg.temp_resol)));
        if (ds18b20_is_conversion_done()) {
            ds18b20_read_temperate(&sensorAddr, &pre_temp_data.temp_data);
            ESP_LOGI(TAG, "%f",pre_temp_data.temp_data);
        }

        if (is_recording) {
            xQueueSend(pre_temp_queue, &pre_temp_data, 0);
        }
        if (is_connect_gui && pre_temp_queue_gui != NULL) {
            xQueueSend(pre_temp_queue_gui, &pre_temp_data, 0);
        }
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
        if (is_sensor_config) {
            inmp441_i2s_deinit();
            inmp441_i2s_init(&inmp441_cfg);
        } 
    
        int bytes = inmp441_i2s_read(audio_data.audio_buf, sizeof(audio_data.audio_buf));
        if (bytes > 0 ) { 
            if (is_recording) {
                xQueueSend(audio_queue, &audio_data, 0);
            }
            if (is_connect_gui && audio_queue_gui != NULL) {
                xQueueSend(audio_queue_gui, &audio_data, 0); 
            }
        }
    }
}

// Lấy dữ liệu từ hàng đợi rồi đưa vào buffer để truyền theo batch 
//    Sau khi buffer đầy thì ghi xuóng file của sd card
void write_data_to_sd_task(void *pvParameter) {
    FILE *data_file = NULL;
    static char filepath[64];
    imu_data_t imu_data_recv;
    audio_data_t audio_data_recv;
    pre_temp_data_t pre_temp_data_recv;
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
            if (imu_batch_index > 0) {
                    packet_header_t header = {
                    .magic = PACKET_MAGIC,
                    .type = PACKET_TYPE_IMU,
                    .length = imu_batch_index * sizeof(imu_data_t),
                };
                sd_spi_write_binary_buffer(data_file, &header, sizeof(header), 1);
                sd_spi_write_binary_buffer(data_file, imu_batch, sizeof(imu_data_t), imu_batch_index);
                imu_batch_index = 0;                
            }
               if (audio_batch_index > 0) {
                packet_header_t header = {
                    .magic = PACKET_MAGIC,
                    .type = PACKET_TYPE_AUDIO,
                    .length = audio_batch_index * sizeof(audio_data_t),
                };
                sd_spi_write_binary_buffer(data_file, &header, sizeof(header), 1);
                sd_spi_write_binary_buffer(data_file, audio_batch, sizeof(audio_data_t), audio_batch_index);
                audio_batch_index = 0;
                }
            
            sd_spi_stop_write(data_file);
            data_file = NULL;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // fflush file theo chu kì 
        if (data_file != NULL && now - last_reopen_us > REOPEN_INTERVAL_US) {
            fflush(data_file);
            last_reopen_us = now;
            continue;
        }

        if (is_recording && data_file != NULL) {
            while (xQueueReceive(imu_queue, &imu_data_recv, 0) == pdTRUE) {
                imu_batch[imu_batch_index++] = imu_data_recv;

                if (imu_batch_index >= IMU_WRITE_BATCH) {
                    packet_header_t header = {
                    .magic = PACKET_MAGIC,
                    .type = PACKET_TYPE_IMU,
                    .length = imu_batch_index * sizeof(imu_data_t),
                };
                sd_spi_write_binary_buffer(data_file, &header, sizeof(header), 1);
                sd_spi_write_binary_buffer(data_file, imu_batch, sizeof(imu_data_t), imu_batch_index);
                imu_batch_index = 0;
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
            
            if (xQueueReceive(pre_temp_queue, &pre_temp_data_recv, 0) == pdTRUE) {
                packet_header_t header = {
                    .magic = PACKET_MAGIC,
                    .type = PACKET_TYPE_PRE_TEMP,
                    .length = sizeof(pre_temp_data_t),
                };
                sd_spi_write_binary_buffer(data_file, &header, sizeof(header), 1);
                sd_spi_write_binary_buffer(data_file, &pre_temp_data_recv, sizeof(pre_temp_data_t), 1);
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

void gui_send_task(void *pvParameter) 
{
    int sock = *((int *)pvParameter);

    imu_data_t imu_data_gui_recv;
    audio_data_t audio_data_gui_recv;
    pre_temp_data_t pre_temp_data_recv;
 
    while(1) {
        if (!is_connect_gui) break;
        
        while (xQueueReceive(imu_queue_gui, &imu_data_gui_recv, 0) == pdTRUE) {
            imu_batch_gui[imu_batch_gui_index++] = imu_data_gui_recv;
            if (imu_batch_gui_index >= IMU_WRITE_BATCH) {
                packet_header_t header = {
                .magic = PACKET_MAGIC,
                .type = PACKET_TYPE_IMU,
                .length = imu_batch_gui_index * sizeof(imu_data_t),
            };
            send(sock, &header, sizeof(header), 0);
            send(sock, imu_batch_gui, imu_batch_gui_index * sizeof(imu_data_t), 0);
            imu_batch_gui_index = 0;
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

            send(sock, &header, sizeof(header), 0);
            send(sock, audio_batch_gui, audio_batch_gui_index * sizeof(audio_data_t), 0);
            audio_batch_gui_index = 0;
            }          
        }

        if (xQueueReceive(pre_temp_queue_gui, &pre_temp_data_recv, 0) == pdTRUE) {
            packet_header_t header = {
                .magic = PACKET_MAGIC,
                .type = PACKET_TYPE_PRE_TEMP,
                .length = sizeof(pre_temp_data_t),
            };
            
            send(sock, &header, sizeof(header), 0);
            send(sock, &pre_temp_data_recv, sizeof(pre_temp_data_t), 0);
        }
        vTaskDelay(pdMS_TO_TICKS(5));     
    }
    close(sock);
    vTaskDelete(NULL);
}

void receive_sensor_config_task(void *pvParameter)
{
    int sock = *((int *)pvParameter);
    free(pvParameter);

    while (1)
    {
        if (!is_connect_gui) break;
    
        sensor_config_t sensor_cfg;
        if (wait_sensor_config(&sensor_cfg, sock) == ESP_OK) {

        mpu6500_cfg.accel_fs = sensor_cfg.accel_fs;
        mpu6500_cfg.gyro_fs = sensor_cfg.gyro_fs;
        mpu6500_cfg.calibration = sensor_cfg.calibration;
        mpu6500_cfg.imu_sample_rate = sensor_cfg.imu_sample_rate;
        inmp441_cfg.audio_sample_rate = sensor_cfg.audio_sample_rate;

        is_sensor_config = true;
        } else {
            is_sensor_config = false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}

void bt_wifi_gui_config_task(void *pvParameter)
{
    if (is_bluetooth) {
        // Đợi nhận cấu hình wifi từ semaphore của bluetooth 
        wait_wifi_config(&wifi_config);

        bluetooth_disconnect(); 
        is_bluetooth = false;
    }

    wifi_connect(wifi_config.ssid, wifi_config.password);

    // Kết nối TCP đến GUI 
    int sock = tcp_socket_connect(wifi_config.ip, 12345);
    if (sock < 0) {
        vTaskDelete(NULL);
        return;
    } 

    gui_socket = sock;
    int *sock_ptr = malloc(sizeof(int));
    if (sock_ptr == NULL) {
        ESP_LOGE(TAG, "Malloc failed for socket ptr");
        close(gui_socket);
        gui_socket = -1;
        vTaskDelete(NULL);
        return;
    }
    *sock_ptr = gui_socket;

    imu_queue_gui = xQueueCreate(35, sizeof(imu_data_t));
    audio_queue_gui = xQueueCreate(15, sizeof(audio_data_t));
    pre_temp_queue_gui = xQueueCreate(10, sizeof(pre_temp_data_t));

    if(!imu_queue_gui || !audio_queue_gui || !pre_temp_queue_gui){
        ESP_LOGE(TAG, "Fail to create queue GUI");
        return;
    }

    is_connect_gui = true;   
    // Truyền socket cho task con 
    xTaskCreatePinnedToCore(gui_send_task, "Send To GUI", 8192, (void *)sock_ptr, 5, NULL, 0);
    xTaskCreatePinnedToCore(receive_sensor_config_task, "Receive Sensor Config From GUI", 6000, (void *)sock_ptr, 4, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    vTaskDelete(NULL);
}

void stop_wifi_gui_connection() 
{       
    is_connect_gui = false;
    if (gui_socket >= 0) {
        close(gui_socket);
        gui_socket = -1;
    }

    if (imu_queue_gui) {
        vQueueDelete(imu_queue_gui);
        imu_queue_gui = NULL;
    }

    if (pre_temp_queue_gui) {
        vQueueDelete(pre_temp_queue_gui);
        pre_temp_queue_gui = NULL;
    }
    if (audio_queue_gui) {
        vQueueDelete(audio_queue_gui);
        audio_queue_gui = NULL;
    }
    wifi_disconnect();

} 
// Kiểm tra trạng thái của button 
// Nếu nhấn dưới 3s thì bật/tắt ghi dữ liệu và thẻ sd
// Nếu nhấn trên 3s thì bật/tắt wifi kết nối với GUI 
void button_monitor_task(void *pvParameter)
{
    bool last_button_state = true;
    int64_t press_start_time = 0;
    const int64_t long_press_duration = 3000000;  // 3s = 3000000 us
    volatile bool is_bt_wifi_started = false;

    while (1) {
        bool button_state = gpio_get_level(BUTTON_GPIO);

        if (!button_state && last_button_state) {
            press_start_time = esp_timer_get_time();
        }

        if (button_state && !last_button_state) {
            int64_t press_duration = (esp_timer_get_time() - press_start_time);

            if (press_duration >= long_press_duration) {
                is_bt_wifi_started = !is_bt_wifi_started;
                if (is_bt_wifi_started) {
                    ESP_LOGI(TAG, "Starting Wifi/GUI connection...");
                    xTaskCreatePinnedToCore(bt_wifi_gui_config_task, "BT_WIFI", 8192, NULL, 4, NULL, 0);
                } else {
                    ESP_LOGI(TAG, "Stopping Wifi/GUI connection...");
                    stop_wifi_gui_connection();
                }
            } else {
                is_recording = !is_recording;
                ESP_LOGI(TAG, "Recording: %s", is_recording ? "ON" : "OFF");                
            }
            vTaskDelay(pdMS_TO_TICKS(200));  // chống dội nút
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

void app_main(void)
{
    // Khởi tạo NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Xoá NVS và khởi tạo lại nếu gặp lỗi
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    bluetooth_init();
    is_bluetooth = true;
    if (sd_spi_mount(&sd_cfg) != ESP_OK){
        return;
    }
    imu_queue = xQueueCreate(35, sizeof(imu_data_t));
    audio_queue = xQueueCreate(15, sizeof(audio_data_t));
    pre_temp_queue = xQueueCreate(10, sizeof(pre_temp_data_t));

    if (!imu_queue || !audio_queue || !pre_temp_queue) {
    ESP_LOGE(TAG, "Failed to create queues!");
    return;
    }

    gpio_init();
    xTaskCreate(button_monitor_task, "Button", 2048, NULL, 4, NULL);
    xTaskCreate(led_blink_task, "LED", 1024, NULL, 4, NULL);

    xTaskCreatePinnedToCore(read_imu_task, "ReadIMU", 3000, NULL, 7, &imu_task_handle, 1);
    xTaskCreatePinnedToCore(read_audio_task, "ReadAudio", 6000, NULL, 8, &audio_task_handle, 1);
    xTaskCreatePinnedToCore(read_pre_temp_task, "ReadPreTemp", 2048, NULL, 9, &pre_temp_task_handle, 1);

    xTaskCreate(write_data_to_sd_task, "WriteDataToSDCard", 6000, NULL, 5, NULL);

    // Tạo timer IMU
    const esp_timer_create_args_t imu_timer_args = {
        .callback = &imu_timer_callback,
        .name = "imu_timer"
    };
    esp_timer_create(&imu_timer_args, &imu_timer);
    esp_timer_start_periodic(imu_timer, 1000000/(mpu6500_cfg.imu_sample_rate));

    // Tạo timer Pre/Temp
    const esp_timer_create_args_t pre_temp_timer_args = {
        .callback = &pre_temp_timer_callback,
        .name = "pre_temp_timer"
    };
    esp_timer_create(&pre_temp_timer_args, &pre_temp_timer);
    esp_timer_start_periodic(pre_temp_timer, 1000000/(bmp280_cfg.pre_sample_rate));
}
