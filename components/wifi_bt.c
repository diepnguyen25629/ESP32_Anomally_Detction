#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h> 

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"       
#include "esp_spp_api.h"

#include <sys/select.h>
#include <sys/time.h>

#include "cJSON.h"
#include "wifi_bt.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_RETRY 5

static const char *TAG = "WIFI_BT";

static EventGroupHandle_t s_wifi_event_group;
static SemaphoreHandle_t bt_config_sem = NULL;
static uint32_t spp_client_handle = 0;
static int retry_num = 0;

static const char *local_device_name = "ESP32_SPP_SERVER"; // Tên thiết bị ESP32

static wifi_gui_config_t wifi_gui_config;
static esp_netif_t *wifi_netif = NULL;

// =========================Bluetooth============================
static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) 
{
    switch (event) {
        case ESP_SPP_INIT_EVT:
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, local_device_name);
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            spp_client_handle = param->srv_open.handle;
            break;
        case ESP_SPP_DATA_IND_EVT: {
            char *buf = calloc(param->data_ind.len + 1, 1);
            if (!buf) return;

            memcpy(buf, param->data_ind.data, param->data_ind.len);

            cJSON *root = cJSON_Parse(buf);
            if (root) {
                const cJSON *ssid = cJSON_GetObjectItem(root, "ssid");
                const cJSON *password = cJSON_GetObjectItem(root, "password");
                const cJSON *ip = cJSON_GetObjectItem(root, "ip");

                if (cJSON_IsString(ssid) && cJSON_IsString(password) && cJSON_IsString(ip)) {
                    strncpy(wifi_gui_config.ssid, ssid->valuestring, sizeof(wifi_gui_config.ssid));
                    wifi_gui_config.ssid[sizeof(wifi_gui_config.ssid) - 1] = '\0';

                    strncpy(wifi_gui_config.password, password->valuestring, sizeof(wifi_gui_config.password));
                    wifi_gui_config.password[sizeof(wifi_gui_config.password) - 1] = '\0';

                    strncpy(wifi_gui_config.ip, ip->valuestring, sizeof(wifi_gui_config.ip));
                    wifi_gui_config.ip[sizeof(wifi_gui_config.ip) - 1] = '\0';
                    
                    if (bt_config_sem) xSemaphoreGive(bt_config_sem);
                }
                cJSON_Delete(root);
            }
            // Gửi phản hồi thành công cho GUI
            const char *response = "OK: WiFi config received\n";
            esp_spp_write(spp_client_handle, strlen(response), (uint8_t *)response);

            free(buf);
            break;
        }
        default:
            break;
    }
}

static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    //ESP_LOGI(TAG, "GAP event: %d", event);
}

// Bật bluetooth và khởi tạo semaphore để thông báo khi nhận được cấu hình wifi từ GUI 
void bluetooth_init(void)
{
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);

    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_bt_gap_register_callback(bt_gap_cb);
    esp_spp_register_callback(spp_callback);

    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0
    };
    esp_spp_enhanced_init(&spp_cfg);

    esp_bt_dev_set_device_name(local_device_name);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    if (!bt_config_sem) bt_config_sem = xSemaphoreCreateBinary();

    ESP_LOGI(TAG, "Bluetooth SPP ready");
}

 // Hủy kết nối Bluetooth  
void bluetooth_disconnect(void) 
{
    esp_spp_deinit();
    vTaskDelay(pdMS_TO_TICKS(100));   // Cho SPP thoát hẳn
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Bluetooth disabled");
}
//=======================================WiFi=======================================================
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_num < MAX_RETRY) {
            esp_wifi_connect();
            retry_num++;
            ESP_LOGI(TAG, "Retry to connect...");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connect to AP failed.");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_connect(const char *ssid, const char *password) 
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    wifi_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();  

    ESP_LOGI(TAG, "WiFi connecting to %s", ssid);
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}

void wifi_disconnect(void) 
{
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit(); 
    if (wifi_netif) {
        esp_netif_destroy(wifi_netif); 
    }
}

//====================================TCP Socket==========================================
int tcp_socket_connect(const char *ip, uint16_t port) 
{
    struct sockaddr_in dest_addr = {
        .sin_addr.s_addr = inet_addr(ip),
        .sin_family = AF_INET,
        .sin_port = htons(port),
    };

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE("TCP_CONNECT", "Unable to create socket: errno %d", errno);
        return -1;
    }

    // Thiết lập timeout 5 giây cho connect
    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGE("TCP_CONNECT", "Socket unable to connect: errno %d", errno);
        close(sock);
        return -1;
    }

    ESP_LOGI("TCP_CONNECT", "Successfully connected to %s:%d", ip, port);
    return sock;
}

//========================================== WiFi & Sensor Config ==========================================
// Đợi để lấy cấu hình wifi từ semaphore bluetooth tạo ra 
void wait_wifi_config(wifi_gui_config_t *config) 
{
    ESP_LOGI(TAG, "Waiting for WiFi config via Bluetooth...");
    xSemaphoreTake(bt_config_sem, portMAX_DELAY);
    memcpy(config, &wifi_gui_config, sizeof(wifi_gui_config_t));
}

esp_err_t wait_sensor_config(sensor_config_t *config, int sock) 
{
    char rx_buffer[512];
    int total_len = 0;
    char ch;
    uint8_t response[2] = {0xA1, 0x00};

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(sock, &read_fds);

    struct timeval timeout = {.tv_sec = 2, .tv_usec = 0};
    int ret = select(sock + 1, &read_fds, NULL, NULL, &timeout);
    if (ret <= 0) return ESP_FAIL;

    while (total_len < sizeof(rx_buffer) - 1) {
        int len = recv(sock, &ch, 1, 0);
        if (len <= 0) {
            response[1] = 0x01;
            send(sock, response, sizeof(response), 0);
            return ESP_FAIL;
        }
        if (ch == '\n') break;
        rx_buffer[total_len++] = ch;
    }

    rx_buffer[total_len] = '\0';
    ESP_LOGI(TAG, "Received sensor config: %s", rx_buffer);

    cJSON *root = cJSON_Parse(rx_buffer);
    if (!root) {
        ESP_LOGE(TAG, "JSON parsing failed");
        response[1] = 0x01;
        send(sock, response, sizeof(response), 0);
        return ESP_FAIL;
    }

    cJSON *accel_fs = cJSON_GetObjectItem(root, "accel_fs");
    cJSON *gyro_fs = cJSON_GetObjectItem(root, "gyro_fs");
    cJSON *imu_rate = cJSON_GetObjectItem(root, "imu_sample_rate");
    cJSON *audio_rate = cJSON_GetObjectItem(root, "audio_sample_rate");
    cJSON *cal_matrix = cJSON_GetObjectItem(root, "accel_cal_matrix");
    cJSON *accel_bias = cJSON_GetObjectItem(root, "accel_bias");
    cJSON *gyro_bias = cJSON_GetObjectItem(root, "gyro_bias");

    if (!cJSON_IsNumber(imu_rate) || !cJSON_IsNumber(accel_fs) || !cJSON_IsNumber(gyro_fs) ||
        !cJSON_IsArray(accel_bias) || cJSON_GetArraySize(accel_bias) != 3 ||
        !cJSON_IsArray(gyro_bias) || cJSON_GetArraySize(gyro_bias) != 3 ||
        !cJSON_IsArray(cal_matrix) || cJSON_GetArraySize(cal_matrix) != 9) 
    {
        ESP_LOGE(TAG, "Invalid sensor config fields");
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    // Gán đơn giản
    config->imu_sample_rate = (int)cJSON_GetNumberValue(imu_rate);
    config->audio_sample_rate = (int)cJSON_GetNumberValue(audio_rate);
    config->accel_fs = (accel_fs_t)cJSON_GetNumberValue(accel_fs);
    config->gyro_fs = (gyro_fs_t)cJSON_GetNumberValue(gyro_fs);

    // Bias vector
    config->calibration.accel_bias_offset.x = (float)cJSON_GetArrayItem(accel_bias, 0)->valuedouble;
    config->calibration.accel_bias_offset.y = (float)cJSON_GetArrayItem(accel_bias, 1)->valuedouble;
    config->calibration.accel_bias_offset.z = (float)cJSON_GetArrayItem(accel_bias, 2)->valuedouble;

    config->calibration.gyro_bias_offset.x = (float)cJSON_GetArrayItem(gyro_bias, 0)->valuedouble;
    config->calibration.gyro_bias_offset.y = (float)cJSON_GetArrayItem(gyro_bias, 1)->valuedouble;
    config->calibration.gyro_bias_offset.z = (float)cJSON_GetArrayItem(gyro_bias, 2)->valuedouble;

    // Ma trận hiệu chuẩn 3x3
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            config->calibration.accel_cal_matric[i][j] = 
                (float)cJSON_GetArrayItem(cal_matrix, i * 3 + j)->valuedouble;
        }
    }

    ESP_LOGI(TAG, "Parsed sensor config OK");
    send(sock, response, sizeof(response), 0);
    cJSON_Delete(root);
    return ESP_OK;
}




