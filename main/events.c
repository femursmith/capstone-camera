#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h" // Added for Semaphores
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/gpio.h" // Added for GPIO control
#include "esp_task_wdt.h"
#include "esp_camera.h"
#include "esp_http_client.h"
// #include "esp_http_server.h" // Removed - No longer streaming
#include "esp_timer.h"
#include "esp_tls.h"
#include "events.h"
#include "wifimanager.h"

// Define MIN macro
#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

// --- Configuration ---
// WiFi
#define CONFIG_ESP_WIFI_SSID      "DUFIE-HOSTEL"
#define CONFIG_ESP_WIFI_PASSWORD  "Duf1e@9723"
#define CONFIG_ESP_MAXIMUM_RETRY  5

// Upload Server
#define CONFIG_UPLOAD_SERVER_IP   "192.168.173.152"
#define CONFIG_UPLOAD_SERVER_PORT 3001
#define CONFIG_UPLOAD_PATH        "/upload"
#define CONFIG_USER_ID            stored_user_id
#define CONFIG_CAMERA_ID          stored_camera_id // Replace with ESP32 Cam ID
#define CONFIG_EVENT_DESC         "Motion detected"
// #define UPLOAD_INTERVAL_MS        10000 // Removed - No longer periodic upload

// PIR Sensor
 // Using GPIO43 for PIR input
#define MOTION_DETECT_COOLDOWN_MS 1000 // 5 seconds cooldown after detection/upload attempt



// --- Global Variables ---
static const char *TAG = "event_handeler";
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;
bool event_recieved = false;
extern QueueHandle_t eventQueue; // Queue for event handling
extern SemaphoreHandle_t xSemaphore; // Semaphore for synchronization

// Semaphore to signal motion detection from ISR
static SemaphoreHandle_t pirSemaphore = NULL;

// --- WiFi Event Handler (Unchanged) ---
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// --- WiFi Initialization (Unchanged) ---
void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // Adjust if needed
             .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", CONFIG_ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGW(TAG, "Failed to connect to SSID:%s", CONFIG_ESP_WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED WIFI EVENT");
    }
}


// --- HTTP Client Event Handler (Unchanged, useful for upload debug) ---
esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    static char *output_buffer;  // Buffer to store response of http request
    static int output_len;       // Stores number of bytes read
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
             if (!esp_http_client_is_chunked_response(evt->client)) {
                if (output_buffer == NULL) {
                    int content_len = esp_http_client_get_content_length(evt->client);
                    if (content_len > 0) {
                         output_buffer = (char *) malloc(content_len + 1); // +1 for null terminator
                         if (output_buffer == NULL) {
                             ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                             return ESP_FAIL;
                         }
                         output_len = 0; // Initialize length tracking
                         ESP_LOGD(TAG, "Allocated %d bytes for response buffer", content_len + 1);
                    } else {
                         ESP_LOGW(TAG, "Content-Length not available or zero");
                         return ESP_OK; // Or handle differently
                    }
                }
                int copy_len = 0;
                 int remaining_buf_size = esp_http_client_get_content_length(evt->client) - output_len;
                 if (remaining_buf_size < 0) remaining_buf_size = 0; // Safety

                copy_len = MIN(evt->data_len, remaining_buf_size);
                if (copy_len > 0) {
                    memcpy(output_buffer + output_len, evt->data, copy_len);
                    output_len += copy_len;
                    output_buffer[output_len] = '\0'; // Null-terminate
                } else if (evt->data_len > 0) {
                     ESP_LOGW(TAG,"Buffer overflow detected in HTTP handler");
                }

            } else {
                 ESP_LOGD(TAG, "Handling chunked response data (not fully implemented in this example)");
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                ESP_LOGI(TAG, "Upload Response: %.*s", output_len, output_buffer);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            if (output_buffer != NULL) {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
         case HTTP_EVENT_REDIRECT: // Added for completeness
             ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
             break;
    }
    return ESP_OK;
}


// --- Upload Function (Modified from task) ---
// Takes a captured frame buffer and uploads it
esp_err_t upload_image(camera_fb_t *fb) {
    if (!fb || !fb->buf || fb->len == 0) {
        ESP_LOGE(TAG, "Invalid frame buffer for upload");
        return ESP_ERR_INVALID_ARG;
    }

    // Check WiFi connection bit one last time before attempting upload
    //EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    // if (!(bits & WIFI_CONNECTED_BIT)) {
    //     ESP_LOGE(TAG, "WiFi not connected, skipping upload.");
    //     return ESP_FAIL;
    // }


    ESP_LOGI(TAG, "Uploading image: %zu bytes", fb->len);
    esp_err_t res = ESP_OK;

    // Construct URL
    char upload_url[128];
    snprintf(upload_url, sizeof(upload_url), "http://%s:%d%s",
             CONFIG_UPLOAD_SERVER_IP, CONFIG_UPLOAD_SERVER_PORT, CONFIG_UPLOAD_PATH);

    // Construct boundary string (needs to be unique for each request)
    char boundary[64];
    snprintf(boundary, sizeof(boundary), "----ESP32CamBoundary%lld", esp_timer_get_time());

    // Reusable buffer for multipart headers/footers
    // Static to avoid repeated malloc/free if function is called often,
    // but be mindful if threading changes. For single upload trigger, malloc/free is ok too.
    #define POST_BUFFER_SIZE 1024
    char *post_data_buffer = (char *)malloc(POST_BUFFER_SIZE);
    if (!post_data_buffer) {
        ESP_LOGE(TAG, "Failed to allocate buffer for upload");
        return ESP_ERR_NO_MEM;
    }

    // Construct multipart form data prefix
    int head_len = snprintf(post_data_buffer, POST_BUFFER_SIZE,
                            "--%s\r\n"
                            "Content-Disposition: form-data; name=\"userId\"\r\n\r\n"
                            "%s\r\n"
                            "--%s\r\n"
                            "Content-Disposition: form-data; name=\"cameraId\"\r\n\r\n"
                            "%s\r\n"
                            "--%s\r\n"
                            "Content-Disposition: form-data; name=\"event\"\r\n\r\n"
                            "%s\r\n"
                            "--%s\r\n"
                            "Content-Disposition: form-data; name=\"file\"; filename=\"image.jpg\"\r\n"
                            "Content-Type: image/jpeg\r\n\r\n",
                            boundary, CONFIG_USER_ID,
                            boundary, CONFIG_CAMERA_ID,
                            boundary, CONFIG_EVENT_DESC,
                            boundary);
    if (head_len >= POST_BUFFER_SIZE) {
         ESP_LOGE(TAG, "Post data buffer too small for headers!");
         free(post_data_buffer);
         return ESP_ERR_INVALID_SIZE;
    }

    // Construct multipart form data suffix
    char footer[128];
    int foot_len = snprintf(footer, sizeof(footer), "\r\n--%s--\r\n", boundary);

    // Calculate total content length
    int total_len = head_len + fb->len + foot_len;

    esp_http_client_config_t config = {
        .url = upload_url,
        .method = HTTP_METHOD_POST,
        .event_handler = _http_event_handler,
        .timeout_ms = 20000, // Increased timeout for potentially slower uploads
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (!client) {
        ESP_LOGE(TAG, "Failed to initialise HTTP client");
        free(post_data_buffer);
        return ESP_FAIL; // Indicate failure
    }

    // Set headers
    char content_type_header[128];
    snprintf(content_type_header, sizeof(content_type_header), "multipart/form-data; boundary=%s", boundary);
    esp_http_client_set_header(client, "Content-Type", content_type_header);

    // Manually write the multipart body
    res = esp_http_client_open(client, total_len);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(res));
    } else {
        // Write header part
        int written = esp_http_client_write(client, post_data_buffer, head_len);
        vTaskDelay(pdMS_TO_TICKS(10));
        if (written < 0 || written != head_len) {
             ESP_LOGE(TAG, "Failed to write header part (written %d, expected %d)", written, head_len);
             res = ESP_FAIL;
        } else {
             ESP_LOGD(TAG,"Wrote header: %d bytes", written);
        }

        // Write image data
        if (res == ESP_OK) {
            written = esp_http_client_write(client, (const char *)fb->buf, fb->len);
            vTaskDelay(pdMS_TO_TICKS(10));
            if (written < 0 || written != (int)fb->len) { // Cast fb->len safely
                ESP_LOGE(TAG, "Failed to write image data (written %d, expected %zu)", written, fb->len);
                res = ESP_FAIL;
            } else {
                 ESP_LOGD(TAG,"Wrote image: %d bytes", written);
            }
        }

        // Write footer part
        if (res == ESP_OK) {
            written = esp_http_client_write(client, footer, foot_len);
            vTaskDelay(pdMS_TO_TICKS(10));
            if (written < 0 || written != foot_len) {
                ESP_LOGE(TAG, "Failed to write footer part (written %d, expected %d)", written, foot_len);
                res = ESP_FAIL;
            } else {
                 ESP_LOGD(TAG,"Wrote footer: %d bytes", written);
            }
        }

        // Check response if all writes were successful
        if (res == ESP_OK) {
            ESP_LOGI(TAG, "HTTP POST multipart data written: %d bytes", total_len);
            int status_code = esp_http_client_fetch_headers(client);
             if (status_code < 0) {
                 ESP_LOGE(TAG, "HTTP client fetch headers failed: %s", esp_err_to_name(status_code));
                 res = ESP_FAIL; // Treat as failure
             } else {
                 status_code = esp_http_client_get_status_code(client);
                 ESP_LOGI(TAG, "HTTP POST Status = %d", status_code);
                 // Read response body via event handler
                 // Check status code
                 if (status_code == 200 || status_code == 201 || status_code == 204) {
                     ESP_LOGI(TAG,"Image uploaded successfully!");
                     res = ESP_OK; // Explicitly mark as success
                 } else {
                     ESP_LOGE(TAG,"Image upload failed with server status: %d", status_code);
                     res = ESP_FAIL; // Mark as failure
                 }
             }
        } else {
             ESP_LOGE(TAG, "HTTP POST request failed during write phase");
        }
    }

    // Cleanup
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(post_data_buffer);

    return res; // Return ESP_OK on success, ESP_FAIL or other error code on failure
}


// --- PIR Sensor ISR ---
static void IRAM_ATTR pir_isr_handler(void* arg) {
    // Give the semaphore - signal that motion was detected
    // Need to use the ISR version of the function
    xSemaphoreGiveFromISR(pirSemaphore, NULL);
}

// --- PIR Sensor Configuration ---
static void configure_pir_sensor(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIR_SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE // Disable interrupt
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "PIR Sensor configured for polling on GPIO %d", PIR_SENSOR_PIN);
}


void upload_image_init(){

    pirSemaphore = xSemaphoreCreateBinary();
    if (pirSemaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create PIR semaphore");
        // Handle error - perhaps restart?
        return;
    }

   
 
  
    // Configure PIR Sensor and ISR
    configure_pir_sensor();
}



void upload_image_task(void* pvParameters) {
    ESP_LOGI(TAG, "Initialization complete. Waiting for motion...");
    while (1) {
        if (gpio_get_level(PIR_SENSOR_PIN) == 1) {
            ESP_LOGI(TAG, "Motion detected! Capturing and uploading image...");
            event_recieved = true;

            camera_fb_t *fb = NULL;
            if (eventQueue == NULL) {
                ESP_LOGE(TAG, "eventQueue is NULL, skipping upload");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            if (xQueueReceive(eventQueue, &fb, portMAX_DELAY) == pdTRUE) {
                event_recieved = false;
                //if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
                    ESP_LOGI(TAG, "Streaming frame %d...", fb->len);
                    esp_err_t upload_status = upload_image(fb);
                    esp_task_wdt_reset(); // Reset watchdog after upload
                    if (upload_status != ESP_OK) {
                        ESP_LOGE(TAG, "Image upload failed.");
                    }
                    //xSemaphoreGive(xSemaphore);
               // }
                free(fb->buf);
                free(fb);
            }

            event_recieved = false;

            ESP_LOGI(TAG, "Cooldown period (%dms)...", MOTION_DETECT_COOLDOWN_MS);
            for (int i = 0; i < MOTION_DETECT_COOLDOWN_MS / 100; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_task_wdt_reset(); // Reset watchdog during cooldown
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}