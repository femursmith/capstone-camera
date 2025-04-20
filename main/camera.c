// camera.c

#include "recorder.h" // Include recorder.h to get playback_active flag
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "driver/gpio.h" // Include for gpio_get_level

#include "peer_connection.h"
#include "events.h" // Include events.h for upload_image and PIR_SENSOR_PIN

extern PeerConnection *g_pc;
extern int gDataChannelOpened;
extern PeerConnectionState eState;
extern int get_timestamp(); // Assuming this exists elsewhere
extern SemaphoreHandle_t xSemaphore; // WebRTC send semaphore
static const char *TAG = "Camera";

// Queues (Ensure they are initialized in camera_init)
QueueHandle_t streamingQueue = NULL;
QueueHandle_t recordingQueue = NULL;
QueueHandle_t eventQueue = NULL;

#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 10
#define CAM_PIN_SIOD 40
#define CAM_PIN_SIOC 39
#define CAM_PIN_D7 48
#define CAM_PIN_D6 11
#define CAM_PIN_D5 12
#define CAM_PIN_D4 14
#define CAM_PIN_D3 16
#define CAM_PIN_D2 18
#define CAM_PIN_D1 17
#define CAM_PIN_D0 15
#define CAM_PIN_VSYNC 38
#define CAM_PIN_HREF 47
#define CAM_PIN_PCLK 13

static camera_config_t camera_config = {
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_pclk = CAM_PIN_PCLK,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .xclk_freq_hz = 20000000,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_SVGA,
    .jpeg_quality = 15,
    .fb_count = 4,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY
};

esp_err_t camera_init() {
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    streamingQueue = xQueueCreate(10, sizeof(camera_fb_t*));
    if (streamingQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create streaming queue");
        return ESP_FAIL;
    }
    recordingQueue = xQueueCreate(100, sizeof(camera_fb_t*));
    if (recordingQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create recording queue");
        return ESP_FAIL;
    }

    eventQueue = xQueueCreate(1, sizeof(camera_fb_t*));
    if (recordingQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create recording queue");
        return ESP_FAIL;
    }
    

    return ESP_OK;
}



void unified_camera_task(void *pvParameters) {
    ESP_LOGI(TAG, "Unified camera task started on Core %d", xPortGetCoreID());

    const TickType_t EVENT_INTERVAL = pdMS_TO_TICKS(5000);  // 5s between event uploads if PIR stays high
    TickType_t last_event_tick = 0;

    // --- Initial Playback Start (for testing) ---
    // Wait a bit for system to stabilize before starting playback
    vTaskDelay(pdMS_TO_TICKS(15000));
    ESP_LOGI(TAG, "Attempting to start playback...");
    // --- IMPORTANT: Replace with an actual AVI file path on your SD card ---
    // start_playback("/sdcard/2023-10-27_10-30-00_SVGA_10_30.avi");
    // start_playback(NULL); // Start from beginning of list
    start_playback(NULL); // Start specific file


    for (;;) {
        TickType_t now = xTaskGetTickCount();
        bool streaming_needed = (eState == PEER_CONNECTION_COMPLETED) && gDataChannelOpened;
        bool recording_needed = doRecording && forceRecord; // Simplified condition
        bool event_needed = (gpio_get_level(PIR_SENSOR_PIN) == 1) && ((now - last_event_tick) >= EVENT_INTERVAL);

        // --- Handle Streaming ---
        if (streaming_needed) {
            if (playback_active) {
                // Consume from playback task via streamingQueue
                camera_fb_t *fb_playback = NULL;
                ESP_LOGD(TAG, "Waiting for playback frame...");
                if (xQueueReceive(streamingQueue, &fb_playback, pdMS_TO_TICKS(100)) == pdTRUE) { // Use timeout
                    if (fb_playback && fb_playback->buf && fb_playback->len > 0) {
                        ESP_LOGI(TAG, "Streaming playback frame %zu bytes", fb_playback->len);
                        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(500)) == pdTRUE) { // Use timeout for semaphore
                            peer_connection_datachannel_send(g_pc, (char*)fb_playback->buf, fb_playback->len);
                            xSemaphoreGive(xSemaphore);
                        } else {
                            ESP_LOGW(TAG, "Failed to get WebRTC semaphore for playback frame.");
                        }
                        // Free the frame buffer allocated by playback_task
                        heap_caps_free(fb_playback->buf);
                        heap_caps_free(fb_playback);
                    } else {
                         ESP_LOGW(TAG, "Received invalid frame from playback queue.");
                         if(fb_playback) heap_caps_free(fb_playback); // Free struct if buf was null
                    }
                } else {
                    // Queue empty or timeout - maybe playback paused or ended?
                     ESP_LOGD(TAG, "Playback streaming queue empty or timed out.");
                     if (!playback_active) {
                         ESP_LOGI(TAG, "Playback seems to have stopped.");
                         // Optionally add logic here if needed when playback stops
                     }
                }
            } else {
                // Consume from live camera
                camera_fb_t *fb = esp_camera_fb_get();
                if (!fb) {
                    ESP_LOGE(TAG, "Live camera capture failed");
                    vTaskDelay(pdMS_TO_TICKS(100)); // Delay on failure
                    continue;
                }

                ESP_LOGD(TAG, "Live Streaming frame %d bytes", fb->len);
                if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(500)) == pdTRUE) {
                    peer_connection_datachannel_send(g_pc, (char*)fb->buf, fb->len);
                    xSemaphoreGive(xSemaphore);
                } else {
                    ESP_LOGW(TAG, "Failed to get WebRTC semaphore for live frame.");
                }

                // --- Handle Recording (from live camera frame) ---
                if (recording_needed) {
                    // Need to copy the frame buffer for the recording queue
                    camera_fb_t *fb_copy_rec = (camera_fb_t*)heap_caps_malloc(sizeof(camera_fb_t), MALLOC_CAP_SPIRAM);
                    if (fb_copy_rec) {
                        *fb_copy_rec = *fb; // Copy metadata
                        fb_copy_rec->buf = (uint8_t*)heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM);
                        if (fb_copy_rec->buf) {
                            memcpy(fb_copy_rec->buf, fb->buf, fb->len);
                            ESP_LOGD(TAG, "Enqueuing frame for recording (%zu bytes)", fb_copy_rec->len);
                            if (xQueueSend(recordingQueue, &fb_copy_rec, pdMS_TO_TICKS(50)) != pdTRUE) {
                                ESP_LOGW(TAG, "Failed to enqueue frame for recording (queue full?).");
                                heap_caps_free(fb_copy_rec->buf); // Free copy if queue fails
                                heap_caps_free(fb_copy_rec);
                            }
                            // Recorder task is responsible for freeing the buffer after processing
                        } else {
                            ESP_LOGE(TAG, "Failed to allocate buffer for recording copy!");
                            heap_caps_free(fb_copy_rec);
                        }
                    } else {
                         ESP_LOGE(TAG, "Failed to allocate fb_copy_rec struct!");
                    }
                }

                // --- Handle Event Upload (from live camera frame) ---
                if (event_needed) {
                    last_event_tick = now; // Update last sent time immediately
                    // Need to copy the frame buffer for the event queue/upload function
                     camera_fb_t *fb_copy_evt = (camera_fb_t*)heap_caps_malloc(sizeof(camera_fb_t), MALLOC_CAP_DEFAULT); // Use default memory for event?
                    if (fb_copy_evt) {
                        *fb_copy_evt = *fb; // Copy metadata
                        // Use default malloc for event upload task if it doesn't need PSRAM? Check upload_image constraints.
                        // Let's assume default memory is okay for upload task.
                        fb_copy_evt->buf = (uint8_t*)malloc(fb->len);
                        if (fb_copy_evt->buf) {
                            memcpy(fb_copy_evt->buf, fb->buf, fb->len);
                            ESP_LOGI(TAG, "Processing frame for event upload (%zu bytes)", fb_copy_evt->len);

                            // Directly call upload function (simpler than queue if task structure allows)
                            if (upload_image(fb_copy_evt) == ESP_OK) {
                                ESP_LOGI(TAG, "Event image upload successful");
                            } else {
                                ESP_LOGE(TAG, "Event image upload failed");
                            }
                            free(fb_copy_evt->buf); // Free copy after upload attempt
                            free(fb_copy_evt);
                        } else {
                            ESP_LOGE(TAG, "Failed to allocate buffer for event copy!");
                            free(fb_copy_evt);
                        }
                    } else {
                         ESP_LOGE(TAG, "Failed to allocate fb_copy_evt struct!");
                    }
                }

                // Return the original live camera frame buffer
                esp_camera_fb_return(fb);

            } // End if/else playback_active
        } else {
             // --- No Streaming Needed ---
             // Still capture frames if recording or event detection is needed, but don't stream.
             if (recording_needed || event_needed) {
                  camera_fb_t *fb = esp_camera_fb_get();
                  if (!fb) {
                      ESP_LOGE(TAG, "Live camera capture failed (no streaming)");
                      vTaskDelay(pdMS_TO_TICKS(100));
                      continue;
                  }

                  // Handle Recording (Copy needed)
                  if (recording_needed) {
                      camera_fb_t *fb_copy_rec = (camera_fb_t*)heap_caps_malloc(sizeof(camera_fb_t), MALLOC_CAP_SPIRAM);
                      if (fb_copy_rec) {
                         *fb_copy_rec = *fb;
                         fb_copy_rec->buf = (uint8_t*)heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM);
                         if (fb_copy_rec->buf) {
                             memcpy(fb_copy_rec->buf, fb->buf, fb->len);
                             if (xQueueSend(recordingQueue, &fb_copy_rec, pdMS_TO_TICKS(50)) != pdTRUE) {
                                  ESP_LOGW(TAG, "Failed to enqueue frame for recording (no stream).");
                                  heap_caps_free(fb_copy_rec->buf);
                                  heap_caps_free(fb_copy_rec);
                             }
                         } else { heap_caps_free(fb_copy_rec); ESP_LOGE(TAG, "Failed alloc rec buf (no stream)"); }
                      } else { ESP_LOGE(TAG, "Failed alloc rec struct (no stream)"); }
                  }

                  // Handle Event Upload (Copy needed)
                  if (event_needed) {
                      last_event_tick = now;
                      camera_fb_t *fb_copy_evt = (camera_fb_t*)malloc(sizeof(camera_fb_t));
                      if (fb_copy_evt) {
                          *fb_copy_evt = *fb;
                          fb_copy_evt->buf = (uint8_t*)malloc(fb->len);
                          if (fb_copy_evt->buf) {
                               memcpy(fb_copy_evt->buf, fb->buf, fb->len);
                               if (upload_image(fb_copy_evt) == ESP_OK) ESP_LOGI(TAG, "Event upload OK (no stream)");
                               else ESP_LOGE(TAG, "Event upload FAIL (no stream)");
                               free(fb_copy_evt->buf);
                               free(fb_copy_evt);
                          } else { free(fb_copy_evt); ESP_LOGE(TAG, "Failed alloc evt buf (no stream)"); }
                      } else { ESP_LOGE(TAG, "Failed alloc evt struct (no stream)"); }
                  }

                  esp_camera_fb_return(fb); // Return the live frame
             } else {
                  // Neither streaming, recording, nor event needed. Maybe idle or just delay?
                  // To avoid constant polling of camera if nothing is needed:
                  vTaskDelay(pdMS_TO_TICKS(100)); // Add delay if no activity needed
             }
        }

        // General task delay (adjust as needed)
        // A small delay helps prevent high CPU usage if loops run very fast (e.g., during playback queue waits)
        vTaskDelay(pdMS_TO_TICKS(5));

    } // End for(;;)
}