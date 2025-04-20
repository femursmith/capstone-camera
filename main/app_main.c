#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <sys/time.h>
#include <sys/param.h>
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "mdns.h"
#include "esp_log.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "protocol_examples_common.h"
#include "recorder.h" 
#include "peer.h"
#include "wifimanager.h"
#include "events.h"



#define PLAYBACK_STACK_SIZE 6144
#define PLAYBACK_PRIORITY 4

static const char *TAG = "webrtc";

static TaskHandle_t xPcTaskHandle = NULL;
static TaskHandle_t xPsTaskHandle = NULL;
// static TaskHandle_t xCameraTaskHandle = NULL; // Replaced by unified task
// static TaskHandle_t xAudioTaskHandle = NULL; // Assuming no audio for now
// static TaskHandle_t xUartTaskHandle = NULL; // If needed
static TaskHandle_t xRecordTaskHandle = NULL; // Handle for the *recording* part (captureTask in recorder.cpp)
static TaskHandle_t xUnifiedCameraTaskHandle = NULL;
static TaskHandle_t xPlaybackTaskHandle = NULL; // ADD THIS LINE

// ... extern declarations ...
extern esp_err_t camera_init();
// extern esp_err_t audio_init(); // If needed
extern void unified_camera_task(void *pvParameters);
// extern void mjpeg_stream_task(void *pvParameters); // Old camera task
// extern void audio_task(void *pvParameters); // If needed
// extern void uart_request_task(void *pvParameters); // If needed
// extern void camera_task(void *pvParameters); // Old camera task
// extern void streaming_task(void *pvParameters); // Old streaming task
extern void upload_image_task(void* pvParameters); // Keep if separate task used
extern void playback_task(void *pvParameters); // Declaration for playback task
extern bool prepRecording(); // Ensure prepRecording is available
SemaphoreHandle_t xSemaphore = NULL;
extern TaskHandle_t captureHandle;

PeerConnection *g_pc;
PeerConnectionState eState = PEER_CONNECTION_CLOSED;
int gDataChannelOpened = 0;

int64_t get_timestamp() {

  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

static void oniceconnectionstatechange(PeerConnectionState state, void *user_data) {

  ESP_LOGI(TAG, "PeerConnectionState: %d", state);
  eState = state;

  // not support datachannel close event
  if (eState != PEER_CONNECTION_COMPLETED) {
    gDataChannelOpened = 0;
  }
}

static void onmessage(char *msg, size_t len, void *userdata, uint16_t sid) {

  ESP_LOGI(TAG, "Datachannel message: %.*s", len, msg);
}

void onopen(void *userdata) {
 
  ESP_LOGI(TAG, "Datachannel opened");
  gDataChannelOpened = 1;
}

static void onclose(void *userdata) {
 
}

void peer_signaling_task(void *arg) {

  ESP_LOGI(TAG, "peer_signaling_task started");

  for(;;) {

    peer_signaling_loop();

    vTaskDelay(pdMS_TO_TICKS(1));
  }

}

void peer_connection_task(void *arg) {

  ESP_LOGI(TAG, "peer_connection_task started");

  for(;;) {

    if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
        peer_connection_loop(g_pc);
        xSemaphoreGive(xSemaphore);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void sync_time()
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;

    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d)", retry);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (timeinfo.tm_year >= (2020 - 1900)) {
        ESP_LOGI(TAG, "Time is synchronized.");
        char strftime_buf[64];
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        ESP_LOGI(TAG, "Current time: %s", strftime_buf);
    } else {
        ESP_LOGE(TAG, "Time sync failed");
    }
}


void app_main(void) {

  handle_wifi_connect();
  

 
  if (camera_init() != ESP_OK) {
    ESP_LOGE(TAG, "Camera initialization failed!");
    // Handle error appropriately
    return;
  }
  ESP_LOGI(TAG, "Camera initialized.");


   // Sync Time
   sync_time();
  

  ESP_LOGI(TAG, "[APP] Startup..");
  ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
  ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
  esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
  esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
  esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
  esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
  esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);





  
  PeerConfiguration config = {
    .ice_servers = {{ .urls = "stun:stun.l.google.com:19302" }},
    .datachannel = DATA_CHANNEL_BINARY,
  };

  

  


  // Initialize Event Handling (PIR sensor, upload init)
  upload_image_init(); // Configure PIR pin, create semaphore
  ESP_LOGI(TAG, "Event system initialized.");

  // Create Global Mutex for WebRTC sending
  xSemaphore = xSemaphoreCreateMutex();
  if(xSemaphore == NULL){
      ESP_LOGE(TAG, "Failed to create WebRTC mutex!");
      return;
  }

  // Initialize WebRTC Peer Connection
  peer_init(); // Initializes signaling client etc.

  
  g_pc = peer_connection_create(&config);
  peer_connection_oniceconnectionstatechange(g_pc, oniceconnectionstatechange);
  peer_connection_ondatachannel(g_pc, onmessage, onopen, onclose);

  // Load IDs and configure signaling
  load_camera_id();
  if (stored_camera_id[0] == '\0') {
    ESP_LOGE(TAG, "No camera ID stored in NVS.");
    // Handle error
    return;
  }
  load_user_id(); // Assuming this exists

  ServiceConfiguration service_config = SERVICE_CONFIG_DEFAULT();
  service_config.client_id = stored_camera_id;
  service_config.pc = g_pc;
  service_config.mqtt_url = "d457c1d9.ala.eu-central-1.emqxsl.com";
  service_config.mqtt_port = 8883;
  service_config.username = "Capstone-app-test-1";
  service_config.password = "Capstone-app-test-1";


  peer_signaling_set_config(&service_config);
  peer_signaling_join_channel();
  
  if (camera_id[0] != '\0') {
    save_camera_id(camera_id);
  }

  //Initialize SD Card and Recorder Components (including playback resources)
  if (recorder_init() != ESP_OK) { 
    // Initializes SD card mount
    ESP_LOGE(TAG, "Recorder SD initialization failed!");
    // Handle error appropriately
    return;
  }
  ESP_LOGI(TAG, "Recorder SD initialized.");
  if (!prepRecording()) { // Initializes buffers, semaphores, capture task
      ESP_LOGE(TAG, "Recorder preparation (buffers, tasks) failed!");
      return;
  }
  ESP_LOGI(TAG, "Recorder prepared (buffers, capture task started).");

  
  ESP_LOGI(TAG, "Creating Tasks...");

    // Signaling Task
    xTaskCreatePinnedToCore(peer_signaling_task, "peer_signaling", 8192, NULL, 9, &xPsTaskHandle, 1);

    // Peer Connection Task
    xTaskCreatePinnedToCore(peer_connection_task, "peer_connection", 8192, NULL, 8, &xPcTaskHandle, 1);

    // Unified Camera/Streaming/Recording/Event Task
    xTaskCreatePinnedToCore(unified_camera_task, "unified_camera", 8192, NULL, 5, &xUnifiedCameraTaskHandle, 0); // Core 0 for camera

    // Playback Task (Handles reading AVI and feeding streamingQueue)
    // Note: captureTask (recording) is created inside prepRecording()
    xTaskCreatePinnedToCore(playback_task, "playback", PLAYBACK_STACK_SIZE, NULL, PLAYBACK_PRIORITY, &playbackTaskHandle, 1); // Core 1 for file I/O

    // Event Upload Task (if running separately)
    // xTaskCreatePinnedToCore(upload_image_task, "upload", 4096, NULL, 6, NULL, 1); // Example priority

    ESP_LOGI(TAG, "[APP] Free memory after task creation: %d bytes", esp_get_free_heap_size());


    // Main loop idle (or add monitoring)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
         ESP_LOGD(TAG, "Main loop tick. Free heap: %d", esp_get_free_heap_size());
         // You could add logic here to check task states, restart if needed, etc.
    }
}
