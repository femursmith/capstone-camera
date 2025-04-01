#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <sys/time.h>
#include <sys/param.h>
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

#include "peer.h"
#include "wifimanager.h"

static const char *TAG = "webrtc";

static TaskHandle_t xPcTaskHandle = NULL;
static TaskHandle_t xPsTaskHandle = NULL;
static TaskHandle_t xCameraTaskHandle = NULL;
static TaskHandle_t xAudioTaskHandle = NULL;
static TaskHandle_t xUartTaskHandle = NULL;





extern esp_err_t camera_init();
extern esp_err_t audio_init();
extern void mjpeg_stream_task(void *pvParameters);
extern void audio_task(void *pvParameters);
extern void uart_request_task(void *pvParameters);
SemaphoreHandle_t xSemaphore = NULL;

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

    vTaskDelay(pdMS_TO_TICKS(10));
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

void app_main(void) {

  handle_wifi_connect();

  PeerConfiguration config = {
   .ice_servers = {{ .urls = "stun:stun.l.google.com:19302" }},
    .datachannel = DATA_CHANNEL_BINARY, 
   };
   
  

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

  






  xSemaphore = xSemaphoreCreateMutex();

  peer_init();


#if defined(CONFIG_ESP32S3_XIAO_SENSE)
  audio_init();
#endif

  g_pc = peer_connection_create(&config);
  peer_connection_oniceconnectionstatechange(g_pc, oniceconnectionstatechange);
  peer_connection_ondatachannel(g_pc, onmessage, onopen, onclose);

 
  load_camera_id();
  if (stored_camera_id[0] == '\0') {
    ESP_LOGE(TAG, "No camera ID stored in NVS. Please set the camera ID using the web interface.");
    return;
  }

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
  
  //peer_connection_create_offer(deviceid,g_pc);

#if defined(CONFIG_ESP32S3_XIAO_SENSE)
  xTaskCreatePinnedToCore(audio_task, "audio", 8192, NULL, 7, &xAudioTaskHandle, 0);
#endif

  xTaskCreatePinnedToCore(mjpeg_stream_task, "camera", 4096, NULL, 8, &xCameraTaskHandle, 1);

  xTaskCreatePinnedToCore(peer_connection_task, "peer_connection", 8192, NULL, 5, &xPcTaskHandle, 1);

  xTaskCreatePinnedToCore(peer_signaling_task, "peer_signaling", 8192, NULL, 6, &xPsTaskHandle, 1);

  xTaskCreatePinnedToCore(&uart_request_task, "uart_request_task", 4096, NULL, 5,&xUartTaskHandle , 1);

  ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());


  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
