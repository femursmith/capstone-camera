#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "ff.h"
#include "diskio.h"
#include "esp_vfs_fat.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include <stddef.h> 
#include <algorithm> 
#include <sys/stat.h> 
#include "recorder.h"
#include <dirent.h> // For directory listing
#include <vector>   // For storing filenames (requires C++)
#include <string>   // For std::string (requires C++)
#include <algorithm> // For std::sort (requires C++)
#include <sys/stat.h> // For stat

#include "peer_connection.h"

static const char *TAG_AVI = "recorder";

// --- Define appGlobals.h equivalents - minimal for recording ---
#define AVI_HEADER_LEN 310
#define CHUNK_HDR 8
#define FILE_NAME_LEN 64 // Ensure this is defined
#define MAX_JPEG (1024 * 1024)
#define RAMSIZE (128 * 1024) // Buffer size for recording
#define AVITEMP "/sdcard/avi_temp.avi"
#define AVI_EXT "avi"
#define FB_BUFFERS 2 // If applicable
#define CAPTURE_STACK_SIZE 4096
#define CAPTURE_PRI 2
#define FRAMESIZE_SVGA      (8)         /*!< SVGA 800x600     */ // Correct index might vary
#define FRAMESIZE_UXGA      (13)        /*!< UXGA 1600x1200   */ // Correct index might vary
#define STARTUP_FAIL "Startup Failed: "
#define SF_LEN 128
#define IDX_ENTRY 16 // bytes per index entry

// --- Global Recording Variables ---
bool forceRecord = false; // Recording enabled by setting this to true
int maxFrames = 300; // maximum number of frames in video before auto close
uint8_t FPS = 10; // Frames Per Second for recording
uint8_t fsizePtr = FRAMESIZE_SVGA; // Frame size index, default SVGA
uint8_t minSeconds = 5; // Minimum recording duration
bool doRecording = true; // Master record enable/disable
uint8_t xclkMhz = 20; // camera clock rate MHz
char camModel[10] = "OV5640";
bool ready = false;
static uint32_t vidSize;
static uint16_t frameCnt;
static uint32_t startTime;
static uint32_t wTimeTot;
static uint32_t oTime;
static uint32_t cTime;
static uint16_t frameInterval; // units of 0.1ms between frames
static size_t idxPtr[2]; // Index buffer pointers
static size_t idxOffset[2]; // Offset for index entries
static size_t moviSize[2]; // Size of 'movi' chunk (movie data)
static size_t indexLen[2]; // Length of the index
uint8_t* iSDbuffer = NULL; // Recording buffer
static size_t highPoint;
static FILE* aviFile_handle = NULL; // Recording file handle
static char aviFileName[FILE_NAME_LEN]; // Recording final filename
TaskHandle_t captureHandle = NULL;
static SemaphoreHandle_t readSemaphore; // Original recorder semaphore (if still needed)
SemaphoreHandle_t aviMutex = NULL;     // Mutex for AVI header/index build
bool isCapturing = false;

// --- Global Playback Variables ---
TaskHandle_t playbackTaskHandle = NULL;
volatile bool playback_active = false; // Indicates if playback is running
static char current_playback_file[FILE_NAME_LEN * 2] = {0}; // Path of the file being played
static SemaphoreHandle_t playbackControlSemaphore = NULL; // To signal start/stop to playback task
static bool stop_playback_request = false; // Flag to signal stop request

#define PIN_NUM_MISO  8
#define PIN_NUM_MOSI  9
#define PIN_NUM_CLK   7
#define PIN_NUM_CS    21


#define PLAYBACK_BUFFER_SIZE (32 * 1024) // Read buffer size for playback

// Dummy STORAGE structure
typedef struct {
    bool (*exists)(const char* path);
    FILE* (*open)(const char* path, const char* mode);
    size_t (*read)(FILE* fp, uint8_t* clientBuf, size_t buffSize);
    size_t (*write)(FILE* fp, const void* clientBuf, size_t buffSize);
    bool (*seek)(FILE* fp, long offset, int origin);
    size_t (*size)(FILE* fp);
    void (*close)(FILE* fp);
    bool (*remove)(const char* path);
    bool (*rename)(const char* oldpath, const char* newpath);
    bool (*mkdir)(const char* path);
} STORAGE_t;


// avi header data - from avi_generator.cpp
const uint8_t dcBuf[4] = {0x30, 0x30, 0x64, 0x63};   // 00dc
static const uint8_t idx1Buf[4] = {0x69, 0x64, 0x78, 0x31}; // idx1
static const uint8_t zeroBuf[4] = {0x00, 0x00, 0x00, 0x00}; // 0000
static uint8_t* idxBuf[2] = {NULL, NULL};
extern PeerConnectionState eState;

// aviHeader template - from avi_generator.cpp
uint8_t aviHeader[AVI_HEADER_LEN] = { // AVI header template
    0x52, 0x49, 0x46, 0x46, 0x00, 0x00, 0x00, 0x00, 0x41, 0x56, 0x49, 0x20, 0x4C, 0x49, 0x53, 0x54,
    0x16, 0x01, 0x00, 0x00, 0x68, 0x64, 0x72, 0x6C, 0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x6C, 0x00, 0x00, 0x00,
    0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73,
    0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
    0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x18, 0x00, 0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x4C, 0x49, 0x53, 0x54, 0x56, 0x00, 0x00, 0x00, 
    0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x61, 0x75, 0x64, 0x73,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x11, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x11, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
    0x12, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x11, 0x2B, 0x00, 0x00, 0x11, 0x2B, 0x00, 0x00,
    0x02, 0x00, 0x10, 0x00, 0x00, 0x00, 
    0x4C, 0x49, 0x53, 0x54, 0x00, 0x00, 0x00, 0x00, 0x6D, 0x6F, 0x76, 0x69,
  };

// frameSizeData - from avi_generator.cpp
struct frameSizeStruct { // Correct struct definition
    uint8_t frameWidth[2];
    uint8_t frameHeight[2];
};
struct frameSizeStruct frameSizeData[] = { // Only include SVGA and UXGA for simplicity if needed
    {{0x80, 0x02}, {0xE0, 0x01}}, // vga (index 8)
    {{0x20, 0x03}, {0x58, 0x02}}, // svga (index 9)
    {{0x40, 0x06}, {0xB0, 0x04}}  // uxga (index 13)
};


STORAGE_t STORAGE = {
    .exists = STORAGE_exists,
    .open = STORAGE_open,
    .read = STORAGE_read,
    .write = STORAGE_write,
    .seek = STORAGE_seek,
    .size = STORAGE_size,
    .close = STORAGE_close,
    .remove = STORAGE_remove,
    .rename = STORAGE_rename,
    .mkdir = STORAGE_mkdir
};


void rc_decrement(rc_camera_fb_t *rc_fb) {
    xSemaphoreTake(rc_fb->mutex, portMAX_DELAY);
    rc_fb->ref_count--;
    int count = rc_fb->ref_count;
    xSemaphoreGive(rc_fb->mutex);
    if (count == 0) {
        // When no task is using this frame, return it to the camera driver.
        esp_camera_fb_return(rc_fb->fb);
        vSemaphoreDelete(rc_fb->mutex);
        vPortFree(rc_fb); // free the wrapper structure if allocated dynamically
        
    
   
        ESP_LOGI(TAG_AVI, "Frame buffer free heap space: %d bytes", esp_get_free_heap_size());
    }
}



esp_err_t recorder_init() {
    esp_err_t ret;
    // Use designated initializers for clarity and safety
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        // .disk_status_check_enable = false // Explicitly initialize if needed (check defaults)
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // Use designated initializers for clarity and safety
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
        // .data4_io_num = -1, // Initialize others if needed
        // .data5_io_num = -1,
        // .data6_io_num = -1,
        // .data7_io_num = -1,
        // .flags = 0,
        // .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        // .intr_flags = 0
    };

    ret = spi_bus_initialize(static_cast<spi_host_device_t>(host.slot), &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_AVI, "Failed to initialize SPI bus.");
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = static_cast<gpio_num_t>(PIN_NUM_CS);
    slot_config.host_id = static_cast<spi_host_device_t>(host.slot);

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_AVI, "Failed to mount filesystem: %s", esp_err_to_name(ret));
        // Consider cleanup: spi_bus_free ?
        return ret;
    }
    ESP_LOGI(TAG_AVI, "Filesystem mounted successfully");
    sdmmc_card_print_info(stdout, card);


    if (playbackControlSemaphore == NULL) {
        playbackControlSemaphore = xSemaphoreCreateBinary();
        if (playbackControlSemaphore == NULL) {
             ESP_LOGE(TAG_AVI, "Failed to create playback control semaphore!");
             vSemaphoreDelete(aviMutex); // Clean up previous mutex
             aviMutex = NULL;
             return false;
        }
   }


    // Frame queue is created in camera_init, not here.

    return ESP_OK;
}


static void deleteTask(TaskHandle_t thisTaskHandle) {
    if (thisTaskHandle != NULL) {
        TaskHandle_t handle_copy = thisTaskHandle; // Copy handle in case it's set to NULL concurrently
        // Setting handle to NULL here isn't thread-safe if another task uses it
        vTaskDelete(handle_copy);
        ESP_LOGI(TAG_AVI, "Task deleted: %p", (void*)handle_copy);
    } else {
        ESP_LOGW(TAG_AVI, "Attempted to delete a NULL task handle.");
    }
}

void endTasks() {
    // Stop playback if active
    stop_playback();
    vTaskDelay(pdMS_TO_TICKS(200)); // Give time for task to potentially stop

    // Delete tasks
    deleteTask(captureHandle);
    deleteTask(playbackTaskHandle); // Delete playback task

    // Free buffers and semaphores
    if (idxBuf[0] != NULL) heap_caps_free(idxBuf[0]);
    idxBuf[0] = NULL;
    if (iSDbuffer != NULL) heap_caps_free(iSDbuffer); // Recording buffer
    iSDbuffer = NULL;

    if (aviMutex != NULL) vSemaphoreDelete(aviMutex);
    aviMutex = NULL;
    if (readSemaphore != NULL) vSemaphoreDelete(readSemaphore); // If used by recorder
    readSemaphore = NULL;
     if (playbackControlSemaphore != NULL) vSemaphoreDelete(playbackControlSemaphore);
     playbackControlSemaphore = NULL;

    // ... potentially unmount SD card here if appropriate ...
    ESP_LOGI(TAG_AVI, "SD card related tasks ended and resources potentially freed.");
}





// --- Implement minimal set of functions for recording --- (Function implementations - same as before, but corrected byte* to uint8_t*)
void prepAviIndex(bool isTL) {
    if (idxBuf[isTL] == NULL) idxBuf[0] = (uint8_t*)heap_caps_malloc((maxFrames+1)*IDX_ENTRY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (idxBuf[isTL] == NULL) {
        ESP_LOGE(TAG_AVI, "Failed to allocate index buffer");
        return;
    }
    memcpy(idxBuf[isTL], idx1Buf, 4);
    idxPtr[isTL] = CHUNK_HDR;
    moviSize[isTL] = indexLen[isTL] = 0;
    idxOffset[isTL] = 4;
}

void buildAviHdr(uint8_t theFPS, uint8_t frameTypeIndex, uint16_t frameCount, bool isTL) {
    // Ensure frameTypeIndex is valid
     if (frameTypeIndex >= sizeof(frameSizeData)/sizeof(frameSizeData[0])) {
        ESP_LOGE(TAG_AVI,"Invalid frameTypeIndex %d for buildAviHdr", frameTypeIndex);
        frameTypeIndex = FRAMESIZE_SVGA; // Default to SVGA
    }
    size_t total_index_size = frameCount * IDX_ENTRY + CHUNK_HDR; // 'idx1' + size + entries
    // Size of movi data (frames + headers) + 4 bytes for 'movi' LIST identifier itself
    size_t movi_list_content_size = moviSize[isTL]; // moviSize[isTL] should hold sum of (chunk_hdr + frame_data_size)
    size_t movi_list_total_size = movi_list_content_size + 4; // +4 for 'movi' ID

    // RIFF size = 'AVI ' + size_of_hdrl_LIST + size_of_movi_LIST + size_of_idx1_CHUNK
    // Note: AVI_HEADER_LEN includes the initial RIFF+AVI+LIST+hdrl structure up to *before* the movi LIST
    // The template already includes placeholders for LIST movi (4 bytes LIST, 4 bytes size, 4 bytes movi) -> 12 bytes
    // So, RIFF size = (AVI_HEADER_LEN - 12) + movi_list_total_size + total_index_size

    // Let's rethink the AVI_HEADER_LEN. It seems to contain the 'hdrl' list *and* the start of the 'movi' list declaration.
    // RIFF [SIZE] AVI  LIST [hdrl_list_size] hdrl [...] LIST [movi_list_size] movi [FRAME_DATA...] idx1 [idx_size] [INDEX_DATA...]
    // Size @ offset 4: Total file size - 8 (exclude RIFF and SIZE itself)
    // Size @ offset 0x12E ('movi' LIST size): Size of 'movi' content + 4 (for 'movi' ID) - This is movi_list_total_size
    size_t riffSize = (AVI_HEADER_LEN - 8) // Size of header structure up to 'movi' list definition, excluding 'RIFF' and initial size field
                     + movi_list_total_size // Size of the 'movi' list (including 'movi' ID and data)
                     + total_index_size; // Size of the index chunk (including 'idx1' ID and size)

    memcpy(aviHeader+4, &riffSize, 4); // Overall file size - 8 bytes

    uint32_t usecs = (uint32_t)round(1000000.0f / theFPS);
    memcpy(aviHeader+0x20, &usecs, 4); // Microseconds per frame (avih)
    memcpy(aviHeader+0x30, &frameCount, 2); // Total frames (avih) - WORD, check size
    memcpy(aviHeader+0x84, &theFPS, 1); // Suggested frame rate (strh) - BYTE
    memcpy(aviHeader+0x8C, &frameCount, 2); // Length (frames) (strh) - WORD, check size

    // Size of the 'movi' LIST chunk (including the 'movi' type identifier)
    memcpy(aviHeader+0x12E, &movi_list_total_size, 4); // LIST size for 'movi'

    // Frame dimensions
    memcpy(aviHeader+0x40, frameSizeData[frameTypeIndex].frameWidth, 2); // Width (avih)
    memcpy(aviHeader+0xA8, frameSizeData[frameTypeIndex].frameWidth, 2); // Width (strf)
    memcpy(aviHeader+0x44, frameSizeData[frameTypeIndex].frameHeight, 2); // Height (avih)
    memcpy(aviHeader+0xAC, frameSizeData[frameTypeIndex].frameHeight, 2); // Height (strf)

    // Reset internal counters used only during header build
    // moviSize[isTL] = idxPtr[isTL] = 0; // NO! moviSize is needed for the calculation above. Don't reset here.
    // idxOffset[isTL] = 4;
}

// void buildAviHdr(uint8_t FPS, uint8_t frameType, uint16_t frameCnt, bool isTL) {
//     size_t aviSize = moviSize[isTL] + AVI_HEADER_LEN + ((CHUNK_HDR+IDX_ENTRY) * frameCnt);
//     memcpy(aviHeader+4, &aviSize, 4);
//     uint32_t usecs = (uint32_t)round(1000000.0f / FPS);
//     memcpy(aviHeader+0x20, &usecs, 4);
//     memcpy(aviHeader+0x30, &frameCnt, 2);
//     memcpy(aviHeader+0x8C, &frameCnt, 2);
//     memcpy(aviHeader+0x84, &FPS, 1);
//     uint32_t dataSize = moviSize[isTL] + (frameCnt * CHUNK_HDR) + 4; 
//     memcpy(aviHeader+0x12E, &dataSize, 4);

//     memcpy(aviHeader+0x40, frameSizeData[frameType].frameWidth, 2);
//     memcpy(aviHeader+0xA8, frameSizeData[frameType].frameWidth, 2);
//     memcpy(aviHeader+0x44, frameSizeData[frameType].frameHeight, 2);
//     memcpy(aviHeader+0xAC, frameSizeData[frameType].frameHeight, 2); 

//     moviSize[isTL] = idxPtr[isTL] = 0;
//     idxOffset[isTL] = 4;
// }

void buildAviIdx(size_t dataSize, bool isVid, bool isTL) {
    moviSize[isTL] += dataSize;
    memcpy(idxBuf[isTL]+idxPtr[isTL], dcBuf, 4);
    memcpy(idxBuf[isTL]+idxPtr[isTL]+4, zeroBuf, 4);
    memcpy(idxBuf[isTL]+idxPtr[isTL]+8, &idxOffset[isTL], 4);
    memcpy(idxBuf[isTL]+idxPtr[isTL]+12, &dataSize, 4);
    idxOffset[isTL] += dataSize + CHUNK_HDR;
    idxPtr[isTL] += IDX_ENTRY;
}


size_t writeAviIndex(uint8_t* clientBuf, size_t buffSize, bool isTL) {
    // This function is used during closeAvi to write the completed index buffer to the file.
    // indexLen should have been set by finalizeAviIndex.
    // idxPtr should be reset to 0 by finalizeAviIndex before calling this.
    if (idxBuf[isTL] == NULL || indexLen[isTL] == 0) return 0;

    size_t remaining_index_bytes = indexLen[isTL] - idxPtr[isTL];
    if (remaining_index_bytes == 0) {
        return 0; // All index data already "written" (copied out)
    }

    size_t bytes_to_copy = std::min(remaining_index_bytes, buffSize);
    memcpy(clientBuf, idxBuf[isTL] + idxPtr[isTL], bytes_to_copy);
    idxPtr[isTL] += bytes_to_copy; // Advance the internal pointer

    return bytes_to_copy;
}


// size_t writeAviIndex(uint8_t* clientBuf, size_t buffSize, bool isTL) {
//     if (idxPtr[0] < indexLen[0]) {
//         if (indexLen[isTL]-idxPtr[isTL] > buffSize) {
//             memcpy(clientBuf, idxBuf[isTL]+idxPtr[isTL], buffSize);
//             idxPtr[isTL] += buffSize;
//             return buffSize;
//         } else {
//             size_t final = indexLen[isTL]-idxPtr[isTL];
//             memcpy(clientBuf, idxBuf[isTL]+idxPtr[isTL], final);
//             idxPtr[isTL] = indexLen[isTL];
//             return final;
//         }
//     }
//     return idxPtr[isTL] = 0;
// }

void finalizeAviIndex(uint16_t frameCnt, bool isTL) {
    if (idxBuf[isTL] == NULL) return;
    uint32_t sizeOfIndex = frameCnt*IDX_ENTRY;
    memcpy(idxBuf[isTL]+4, &sizeOfIndex, 4);
    indexLen[isTL] = sizeOfIndex + CHUNK_HDR;
    idxPtr[isTL] = 0;
}

static IRAM_ATTR void frameISR(void* arg) { // Removed IRAM_ATTR from the second definition
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(captureHandle, &xHigherPriorityTaskWoken); // Corrected function name - removed assignment
    if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

void controlFrameTimer(bool restartTimer) {
    static esp_timer_handle_t frameTimer = NULL;

    if (frameTimer) {
        esp_timer_stop(frameTimer);
        esp_timer_delete(frameTimer);
        frameTimer = NULL;
    }

    if (restartTimer) {
        esp_timer_create_args_t frame_timer_args = {
            .callback = &frameISR,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "frame_timer",
            .skip_unhandled_events = false,
        };
        ESP_ERROR_CHECK(esp_timer_create(&frame_timer_args, &frameTimer));

        frameInterval = 10000 / FPS;
        frameInterval *= 100;
        ESP_LOGI(TAG_AVI, "Frame timer interval %llu us for FPS %u", (uint64_t)frameInterval, FPS);
        ESP_ERROR_CHECK(esp_timer_start_periodic(frameTimer, frameInterval));


    }
}

// recorder.cpp

// ... (previous code) ...

static void openAvi() {
    char partName[FILE_NAME_LEN]; // Buffer for date string
    oTime = esp_timer_get_time() / 1000; // Record start time for opening

    dateFormat(partName, sizeof(partName), true); // Get date part YYYY-MM-DD
    char dateDirPath[FILE_NAME_LEN]; // Buffer for full directory path
    snprintf(dateDirPath, sizeof(dateDirPath), "%s/%s", MOUNT_POINT, partName);
    if (!STORAGE.exists(dateDirPath)) {
        ESP_LOGI(TAG_AVI, "Creating directory: %s", dateDirPath);
        if (!STORAGE.mkdir(dateDirPath)) {
             ESP_LOGE(TAG_AVI, "Failed to create directory: %s", dateDirPath);
        }
    }

    ESP_LOGI(TAG_AVI, "Opening temporary AVI file for recording: %s", AVITEMP);
    if (STORAGE.exists(AVITEMP)) {
        ESP_LOGW(TAG_AVI, "Temporary file %s exists, removing.", AVITEMP);
        STORAGE.remove(AVITEMP);
    }

    aviFile_handle = STORAGE.open(AVITEMP, "wb");
    if (aviFile_handle == NULL) { /* ... error handling ... */ return; }

    oTime = (esp_timer_get_time() / 1000) - oTime;
    ESP_LOGI(TAG_AVI, "File opening time: %lu ms", oTime);

    startTime = esp_timer_get_time() / 1000;
    frameCnt = 0; wTimeTot = 0; vidSize = 0;
    highPoint = AVI_HEADER_LEN; // Set buffer offset for header --> SHOULD BE 310
    ESP_LOGI(TAG_AVI, "Initial highPoint set to: %zu", highPoint); // Log initial offset
    prepAviIndex(false);

    ESP_LOGI(TAG_AVI, "Writing AVI header placeholder (%d bytes)...", AVI_HEADER_LEN);
    size_t header_written = STORAGE.write(aviFile_handle, aviHeader, AVI_HEADER_LEN);
    if (header_written != AVI_HEADER_LEN) { /* ... error handling ... */ return; }
    // --- Add flush to ensure header is physically written ---
    if(fflush(aviFile_handle) != 0) {
        ESP_LOGW(TAG_AVI,"fflush after header write failed!");
    } else {
        ESP_LOGI(TAG_AVI, "fflush after header write successful.");
    }

    long current_pos = ftell(aviFile_handle);
    ESP_LOGI(TAG_AVI, "File position after header write: %ld (Expected %d)", current_pos, AVI_HEADER_LEN);

    highPoint = 0;
    ESP_LOGI(TAG_AVI, "iSDbuffer highPoint initialized to: %zu", highPoint);

    prepAviIndex(false); // Prepare index structure in memory
    startTime = esp_timer_get_time() / 1000;
    frameCnt = 0; wTimeTot = 0; vidSize = 0;
    // --- Log file size after writing header ---
    size_t size_after_header = STORAGE_size(aviFile_handle);
    ESP_LOGI(TAG_AVI, "File size after writing header: %zu bytes", size_after_header);
    if(size_after_header != AVI_HEADER_LEN) {
         ESP_LOGW(TAG_AVI,"File size mismatch after header write!");
    }
    ESP_LOGI(TAG_AVI, "AVI header placeholder written successfully.");
}


static void saveFrame(camera_fb_t* fb) {

    bool is_first_frame = (frameCnt == 0);
    if (is_first_frame) {
        ESP_LOGI(TAG_AVI, "*** Processing FIRST frame ***");
        ESP_LOGI(TAG_AVI, "    highPoint before saveFrame: %zu", highPoint);
    }
     ESP_LOGD(TAG_AVI, "Frame %u: highPoint=%zu, fb->len=%zu", frameCnt + 1, highPoint, fb->len);
    // --- End check ---

    if (!iSDbuffer || !aviFile_handle) { /* ... error handling ... */ return; }

    uint32_t writeStartTime = esp_timer_get_time();

    uint16_t filler = (4 - (fb->len & 0x00000003)) & 0x00000003;
    size_t jpegChunkSize = fb->len + filler;

    if (is_first_frame) {
        ESP_LOGI(TAG_AVI, "    Writing '00dc' header at buffer offset %zu", highPoint);
        ESP_LOGI(TAG_AVI, "    jpegChunkSize: %zu (len=%zu, fill=%u)", jpegChunkSize, fb->len, filler);
   }

    

    size_t total_chunk_size = CHUNK_HDR + jpegChunkSize; // Total bytes needed for this chunk

    // --- Buffer Flushing Logic ---
    // Check if adding this ENTIRE chunk exceeds the buffer capacity.
    // OR, more simply, check if adding just the header exceeds it first,
    // then handle the data possibly spanning buffer writes. Let's try the simpler check first:
    // If the buffer doesn't have space for *at least* the header, flush it.
    // A more robust check: if current buffer usage + this chunk's total size > buffer capacity, flush first.
    if (highPoint + total_chunk_size > RAMSIZE && highPoint > 0) {
         // Flush the existing buffer content *before* adding the new chunk
         ESP_LOGD(TAG_AVI,"Buffer needs flushing before adding chunk (highPoint=%zu, needed=%zu). Writing %zu bytes.", highPoint, total_chunk_size, highPoint);
         size_t data_written = STORAGE.write(aviFile_handle, iSDbuffer, highPoint);
         if (data_written != highPoint) {
             ESP_LOGE(TAG_AVI, "Error writing buffer before adding new chunk! Wrote %zu/%zu", data_written, highPoint);
             // Handle error (e.g., stop recording)
             return;
         }
         // Reset buffer pointer
         highPoint = 0;
         ESP_LOGD(TAG_AVI, "Buffer flushed. New highPoint = 0. File pos = %ld", ftell(aviFile_handle));
    }

    // --- Add Chunk Header to Buffer ---
    memcpy(iSDbuffer + highPoint, dcBuf, 4);
    memcpy(iSDbuffer + highPoint + 4, &jpegChunkSize, 4);
    highPoint += CHUNK_HDR;

    // --- Add Chunk Data to Buffer ---
    // This part needs to handle data larger than remaining buffer space
    size_t remaining_data = fb->len; // Start with JPEG data length
    size_t data_offset = 0;         // Offset into fb->buf

    while (remaining_data > 0) {
        size_t space_in_buffer = RAMSIZE - highPoint;
        size_t bytes_to_copy = std::min(remaining_data, space_in_buffer);

        memcpy(iSDbuffer + highPoint, fb->buf + data_offset, bytes_to_copy);
        highPoint += bytes_to_copy;
        data_offset += bytes_to_copy;
        remaining_data -= bytes_to_copy;

        // If buffer is now full, write it to the file
        if (highPoint == RAMSIZE) {
             ESP_LOGD(TAG_AVI,"Buffer full while copying frame data. Writing %d bytes.", RAMSIZE);
             size_t data_written = STORAGE.write(aviFile_handle, iSDbuffer, RAMSIZE);
             if (data_written != RAMSIZE) {
                 ESP_LOGE(TAG_AVI, "Error writing full buffer mid-frame! Wrote %zu/%d", data_written, RAMSIZE);
                 // Handle error
                 return;
             }
             // Reset buffer pointer
             highPoint = 0;
             ESP_LOGD(TAG_AVI, "Buffer flushed mid-frame. New highPoint = 0. File pos = %ld", ftell(aviFile_handle));
        }
    }

    // --- Add Filler Bytes to Buffer ---
    remaining_data = filler; // Now handle filler
    while (remaining_data > 0) {
        size_t space_in_buffer = RAMSIZE - highPoint;
        size_t bytes_to_copy = std::min(remaining_data, space_in_buffer);

        memset(iSDbuffer + highPoint, 0, bytes_to_copy); // Add zero padding
        highPoint += bytes_to_copy;
        remaining_data -= bytes_to_copy;

        // If buffer is now full, write it to the file
        if (highPoint == RAMSIZE) {
             ESP_LOGD(TAG_AVI,"Buffer full while copying filler. Writing %d bytes.", RAMSIZE);
             size_t data_written = STORAGE.write(aviFile_handle, iSDbuffer, RAMSIZE);
             if (data_written != RAMSIZE) {
                 ESP_LOGE(TAG_AVI, "Error writing full buffer mid-filler! Wrote %zu/%d", data_written, RAMSIZE);
                 // Handle error
                 return;
             }
             // Reset buffer pointer
             highPoint = 0;
             ESP_LOGD(TAG_AVI, "Buffer flushed mid-filler. New highPoint = 0. File pos = %ld", ftell(aviFile_handle));
        }
    }

    // --- Update Index ---
    buildAviIdx(jpegChunkSize, true, false); // Index uses size *with* padding
    vidSize += total_chunk_size; // Accumulate total size written for this frame
    frameCnt++;
    ESP_LOGD(TAG_AVI, "Frame %u finished processing. Buffer highPoint=%zu", frameCnt, highPoint);
}


// static void saveFrame(camera_fb_t* fb) {

//     bool is_first_frame = (frameCnt == 0);
//     if (is_first_frame) {
//         ESP_LOGI(TAG_AVI, "*** Processing FIRST frame ***");
//         ESP_LOGI(TAG_AVI, "    highPoint before saveFrame: %zu", highPoint);
//     }
//      ESP_LOGD(TAG_AVI, "Frame %u: highPoint=%zu, fb->len=%zu", frameCnt + 1, highPoint, fb->len);
//     // --- End check ---

//     if (!iSDbuffer || !aviFile_handle) { /* ... error handling ... */ return; }

//     uint32_t writeStartTime = esp_timer_get_time();

//     uint16_t filler = (4 - (fb->len & 0x00000003)) & 0x00000003;
//     size_t jpegChunkSize = fb->len + filler;

//     if (is_first_frame) {
//         ESP_LOGI(TAG_AVI, "    Writing '00dc' header at buffer offset %zu", highPoint);
//         ESP_LOGI(TAG_AVI, "    jpegChunkSize: %zu (len=%zu, fill=%u)", jpegChunkSize, fb->len, filler);
//    }

//     memcpy(iSDbuffer+highPoint, dcBuf, 4);
//     memcpy(iSDbuffer+highPoint+4, &jpegChunkSize, 4);
//     highPoint += CHUNK_HDR;

//     if (is_first_frame) {
//         ESP_LOGI(TAG_AVI, "    Buffer content at offset %zu after chunk header:", highPoint - CHUNK_HDR);
//         // Print first ~16 bytes starting from where the chunk header was written
//         char hex_buf[64]; // Buffer for hex string
//         int print_len = highPoint > 16 ? 16 : highPoint; // Print up to 16 bytes
//          if (highPoint >= CHUNK_HDR) { // Ensure we can access the header written
//              print_len = (highPoint - (highPoint - CHUNK_HDR)) > 16 ? 16 : (highPoint - (highPoint - CHUNK_HDR) + 8); // Print header + maybe start of data
//              size_t start_offset = highPoint - CHUNK_HDR;
//              for(int i=0; i < print_len && (start_offset+i) < (RAMSIZE + CHUNK_HDR) * 2 ; ++i) { // Basic bounds check
//                  sprintf(hex_buf + i*3, "%02X ", iSDbuffer[start_offset+i]);
//              }
//               hex_buf[print_len*3] = '\0';
//               ESP_LOGI(TAG_AVI, "    Hex: %s", hex_buf);
//          }
//     }

//     if (highPoint >= RAMSIZE) {
//         highPoint -= RAMSIZE;
//         size_t data_written = STORAGE.write(aviFile_handle, iSDbuffer, RAMSIZE);
//         if (data_written != RAMSIZE) {
//             ESP_LOGE(TAG_AVI, "Error writing frame data (RAMSIZE block)! Wrote %zu bytes, expected %d", data_written, RAMSIZE);
//             // Handle error - maybe close file and stop recording
//             return; // Or break out of the function and handle error more gracefully
//         }
//         memcpy(iSDbuffer, iSDbuffer+RAMSIZE, highPoint);
//     }
//     size_t jpegRemain = jpegChunkSize;
//     uint32_t wTime = esp_timer_get_time() / 1000;
//     while (jpegRemain >= RAMSIZE - highPoint) {
//         memcpy(iSDbuffer+highPoint, fb->buf + jpegChunkSize - jpegRemain, RAMSIZE - highPoint);
//         size_t data_written_loop = STORAGE.write(aviFile_handle, iSDbuffer, RAMSIZE);
//         if (data_written_loop != RAMSIZE) {
//              ESP_LOGE(TAG_AVI, "Error writing frame data (loop block)! Wrote %zu bytes, expected %d", data_written_loop, RAMSIZE);
//             // Handle error
//             return; // Or break and handle error
//         }
//         jpegRemain -= RAMSIZE - highPoint;
//         highPoint = 0;
//     }
//     wTime = (esp_timer_get_time() / 1000) - wTime;
//     wTimeTot += wTime;
//     ESP_LOGD(TAG_AVI, "SD storage time %lu ms", wTime);
//     memcpy(iSDbuffer+highPoint, fb->buf + jpegChunkSize - jpegRemain, jpegRemain);
//     highPoint += jpegRemain;

//     buildAviIdx(jpegChunkSize, true, false);
//     vidSize += jpegChunkSize + CHUNK_HDR;
//     frameCnt++;
//     ESP_LOGD(TAG_AVI, "============================");
//     uint32_t writeEndTime = esp_timer_get_time();
//     wTimeTot += (writeEndTime - writeStartTime) / 1000;

//     ESP_LOGD(TAG_AVI, "Frame %u processed. Total frames: %u", frameCnt, frameCnt);
// }

static bool closeAvi() {
    if (!aviFile_handle) {
        ESP_LOGW(TAG_AVI, "closeAvi called but file not open.");
        return false;
    }

    uint32_t closeStartTime = esp_timer_get_time();
    uint32_t vidDuration = (closeStartTime / 1000) - startTime; // Duration in ms
    uint32_t vidDurationSecs = vidDuration / 1000;

    ESP_LOGI(TAG_AVI, "Closing AVI. Duration: %lu ms (%lu s), Frames: %u", vidDuration, vidDurationSecs, frameCnt);

    // Write any remaining data from the buffer
    if (highPoint > 0) {
        ESP_LOGI(TAG_AVI, "Writing final buffer data: %zu bytes", highPoint);
        size_t final_buffer_written = STORAGE.write(aviFile_handle, iSDbuffer, highPoint);
        if (final_buffer_written != highPoint) {
            ESP_LOGE(TAG_AVI, "Error writing final buffer! Wrote %zu, expected %zu", final_buffer_written, highPoint);
            // Continue closing, but log error
        }
        highPoint = 0; // Reset buffer mark
    }

    // Finalize and write the index chunk ('idx1')
    finalizeAviIndex(frameCnt, false);
    ESP_LOGI(TAG_AVI, "Writing AVI index (%u bytes)...", indexLen[0]);
    size_t indexBytesWritten = 0;
    size_t readLen_idx = 0;
    do {
        // Use iSDbuffer temporarily to write index blocks
        readLen_idx = writeAviIndex((uint8_t*)iSDbuffer, RAMSIZE, false);
        if (readLen_idx > 0) {
            size_t index_block_written = STORAGE.write(aviFile_handle, iSDbuffer, readLen_idx);
            if (index_block_written != readLen_idx) {
                ESP_LOGE(TAG_AVI, "Error writing index block! Wrote %zu, expected %zu", index_block_written, readLen_idx);
                // Abort? Continue?
            }
            indexBytesWritten += index_block_written;
        }
    } while (readLen_idx > 0);
    ESP_LOGI(TAG_AVI, "AVI index written (%zu bytes).", indexBytesWritten);

    // Calculate actual FPS
    float actualFPS = (vidDuration > 0) ? (1000.0f * (float)frameCnt) / ((float)vidDuration) : 0.0f;
    uint8_t actualFPSint = (uint8_t)(lround(actualFPS));
    if (actualFPSint == 0 && frameCnt > 0) actualFPSint = 1; // Avoid 0 FPS if frames exist

    // Update AVI header with final values (frame count, FPS, sizes)
    xSemaphoreTake(aviMutex, portMAX_DELAY); // Protect header generation if needed elsewhere
    buildAviHdr(actualFPSint, fsizePtr, frameCnt, false);
    xSemaphoreGive(aviMutex);

    // Seek to beginning and rewrite the header
    ESP_LOGI(TAG_AVI, "Seeking to file start to rewrite header...");
    if (!STORAGE.seek(aviFile_handle, 0, SEEK_SET)) { // SEEK_SET = 0
        ESP_LOGE(TAG_AVI, "Error seeking to beginning of file!");
        // Continue closing, but header won't be updated
    } else {
        ESP_LOGI(TAG_AVI, "Rewriting AVI header (%d bytes)...", AVI_HEADER_LEN);
        size_t header_rewrite_written = STORAGE.write(aviFile_handle, aviHeader, AVI_HEADER_LEN);
        if (header_rewrite_written != AVI_HEADER_LEN) {
            ESP_LOGE(TAG_AVI, "Error rewriting AVI header! Wrote %zu, expected %d", header_rewrite_written, AVI_HEADER_LEN);
        } else {
            ESP_LOGI(TAG_AVI, "AVI header rewritten successfully.");
        }
    }

    // Close the file handle
    ESP_LOGI(TAG_AVI, "Closing file handle.");
    STORAGE.close(aviFile_handle);
    aviFile_handle = NULL; // Mark as closed

    // Rename the temporary file if duration is sufficient
    bool renamed = false;
    if (vidDurationSecs >= minSeconds) {
        char dirpartName[12]; // Buffer for date directory YYYY-MM-DD
        dateFormat(dirpartName, sizeof(dirpartName), true);
        char dateDirPath[FILE_NAME_LEN];
        snprintf(dateDirPath, sizeof(dateDirPath), "%s/%s", MOUNT_POINT, dirpartName);

        char timestampPart[20]; // Buffer for time H-M-S
        dateFormat(timestampPart, sizeof(timestampPart), false); // Get full timestamp

        // Construct final filename: /sdcard/YYYY-MM-DD/YYYY-MM-DD_HH-MM-SS_FMT_FPS_DUR.avi
        // Extract only HH-MM-SS part from timestampPart if needed
        const char* timeOnly = strchr(timestampPart, '_');
        if (timeOnly) timeOnly++; else timeOnly = timestampPart; // Fallback if '_' not found

        // Get frame size string (e.g., "HD") - Requires mapping fsizePtr to string
        const char *fsizeStr = "UNK";
        if (fsizePtr == FRAMESIZE_SVGA) fsizeStr = "SVGA";
        else if (fsizePtr == FRAMESIZE_HD) fsizeStr = "HD";
        else if (fsizePtr == FRAMESIZE_UXGA) fsizeStr = "UXGA";
        // Add other mappings as needed

        snprintf(aviFileName, sizeof(aviFileName) - 1, "%s/%s_%s_%s_%u_%lus.avi",
                 dateDirPath, // Directory path
                 dirpartName, // Date part YYYY-MM-DD
                 timeOnly,    // Time part HH-MM-SS
                 fsizeStr,    // Frame size string
                 actualFPSint, // Actual FPS
                 vidDurationSecs); // Duration
        aviFileName[sizeof(aviFileName) - 1] = '\0'; // Ensure null termination

        ESP_LOGI(TAG_AVI, "Renaming %s to %s", AVITEMP, aviFileName);
        if (!STORAGE.rename(AVITEMP, aviFileName)) {
            ESP_LOGE(TAG_AVI, "Error renaming file from %s to %s", AVITEMP, aviFileName);
            // Handle rename error - maybe try removing temp file?
            STORAGE.remove(AVITEMP);
        } else {
            ESP_LOGI(TAG_AVI, "File renamed successfully to %s", aviFileName);
            renamed = true;
        }
    } else {
        ESP_LOGI(TAG_AVI, "Insufficient capture duration (%lu s < %u s). Removing temporary file: %s",
                 vidDurationSecs, minSeconds, AVITEMP);
        STORAGE.remove(AVITEMP);
    }

    uint32_t closeEndTime = esp_timer_get_time();
    cTime = (closeEndTime - closeStartTime) / 1000; // Total close time in ms

    // Print stats only if file was kept
    if (renamed) {
        ESP_LOGI(TAG_AVI, "******** AVI recording stats ********");
        ESP_LOGI(TAG_AVI, "Recorded %s", aviFileName);
        ESP_LOGI(TAG_AVI, "AVI duration: %lu secs", vidDurationSecs);
        ESP_LOGI(TAG_AVI, "Number of frames: %u", frameCnt);
        ESP_LOGI(TAG_AVI, "Required FPS: %u", FPS);
        ESP_LOGI(TAG_AVI, "Actual FPS: %0.1f", actualFPS);
        ESP_LOGI(TAG_AVI, "File size: %s", fmtSize(vidSize + AVI_HEADER_LEN + indexLen[0])); // Approx total size
        if (frameCnt > 0) {
            ESP_LOGI(TAG_AVI, "Average frame length (data+hdr): %lu bytes", (unsigned long)(vidSize / frameCnt));
            ESP_LOGI(TAG_AVI, "Average frame storage time: %lu ms", (unsigned long)(wTimeTot / frameCnt));
        }
        ESP_LOGI(TAG_AVI, "Average SD write speed (data only): %lu kB/s", (wTimeTot > 0) ? (unsigned long)(((vidSize / wTimeTot) * 1000) / 1024) : 0);
        ESP_LOGI(TAG_AVI, "File open / completion times: %lu ms / %lu ms", oTime, cTime);
        // ESP_LOGI(TAG_AVI, "Busy: %u%%", std::min((int)(100 * (wTimeTot + oTime + cTime) / vidDuration), 100)); // Rough estimate
        checkMemory();
        ESP_LOGI(TAG_AVI, "*************************************");
    }
    if (!checkFreeStorage()) doRecording = false; // Check space after recording

    return renamed; // Return true if file was successfully recorded and renamed
}


static bool processFrame(camera_fb_t* fb) {
    static bool wasCapturing = false;
    bool finishRecording = false;

    if (fb == NULL || fb->len == 0 || fb->len > MAX_JPEG) {
        ESP_LOGW(TAG_AVI, "Invalid frame: NULL or size out of bounds");
        return false;
    }

    if (fb->len < 4 || fb->buf[0] != 0xFF || fb->buf[1] != 0xD8) {
        ESP_LOGE(TAG_AVI, "Corrupted JPEG: NO SOI marker");
        return false; // Do NOT call esp_camera_fb_return here
    }

    isCapturing = forceRecord || doRecording;
    if (isCapturing && !wasCapturing) {
        ESP_LOGI(TAG_AVI, "Starting recording");
        openAvi();
        wasCapturing = true;
    }
    if (isCapturing) {
        saveFrame(fb);
        if (frameCnt >= maxFrames) {
            ESP_LOGI(TAG_AVI, "Auto closed recording after %u frames", maxFrames);
            forceRecord = false;
            isCapturing = false;
        }
    }
    if (!isCapturing && wasCapturing) {
        ESP_LOGI(TAG_AVI, "Stopping recording");
        closeAvi();
        finishRecording = true;
        wasCapturing = false;
        forceRecord = true;
    }

    return true;
}

static void captureTask(void* parameter) {
    forceRecord = true;
    const TickType_t EVENT_INTERVAL = pdMS_TO_TICKS(1000);  // e.g. 5 s between uploads
    TickType_t         last_event_tick = 0;

    
    while (true) {
        TickType_t now = xTaskGetTickCount();
        if(eState == PEER_CONNECTION_CONNECTED) {
            last_event_tick = xTaskGetTickCount();
        } else if (eState == PEER_CONNECTION_DISCONNECTED) {
            last_event_tick = 0;
        }
        camera_fb_t* fb = NULL;
        ESP_LOGI(TAG_AVI, "captureTask waiting for frame");
        
        if((eState != PEER_CONNECTION_CHECKING)|| (eState == PEER_CONNECTION_CONNECTED && ((now - last_event_tick) >= EVENT_INTERVAL))) {   
            if (xQueueReceive(recordingQueue, &fb, portMAX_DELAY) == pdTRUE) {
             
                    ESP_LOGI(TAG_AVI, "Frame received in captureTask");
                    if (fb != NULL) {
                        ESP_LOGI(TAG_AVI, "Received frame, len: %u", fb->len);
                        if (processFrame(fb)) {
                            ESP_LOGI(TAG_AVI, "Frame processed successfully");
                        } else {
                            ESP_LOGW(TAG_AVI, "Failed to process frame");
                        }
                        heap_caps_free(fb->buf); // Free only here
                        heap_caps_free(fb);      // Free only here
                    } else {
                        ESP_LOGW(TAG_AVI, "Received NULL frame from queue");
                    }
                   // xSemaphoreGive(xSemaphore);
                } else {
                    ESP_LOGW(TAG_AVI, "Failed to take readSemaphore in captureTask");
                }
                
            
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Avoid busy waiting
    }
    vTaskDelete(NULL);
}

uint8_t setFPS(uint8_t val) {
    if (val) {
        FPS = val;
        controlFrameTimer(true);
    }
    return FPS;
}

uint8_t setFPSlookup(uint8_t val) {
    fsizePtr = val;
    return setFPS(10); // Simplified FPS setting
}

static void startSDtasks() {
    BaseType_t captureTaskCreated = xTaskCreatePinnedToCore(&captureTask, "captureTask", CAPTURE_STACK_SIZE, NULL, CAPTURE_PRI, &captureHandle, 0);
    if (captureTaskCreated != pdTRUE) {
        ESP_LOGE(TAG_AVI, "Failed to create captureTask");
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, (framesize_t)fsizePtr);
    } else {
        ESP_LOGE(TAG_AVI, "Failed to get camera sensor");
    }

    ESP_LOGI(TAG_AVI, "Started SD tasks");

    setFPS(FPS);
    debugMemory("startSDtasks");


}

bool prepRecording() {
    // readSemaphore = xSemaphoreCreateBinary(); // Semaphore for recorder task? Check usage. If unused, remove.
    aviMutex = xSemaphoreCreateMutex(); // For accessing shared AVI header/index data during recording
    if (aviMutex == NULL) {
        ESP_LOGE(TAG_AVI, "Failed to create aviMutex!");
        return false;
    }

    // Initialize Playback Semaphore
    
    // Allocate buffer used for *recording*
    if (iSDbuffer == NULL) { // Allocate only if not already allocated
        iSDbuffer = (uint8_t*)heap_caps_malloc((RAMSIZE + CHUNK_HDR) * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (iSDbuffer == NULL) {
            ESP_LOGE(TAG_AVI, "Failed to allocate iSDbuffer for recording in PSRAM!");
            vSemaphoreDelete(aviMutex); // Clean up
            aviMutex = NULL;
            vSemaphoreDelete(playbackControlSemaphore);
            playbackControlSemaphore = NULL;
            return false; // Indicate failure
        }
        ESP_LOGI(TAG_AVI, "Recording iSDbuffer allocated in PSRAM");
    } else {
         ESP_LOGW(TAG_AVI, "Recording iSDbuffer already allocated.");
    }


    startSDtasks(); // Creates captureTask (which handles recording)
    // Note: Playback task is created separately in app_main.c

    ESP_LOGI(TAG_AVI, "Camera model %s ready @ %uMHz", camModel, xclkMhz);
    debugMemory("prepRecording");
    return true;
}




// Placeholder functions Implementation (minimal)
void dateFormat(char* buffer, size_t bufSize, bool dateOnly) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    if (dateOnly) {
        strftime(buffer, bufSize, "%Y-%m-%d", &timeinfo);
    } else {
        strftime(buffer, bufSize, "%Y-%m-%d_%H-%M-%S", &timeinfo);
    }
}

char* fmtSize(size_t bytes) {
    static char buffer[20];
    if (bytes < 1024) {
        snprintf(buffer, sizeof(buffer), "%zu B", bytes);
    } else if (bytes < 1024 * 1024) {
        snprintf(buffer, sizeof(buffer), "%.1f KB", (float)bytes / 1024.0f);
    } else {
        snprintf(buffer, sizeof(buffer), "%.1f MB", (float)bytes / (1024.0f * 1024.0f));
    }
    return buffer;
}


bool checkFreeStorage() {
    return true; // Placeholder
}

void checkMemory() {
    // Placeholder
}
void debugMemory(const char* tag) {
    ESP_LOGD(TAG_AVI, "debugMemory: %s", tag);
}

// --- Implement STORAGE structure functions --- (Same as before, just moved to the end for better readability)
// Implement STORAGE functions using standard C file operations
bool STORAGE_exists(const char* path) {
    struct stat st;
    return (stat(path, &st) == 0);
}

FILE* STORAGE_open(const char* path, const char* mode) {
    return fopen(path, mode);
}

size_t STORAGE_read(FILE* fp, uint8_t* clientBuf, size_t buffSize) {
    return fread(clientBuf, 1, buffSize, fp);
}

size_t STORAGE_write(FILE* fp, const void* clientBuf, size_t buffSize) {
    return fwrite(clientBuf, 1, buffSize, fp);
}

bool STORAGE_seek(FILE* fp, long offset, int origin) {
    return fseek(fp, offset, origin) == 0;
}

size_t STORAGE_size(FILE* fp) {
    long pos = ftell(fp);
    fseek(fp, 0, SEEK_END);
    size_t size = ftell(fp);
    fseek(fp, pos, SEEK_SET);
    return size;
}

void STORAGE_close(FILE* fp) {
    fclose(fp);
}

bool STORAGE_remove(const char* path) {
    return remove(path) == 0;
}

bool STORAGE_rename(const char* oldpath, const char* newpath) {
    return rename(oldpath, newpath) == 0;
}

bool STORAGE_mkdir(const char* path) {
    return mkdir(path, 0777) == 0;
}

// Assign function pointers



void start_playback(const char *filename) {
    if (playback_active) {
        ESP_LOGW(TAG_AVI, "Playback already active. Stop current playback first.");
        // stop_playback(); // Consider adding stop here if desired behavior
        // vTaskDelay(pdMS_TO_TICKS(100));
         return; // Don't allow starting if already active
    }

    // Check if recording is active - potentially prevent playback?
    // if (isCapturing) { ESP_LOGW(TAG_AVI,"Cannot start playback while recording is active."); return; }


    if (filename && strlen(filename) > 0 && strlen(filename) < sizeof(current_playback_file)) {
        strncpy(current_playback_file, filename, sizeof(current_playback_file) - 1);
        current_playback_file[sizeof(current_playback_file) - 1] = '\0';
        ESP_LOGI(TAG_AVI, "Requesting playback start specifically for: %s", current_playback_file);
    } else {
        current_playback_file[0] = '\0'; // Clear specific file request
        ESP_LOGI(TAG_AVI, "Requesting playback start from the beginning.");
    }

    stop_playback_request = false;
    playback_active = true; // Set flag before signaling

    // Ensure playback task exists
    if(playbackTaskHandle == NULL) {
        ESP_LOGI(TAG_AVI,"Playback task not running, creating it.");
        BaseType_t taskCreated = xTaskCreatePinnedToCore(
                                        playback_task,
                                        "playbackTask",
                                        6 * 1024, // Increased stack size for C++ strings/vector etc.
                                        NULL,
                                        5, // Priority
                                        &playbackTaskHandle,
                                        1); // Core 1
        if(taskCreated != pdPASS) {
             ESP_LOGE(TAG_AVI,"Failed to create playback task!");
             playback_active = false; // Reset flag
             return;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay for task setup
    }

    if (playbackControlSemaphore != NULL) {
        xSemaphoreGive(playbackControlSemaphore); // Signal the task to start/continue
    } else {
        ESP_LOGE(TAG_AVI, "Playback control semaphore not initialized! Cannot start.");
        playback_active = false; // Reset flag on error
    }
}
void stop_playback() {
    if (playback_active) {
        ESP_LOGI(TAG_AVI, "Requesting playback stop.");
        stop_playback_request = true;
        if (playbackControlSemaphore != NULL) {
             xSemaphoreGive(playbackControlSemaphore); // Wake task if waiting
        }
        // Note: playback_active flag is cleared by the playback task itself when it stops
    } else {
        ESP_LOGI(TAG_AVI, "Stop playback requested, but not active.");
    }
}

// --- Helper Function to Get Sorted AVI Files ---
bool get_sorted_avi_files(const char *dir_path_to_scan, const char *requested_start_file, std::vector<std::string> &files, int current_index_ref) {
    files.clear();
    current_index_ref = -1; // Initialize index before search

    ESP_LOGI(TAG_AVI, "Scanning directory: %s for AVI files", dir_path_to_scan);
    DIR *dir = opendir(dir_path_to_scan);
    if (!dir) {
        ESP_LOGE(TAG_AVI, "Failed to open directory: %s", dir_path_to_scan);
        return false;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        std::string entry_name = entry->d_name;
        std::string full_path = std::string(dir_path_to_scan) + "/" + entry_name;

        // Skip "." and ".." entries
        if (entry_name == "." || entry_name == "..") {
            continue;
        }

        struct stat st;
        if (stat(full_path.c_str(), &st) == 0) {
            if (S_ISREG(st.st_mode)) { // It's a file
                const char *dot = strrchr(entry->d_name, '.');
                if (dot && strcasecmp(dot, ".avi") == 0) { // Case-insensitive check
                    if (full_path != AVITEMP) { // Skip temporary file
                        files.push_back(full_path);
                        ESP_LOGD(TAG_AVI, "Found AVI file: %s", full_path.c_str());
                    } else {
                        ESP_LOGD(TAG_AVI, "Skipping temporary file: %s", full_path.c_str());
                    }
                }
            } else if (S_ISDIR(st.st_mode)) {
                 // --- Recursive Call to Scan Subdirectories ---
                 ESP_LOGD(TAG_AVI,"Entering subdirectory: %s", full_path.c_str());
                 // Note: current_index_ref passed here is not used by the recursive call result directly
                 get_sorted_avi_files(full_path.c_str(), requested_start_file, files, current_index_ref);
                 // The files vector is appended by the recursive call.
                 // The current_index_ref might get overwritten, reset it after recursion if needed.
                 current_index_ref = -1; // Reset index after recursion returns (will be found later)
            }
        } else {
             ESP_LOGW(TAG_AVI,"Failed to get stat for: %s", full_path.c_str());
        }
    }
    closedir(dir);

    // Only sort and proceed if we are at the top-level call (or handle sorting differently)
    // For simplicity, let's sort the entire accumulated list here.
    if (files.empty()) {
        ESP_LOGW(TAG_AVI, "No AVI files found in %s or subdirectories (excluding temp)", dir_path_to_scan);
        return false;
    }

    std::sort(files.begin(), files.end());
    ESP_LOGI(TAG_AVI, "Found and sorted %d total AVI files.", files.size());
    for(size_t i = 0; i < files.size(); ++i) {
        ESP_LOGD(TAG_AVI, "Sorted List [%zu]: %s", i, files[i].c_str());
    }


    // --- Find Index Logic (after collecting all files) ---
    ESP_LOGI(TAG_AVI, "Searching for starting file: %s", requested_start_file ? requested_start_file : "NULL (start from beginning)");
    if (requested_start_file && strlen(requested_start_file) > 0) {
        for (size_t i = 0; i < files.size(); ++i) {
            if (files[i] == requested_start_file) {
                current_index_ref = i; // Found it!
                ESP_LOGI(TAG_AVI, "Requested starting file found at index: %d", current_index_ref);
                break;
            }
        }
    }

    // If not found or not requested, default to index 0
    if (current_index_ref == -1) {
        if (requested_start_file && strlen(requested_start_file) > 0) {
            ESP_LOGW(TAG_AVI, "Requested start file '%s' not found in collected list. Starting from index 0.", requested_start_file);
        } else {
            ESP_LOGI(TAG_AVI, "No specific start file requested or file not found. Starting from index 0.");
        }
        if (!files.empty()) {
            current_index_ref = 0;
            // Update the global current_playback_file to reflect the actual start
            strncpy(current_playback_file, files[0].c_str(), sizeof(current_playback_file) - 1);
            current_playback_file[sizeof(current_playback_file) - 1] = '\0';
            ESP_LOGI(TAG_AVI, "Set current_playback_file to: %s", current_playback_file);
        } else {
             ESP_LOGE(TAG_AVI,"Cannot set index 0, file list is empty!");
             return false; // Cannot proceed
        }
    }

    // Final validation
    if (current_index_ref < 0 || current_index_ref >= (int)files.size()) {
        ESP_LOGE(TAG_AVI, "Final calculated index %d is invalid for file list size %d", current_index_ref, files.size());
        return false;
    }

    return true;
}


// --- Playback Task Implementation ---
static bool find_first_avi_recursive(const char* dir_path, std::string& first_file_found) {
    bool found_any = false;
    ESP_LOGD(TAG_AVI, "find_first: Scanning %s", dir_path);
    DIR *dir = opendir(dir_path);
    if (!dir) {
        ESP_LOGE(TAG_AVI, "find_first: Failed to open directory: %s", dir_path);
        return false;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        std::string entry_name = entry->d_name;
        // Construct full path safely, avoid double slashes if dir_path is "/"
        std::string full_path;
        if (strcmp(dir_path, "/") == 0) {
            full_path = std::string("/") + entry_name;
        } else {
            full_path = std::string(dir_path) + "/" + entry_name;
        }


        if (entry_name == "." || entry_name == "..") continue;

        struct stat st;
        // Add small delay to yield CPU during intensive scanning
        // vTaskDelay(pdMS_TO_TICKS(1)); // Uncomment if needed, might slow down scan significantly

        if (stat(full_path.c_str(), &st) == 0) {
            if (S_ISREG(st.st_mode)) {
                const char *dot = strrchr(entry->d_name, '.');
                if (dot && strcasecmp(dot, ".avi") == 0 && full_path != AVITEMP) {
                    // Check if this file is the first found OR lexicographically smaller than the current best
                    if (first_file_found.empty() || full_path < first_file_found) {
                        ESP_LOGD(TAG_AVI, "find_first: Found candidate: %s", full_path.c_str());
                        first_file_found = full_path;
                        found_any = true;
                    }
                }
            } else if (S_ISDIR(st.st_mode)) {
                // Recurse - update first_file_found if a smaller one is found in subdirectory
                if (find_first_avi_recursive(full_path.c_str(), first_file_found)) {
                    found_any = true; // Found something in the subdirectory tree
                }
            }
        } else {
            ESP_LOGW(TAG_AVI, "find_first: Failed to get stat for %s (errno: %d)", full_path.c_str(), errno);
            // Continue scanning other files
        }
    }
    closedir(dir);
    return found_any;
}

// --- Helper Function to Find the NEXT AVI File ---
// Searches dir_path recursively for the smallest AVI file path that is lexicographically *greater than* current_file.
// Updates next_file_found if such a file is discovered and is better than the current next_file_found.
static bool find_next_avi_recursive(const char* dir_path, const std::string& current_file, std::string& next_file_found) {
    bool found_candidate_in_this_level_or_below = false; // Found *any* valid candidate in this path or subdirs?
    ESP_LOGD(TAG_AVI, "find_next: Scanning %s (current: %s, current_next: %s)",
             dir_path, current_file.c_str(), next_file_found.empty() ? "<none>" : next_file_found.c_str());
    DIR *dir = opendir(dir_path);
    if (!dir) {
        ESP_LOGE(TAG_AVI, "find_next: Failed to open directory: %s", dir_path);
        return false;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        std::string entry_name = entry->d_name;
        std::string full_path;
        if (strcmp(dir_path, "/") == 0) {
            full_path = std::string("/") + entry_name;
        } else {
            full_path = std::string(dir_path) + "/" + entry_name;
        }

        if (entry_name == "." || entry_name == "..") continue;

        struct stat st;
        // vTaskDelay(pdMS_TO_TICKS(1)); // Optional delay

        if (stat(full_path.c_str(), &st) == 0) {
            if (S_ISREG(st.st_mode)) {
                const char *dot = strrchr(entry->d_name, '.');
                if (dot && strcasecmp(dot, ".avi") == 0 && full_path != AVITEMP) {
                    // Condition: Found file must be > current_file
                    // AND it must be better than the current best 'next_file_found'
                    // (i.e., < next_file_found OR next_file_found is still empty)
                    if (full_path > current_file && (next_file_found.empty() || full_path < next_file_found)) {
                         ESP_LOGD(TAG_AVI, "find_next: Found new candidate: %s", full_path.c_str());
                         next_file_found = full_path; // Update the best candidate found so far
                         found_candidate_in_this_level_or_below = true;
                    }
                }
            } else if (S_ISDIR(st.st_mode)) {
                // Recurse - next_file_found is passed by reference and updated if a better candidate exists in the subtree
                 if (find_next_avi_recursive(full_path.c_str(), current_file, next_file_found)) {
                    // If the recursive call found/updated the candidate, mark it for the return value
                    found_candidate_in_this_level_or_below = true;
                 }
            }
        } else {
             ESP_LOGW(TAG_AVI, "find_next: Failed to get stat for %s (errno: %d)", full_path.c_str(), errno);
        }
    }
    closedir(dir);
    // Return true if 'next_file_found' was potentially updated (or already held a valid candidate)
    // during the scan of this directory level or its subdirectories.
    return found_candidate_in_this_level_or_below || !next_file_found.empty();
}


// --- Playback Task Implementation (Modified to Find 'movi') ---
void playback_task(void *pvParameters) {
    ESP_LOGI(TAG_AVI, "Playback Task Started (Header Read, Iterative Find) on Core %d", xPortGetCoreID());

    // Use a smaller buffer just for reading headers/chunks initially. Frame data read directly later.
    uint8_t *temp_buffer = (uint8_t *)heap_caps_malloc(1024, MALLOC_CAP_DEFAULT); // Smaller buffer for headers
    if (!temp_buffer) {
        ESP_LOGE(TAG_AVI, "Failed alloc temp_buffer!"); playbackTaskHandle = NULL; vTaskDelete(NULL); return;
    }

    std::string file_to_play_str; // Current file being processed
    bool first_file_in_sequence = true; // Is this the first file after a start signal?

    while (1) {
        ESP_LOGD(TAG_AVI, "Playback task waiting for signal...");
        if (xSemaphoreTake(playbackControlSemaphore, portMAX_DELAY) == pdTRUE) {
            if (stop_playback_request || !playback_active) {
                ESP_LOGI(TAG_AVI, "Playback task stopping (stop_req=%d, active=%d).", stop_playback_request, playback_active);
                playback_active = false; stop_playback_request = false;
                file_to_play_str.clear(); // Clear current file on stop
                first_file_in_sequence = true; // Reset for next start
                continue;
            }

            ESP_LOGI(TAG_AVI, "Playback task received start signal.");

            // --- File Sequencing Loop (Iterative) ---
            while (playback_active && !stop_playback_request) {

                // --- Find the file to play for this iteration ---
                if (first_file_in_sequence) {
                    // Logic to find the starting file (first available or specific requested)
                    first_file_in_sequence = false;
                    bool file_found;
                    file_to_play_str.clear();

                    if (current_playback_file[0] != '\0') {
                         ESP_LOGI(TAG_AVI,"Attempting to start playback from requested file: %s", current_playback_file);
                         struct stat st;
                         if (stat(current_playback_file, &st) == 0 && S_ISREG(st.st_mode)) {
                             const char *dot = strrchr(current_playback_file, '.');
                              if (dot && strcasecmp(dot, ".avi") == 0 && strcmp(current_playback_file, AVITEMP) != 0) {
                                 file_to_play_str = current_playback_file;
                                 file_found = true;
                              } else {
                                  ESP_LOGW(TAG_AVI, "Requested start file '%s' is not a valid AVI file. Finding first.", current_playback_file);
                                  file_found = find_first_avi_recursive(MOUNT_POINT, file_to_play_str);
                              }
                         } else {
                              ESP_LOGW(TAG_AVI, "Requested start file '%s' not found or not a file (errno: %d). Finding first.", current_playback_file, errno);
                              file_found = find_first_avi_recursive(MOUNT_POINT, file_to_play_str);
                         }
                     } else {
                         ESP_LOGI(TAG_AVI, "No specific start file, finding first AVI file...");
                         file_found = find_first_avi_recursive(MOUNT_POINT, file_to_play_str);
                     }

                     if (!file_found || file_to_play_str.empty()) {
                         ESP_LOGE(TAG_AVI, "Could not find any AVI file to start playback. Stopping sequence.");
                         playback_active = false; stop_playback_request = false;
                         first_file_in_sequence = true;
                         break; // Exit file sequencing loop
                     }
                     // Update global current_playback_file if search was used or requested file was invalid
                     if (strcmp(current_playback_file, file_to_play_str.c_str()) != 0) {
                          strncpy(current_playback_file, file_to_play_str.c_str(), sizeof(current_playback_file) - 1);
                          current_playback_file[sizeof(current_playback_file) - 1] = '\0';
                     }
                     ESP_LOGI(TAG_AVI, "Actual playback starting with file: %s", current_playback_file);

                } else {
                    // Find the *next* file after the one just played (file_to_play_str)
                    std::string next_file = "";
                    ESP_LOGI(TAG_AVI, "Finding next AVI file after: %s", file_to_play_str.c_str());
                    if (find_next_avi_recursive(MOUNT_POINT, file_to_play_str, next_file) && !next_file.empty()) {
                        file_to_play_str = next_file;
                        strncpy(current_playback_file, file_to_play_str.c_str(), sizeof(current_playback_file) - 1);
                        current_playback_file[sizeof(current_playback_file) - 1] = '\0';
                        ESP_LOGI(TAG_AVI, "Found next file: %s", file_to_play_str.c_str());
                    } else {
                        ESP_LOGI(TAG_AVI, "No subsequent AVI file found. Ending playback sequence.");
                        playback_active = false; stop_playback_request = false;
                        first_file_in_sequence = true;
                        break; // Exit file sequencing loop
                    }
                }

                // --- Current File Playback Logic ---
                ESP_LOGI(TAG_AVI, "Attempting to play: %s", file_to_play_str.c_str());
                FILE *pf = fopen(file_to_play_str.c_str(), "rb");
                if (!pf) {
                    ESP_LOGE(TAG_AVI, "Failed to open playback file: %s (errno: %d). Skipping.", file_to_play_str.c_str(), errno);
                    // Loop continues to find the *next* file after this one
                    continue;
                }
                ESP_LOGI(TAG_AVI, "Successfully opened: %s", file_to_play_str.c_str());
                uint32_t frame_count_in_file = 0; // Reset frame counter for this file

                // --- Read Header & Extract FPS ---
                uint32_t recorded_fps = 10; // Default FPS
                uint32_t frame_delay_ms = 100; // Default delay
                size_t header_read = fread(temp_buffer, 1, AVI_HEADER_LEN, pf);
                if (header_read >= 0x84 + 1) { // Need at least offset 0x84 + 1 byte
                    recorded_fps = temp_buffer[0x84]; // Read FPS byte from strh chunk
                    if (recorded_fps == 0 || recorded_fps > 60) { // Sanity check
                        ESP_LOGW(TAG_AVI, "Invalid FPS %u read from header, using default 10", recorded_fps);
                        recorded_fps = 10;
                    }
                    frame_delay_ms = (recorded_fps > 0) ? (1000 / recorded_fps) : 100;
                    ESP_LOGI(TAG_AVI, "FPS from header: %u, Frame delay: %lu ms", recorded_fps, frame_delay_ms);
                } else {
                    ESP_LOGW(TAG_AVI, "Could not read enough header bytes (%zu) to determine FPS. Using default %u.", header_read, recorded_fps);
                }

                // --- Locate 'movi' chunk ---
                bool movi_found = false;
                long movi_start_offset = -1;
                uint32_t chunk_id = 0;
                uint32_t chunk_size = 0;
                uint32_t list_type = 0;

                // Start searching after the RIFF 'AVI ' header (offset 12)
                if (fseek(pf, 12, SEEK_SET) != 0) {
                    ESP_LOGE(TAG_AVI, "Failed to seek past RIFF header in %s", file_to_play_str.c_str());
                    fclose(pf); continue; // Try next file
                }
                ESP_LOGD(TAG_AVI,"Searching for 'movi' LIST chunk starting at offset 12...");
                long current_pos = 12;

                while (current_pos < STORAGE_size(pf)) { // Avoid reading past EOF
                     ESP_LOGD(TAG_AVI, "Current file position for chunk search: %ld", current_pos);
                     if (fseek(pf, current_pos, SEEK_SET) != 0) {
                         ESP_LOGE(TAG_AVI, "Seek error during movi search to pos %ld", current_pos);
                         break;
                     }

                     if (fread(&chunk_id, 1, 4, pf) != 4) { ESP_LOGE(TAG_AVI,"Failed read chunk ID at %ld", current_pos); break; }
                     if (fread(&chunk_size, 1, 4, pf) != 4) { ESP_LOGE(TAG_AVI,"Failed read chunk size at %ld", current_pos+4); break; }
                     current_pos += 8; // Advance past ID and size

                    // AVI uses little-endian, ESP32 is little-endian. Direct comparison *should* work if constants are defined correctly.
                    // Let's define constants as little-endian numerical values.
                    #define CHUNK_ID_LIST 0x5453494C // 'LIST' in little-endian
                    #define CHUNK_ID_MOVI 0x69766F6D // 'movi' in little-endian
                    #define CHUNK_ID_HDRL 0x6C726468 // 'hdrl' in little-endian
                    #define CHUNK_ID_00DC 0x63643030 // '00dc' in little-endian

                    ESP_LOGD(TAG_AVI, "Read Chunk: ID=0x%08lX, Size=%lu at offset %ld", chunk_id, chunk_size, current_pos - 8);

                    if (chunk_id == CHUNK_ID_LIST) { // "LIST"
                        if (fread(&list_type, 1, 4, pf) != 4) { ESP_LOGE(TAG_AVI,"Failed read LIST type at %ld", current_pos); break; }
                        current_pos += 4; // Advance past list type

                        ESP_LOGD(TAG_AVI, "  List Type: 0x%08lX", list_type);
                        if (list_type == CHUNK_ID_MOVI) { // "movi"
                            ESP_LOGI(TAG_AVI, "'movi' LIST chunk found! Header size: %lu. Data starts at offset %ld", chunk_size, current_pos);
                            movi_found = true;
                            movi_start_offset = current_pos; // This is the offset *after* 'LIST', size, and 'movi' ID
                            break; // Exit search loop
                        } else {
                            // It's some other LIST (like 'hdrl'). Skip its content.
                            // The size (chunk_size) includes the 4 bytes for the list_type we just read.
                            ESP_LOGD(TAG_AVI, "Skipping content of LIST type 0x%08lX (size %lu bytes)", list_type, chunk_size - 4);
                            current_pos += (chunk_size - 4); // Move pointer to the end of this LIST's data
                        }
                    } else {
                        // It's a regular chunk (not a LIST). Skip its content.
                        ESP_LOGD(TAG_AVI, "Skipping content of Chunk ID 0x%08lX (size %lu bytes)", chunk_id, chunk_size);
                        current_pos += chunk_size; // Move pointer to the end of this chunk's data
                    }

                     // Word align the position for the next chunk read
                     if (current_pos % 2 != 0) {
                         ESP_LOGD(TAG_AVI, "Adjusting position by 1 byte for word alignment from %ld", current_pos);
                         current_pos++;
                     }
                } // End while searching for movi

                if (!movi_found || movi_start_offset < 0) {
                    ESP_LOGE(TAG_AVI, "'movi' chunk not found or error occurred in %s. Skipping file.", file_to_play_str.c_str());
                    fclose(pf);
                    continue; // Try next file
                }

                // --- Frame Reading Loop ---
                ESP_LOGI(TAG_AVI,"Starting frame reading from file offset %ld", movi_start_offset);
                if(fseek(pf, movi_start_offset, SEEK_SET) != 0) {
                     ESP_LOGE(TAG_AVI, "Failed to seek to movi start offset %ld!", movi_start_offset);
                     fclose(pf); continue; // Try next file
                }
                current_pos = movi_start_offset; // Track position within movi data

                while (playback_active && !stop_playback_request) {
                    uint8_t frame_chunk_header[CHUNK_HDR];
                    long frame_header_offset = current_pos; // Record offset for logging

                    size_t bytes_read = fread(frame_chunk_header, 1, CHUNK_HDR, pf);
                    if (bytes_read == 0 && feof(pf)) {
                         ESP_LOGI(TAG_AVI, "EOF reached within movi data for %s (Offset: %ld)", file_to_play_str.c_str(), frame_header_offset);
                         break; // End of file normally
                     }
                    if (bytes_read < CHUNK_HDR) {
                         ESP_LOGE(TAG_AVI, "Read error reading frame chunk header at offset %ld in %s (read %zu bytes)", frame_header_offset, file_to_play_str.c_str(), bytes_read);
                         break; // Error reading header
                     }
                     current_pos += CHUNK_HDR;

                    uint32_t frame_chunk_id = 0;
                    uint32_t jpeg_size = 0;
                    memcpy(&frame_chunk_id, frame_chunk_header, 4);
                    memcpy(&jpeg_size, frame_chunk_header + 4, 4);

                    ESP_LOGD(TAG_AVI,"Read frame chunk: ID=0x%08lX, Size=%lu at offset %ld", frame_chunk_id, jpeg_size, frame_header_offset);

                    // Check if it's a video frame chunk ('00dc')
                    if (frame_chunk_id == CHUNK_ID_00DC) {
                        if (jpeg_size == 0 || jpeg_size > MAX_JPEG) {
                            ESP_LOGW(TAG_AVI, "Invalid JPEG size (%lu) in frame chunk at offset %ld in %s.", jpeg_size, frame_header_offset, file_to_play_str.c_str());
                            // Try to skip this chunk and continue? Risky. Let's break.
                            break;
                        }

                        // Allocate Frame Buffer (Use PSRAM for image data)
                        // Allocate dynamically each time to handle varying frame sizes
                        camera_fb_t *fb_out = (camera_fb_t*)heap_caps_malloc(sizeof(camera_fb_t), MALLOC_CAP_DEFAULT);
                        if (!fb_out) { ESP_LOGE(TAG_AVI, "Failed alloc fb_out! Delaying."); vTaskDelay(pdMS_TO_TICKS(50)); continue; } // Skip frame on alloc failure

                        fb_out->buf = (uint8_t *)heap_caps_malloc(jpeg_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                        if (!fb_out->buf) {
                             ESP_LOGE(TAG_AVI, "Failed alloc fb_out->buf (%lu)! Freeing fb_out, delaying.", jpeg_size);
                             heap_caps_free(fb_out);
                             vTaskDelay(pdMS_TO_TICKS(50));
                             continue; // Skip frame on alloc failure
                         }
                        fb_out->len = 0; // Initialize length

                        // Read JPEG Data
                        ESP_LOGD(TAG_AVI,"Reading %lu bytes of JPEG data from offset %ld", jpeg_size, current_pos);
                        size_t jpeg_bytes_read = fread(fb_out->buf, 1, jpeg_size, pf);
                        current_pos += jpeg_bytes_read; // Advance position by amount actually read

                        if (jpeg_bytes_read == jpeg_size) {
                            fb_out->len = jpeg_size; fb_out->width = 0; fb_out->height = 0; fb_out->format = PIXFORMAT_JPEG; fb_out->timestamp.tv_sec = 0; fb_out->timestamp.tv_usec = 0;
                            ESP_LOGD(TAG_AVI, "Queueing playback frame: %lu bytes (File frame %u)", fb_out->len, frame_count_in_file + 1);

                            

                            if (xQueueSend(streamingQueue, &fb_out, pdMS_TO_TICKS(200)) != pdTRUE) { // Increased timeout slightly
                                ESP_LOGW(TAG_AVI, "Streaming queue full. Dropping playback frame.");
                                // Free everything if queue fails
                                heap_caps_free(fb_out->buf);
                                heap_caps_free(fb_out);
                                
                               
                            } else {
                                // Frame successfully queued, increment counter
                                frame_count_in_file++;
                                // Delay for the correct frame rate AFTER successfully queueing
                                vTaskDelay(pdMS_TO_TICKS(frame_delay_ms));
                            }
                        } else {
                             ESP_LOGE(TAG_AVI, "Failed read JPEG data (%zu/%lu) from offset %ld in %s", jpeg_bytes_read, jpeg_size, current_pos - jpeg_bytes_read, file_to_play_str.c_str());
                             heap_caps_free(fb_out->buf); heap_caps_free(fb_out);
                             break; // Exit frame reading loop on error
                        }

                         // Word align the position *after* reading the chunk data
                         if (current_pos % 2 != 0) {
                             ESP_LOGD(TAG_AVI, "Seeking 1 byte for word alignment after frame data from %ld", current_pos);
                             if (fseek(pf, 1, SEEK_CUR) != 0) { // Use SEEK_CUR
                                  ESP_LOGE(TAG_AVI,"Failed seek alignment byte after frame!");
                                  break; // Exit loop on seek error
                             }
                             current_pos++; // Update our tracked position
                         }

                    } else {
                         // Found a chunk ID other than '00dc' inside 'movi'
                         // Could be audio ('01wb'), index ('ix00'), JUNK, etc.
                         // For simple video playback, we can try to skip it.
                         ESP_LOGW(TAG_AVI, "Unexpected chunk ID [0x%08lX] size %lu inside 'movi' at offset %ld in %s. Skipping.", frame_chunk_id, jpeg_size, frame_header_offset, file_to_play_str.c_str());
                         if (fseek(pf, jpeg_size, SEEK_CUR) != 0) { // Skip the data
                             ESP_LOGE(TAG_AVI, "Failed to seek past unexpected chunk data!");
                             break; // Exit loop on seek error
                         }
                         current_pos += jpeg_size; // Update position
                         // Word align after skipping
                         if (current_pos % 2 != 0) {
                              ESP_LOGD(TAG_AVI, "Seeking 1 byte for word alignment after skipping chunk from %ld", current_pos);
                              if (fseek(pf, 1, SEEK_CUR) != 0) { ESP_LOGE(TAG_AVI,"Failed seek alignment byte after skip!"); break; }
                              current_pos++;
                         }
                         // Continue to the next chunk within movi
                    }
                } // End frame reading loop (while playback_active)

                fclose(pf); // Close the current file
                ESP_LOGI(TAG_AVI, "Finished playing file %s (%u frames)", file_to_play_str.c_str(), frame_count_in_file);

                // Check if playback was stopped *during* file playback
                if (!playback_active || stop_playback_request) {
                    ESP_LOGI(TAG_AVI, "Playback stopped during file %s.", file_to_play_str.c_str());
                    playback_active = false; stop_playback_request = false;
                    first_file_in_sequence = true; // Need to find first/requested next time
                    break; // Exit file sequencing loop
                }
                // Otherwise, loop continues to find the next file

            } // End file sequencing loop (while playback_active)

            // --- Log Reason for Exiting File Sequencing Loop ---
            if (stop_playback_request) ESP_LOGI(TAG_AVI, "Playback sequence stopped by request.");
            if (!playback_active && !first_file_in_sequence) ESP_LOGI(TAG_AVI, "Playback sequence finished (no more files or stopped internally).");
            playback_active = false; stop_playback_request = false; // Ensure flags are clear
            first_file_in_sequence = true; // Reset for next start signal

        } // End if semaphore taken
    } // End main task loop (while 1)

    // --- Task Cleanup ---
    ESP_LOGI(TAG_AVI,"Playback Task Exiting.");
    if (temp_buffer) heap_caps_free(temp_buffer);
    playbackTaskHandle = NULL;
    vTaskDelete(NULL);
}