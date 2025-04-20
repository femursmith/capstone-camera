// recorder.h
#ifdef __cplusplus
extern "C" {
#endif
#include "esp_err.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h" // Added for SemaphoreHandle_t
#include <stdbool.h>      // Added for bool type
#include <stdint.h>       // Added for uint types
#include <stddef.h>       // Added for size_t

#define AVI_HEADER_LEN 310
#define MOUNT_POINT "/sdcard"

typedef struct {
    camera_fb_t *fb;       // Pointer to the camera frame buffer
    int ref_count;         // Reference count for the frame
    SemaphoreHandle_t mutex; // Mutex for thread-safe operations
} rc_camera_fb_t;

// Declare queues (already present but ensure extern)
extern QueueHandle_t recordingQueue; // Queue for recording frames
extern QueueHandle_t streamingQueue; // Queue for streaming frames
extern QueueHandle_t eventQueue;     // Queue for event frames (if used)
extern SemaphoreHandle_t xSemaphore; // Semaphore for WebRTC access


// --- Global Variables ---
extern bool forceRecord;
extern bool ready;
extern int maxFrames;
extern uint8_t FPS;
extern uint8_t fsizePtr;
extern uint8_t minSeconds;
extern bool doRecording;
extern uint8_t xclkMhz;
extern char camModel[10];
extern TaskHandle_t captureHandle;
extern TaskHandle_t playbackTaskHandle; // Add handle for playback task
extern volatile bool playback_active; // Flag to indicate playback state


// --- Function Declarations ---
esp_err_t recorder_init(); // Initialization function
// void recorder_task(void *parameter); // Recorder task function (integrated into captureTask now)
void prepAviIndex(bool isTL);
void buildAviHdr(uint8_t FPS, uint8_t frameType, uint16_t frameCnt, bool isTL);
void buildAviIdx(size_t dataSize, bool isVid, bool isTL);
size_t writeAviIndex(uint8_t* clientBuf, size_t buffSize, bool isTL);
void finalizeAviIndex(uint16_t frameCnt, bool isTL);
void controlFrameTimer(bool restartTimer);
// static void openAvi(); // Keep static if only used internally
// static void saveFrame(camera_fb_t* fb); // Keep static
// static bool closeAvi(); // Keep static
// static bool processFrame(); // Keep static (part of captureTask logic)
uint8_t setFPS(uint8_t val);
uint8_t setFPSlookup(uint8_t val);
// static void startSDtasks(); // Keep static
bool prepRecording();
// static void deleteTask(TaskHandle_t thisTaskHandle); // Keep static
void endTasks();
void dateFormat(char* buffer, size_t bufSize, bool dateOnly);
char* fmtSize(size_t bytes);
bool checkFreeStorage();
void checkMemory();
void debugMemory(const char* tag);

// --- Storage Abstraction (already declared, ensure prototypes match) ---
bool STORAGE_exists(const char* path);
FILE* STORAGE_open(const char* path, const char* mode);
size_t STORAGE_read(FILE* fp, uint8_t* clientBuf, size_t buffSize);
size_t STORAGE_write(FILE* fp, const void* clientBuf, size_t buffSize);
bool STORAGE_seek(FILE* fp, long offset, int origin);
size_t STORAGE_size(FILE* fp);
void STORAGE_close(FILE* fp);
bool STORAGE_remove(const char* path);
bool STORAGE_rename(const char* oldpath, const char* newpath);
bool STORAGE_mkdir(const char* path);

// --- Reference Counting (already declared) ---
void rc_decrement(rc_camera_fb_t *rc_fb); // Ensure this is correct

// --- Playback Functions ---
void start_playback(const char *filename); // Function to initiate playback
void stop_playback();                      // Function to stop playback
void playback_task(void *pvParameters);    // The playback task itself


#ifdef __cplusplus
}
#endif