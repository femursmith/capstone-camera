#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

// Initialize WiFi connection or start AP mode for configuration.
void handle_wifi_connect(void);

// Load the camera ID from flash (NVS).
void load_camera_id(void);

// Save the camera ID to flash (NVS).
// The parameter camera_id is expected to be a null-terminated string.
void save_camera_id(const char *camera_id);

void load_user_id(void);

void save_user_id(const char *user_id);

extern char stored_camera_id[37]; 
extern char stored_user_id[22];
#ifdef __cplusplus
}
#endif

#endif // WIFI_MANAGER_H
