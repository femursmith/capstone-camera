#define PIR_SENSOR_PIN          43
extern bool event_recieved;

void upload_image_init();
esp_err_t upload_image(camera_fb_t *fb);