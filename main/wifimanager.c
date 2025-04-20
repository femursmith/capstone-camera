#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "time.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "mdns.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "wifimanager.h"

// ---------- Definitions and Globals ----------

#define MAX_WIFI_ATTEMPTS 2
#define MAX_WIFI_CREDENTIALS 5

static const char *TAG = "wifi_manager";

// Data structure for WiFi credentials
typedef struct {
    char ssid[33];      // up to 32 characters + null
    char password[65];  // up to 64 characters + null
} wifi_credentials_t;

static esp_timer_handle_t green_led_timer;



#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE     115200
#define UART_TX_GPIO_NUM   1
#define UART_RX_GPIO_NUM   3
#define UART_BUF_SIZE      1024

// Global storage (simulate NVS storage for demonstration)
wifi_credentials_t stored_wifi[MAX_WIFI_CREDENTIALS];
int wifi_credentials_count = 0;
char stored_camera_id[37] = {0};
char stored_user_id[22] = {0};

// Event group to signal WiFi connection
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

wifi_credentials_t current_wifi = {0};

// ---------- Forward Declarations ----------
static void init_leds(void);
void handle_wifi_connect(void);
void start_mdns(void);
static void wifi_init_sta(void);
static void wifi_init_softap(void);
static httpd_handle_t start_webserver(void);
static void wifi_connect_task(void);
static void load_wifi_credentials(void);
static void save_wifi_credentials(const char *ssid, const char *password, const char *camera_secret);
void uart_request_task(void *pvParameters);
// HTTP Handlers
static esp_err_t root_get_handler(httpd_req_t *req);
static esp_err_t save_post_handler(httpd_req_t *req);



static void init_leds() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << 1) | (1ULL << 2), // GPIO1 (blue) and GPIO2 (green)
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // Initially turn off both LEDs
    gpio_set_level(1, 0); // Blue off
    gpio_set_level(2, 0); // Green off
    ESP_LOGI(TAG, "LEDs initialized on GPIO1 (blue) and GPIO2 (green)");
}

static void turn_off_green(void* arg) {
    gpio_set_level(2, 0); // Turn off green LED
    ESP_LOGI(TAG, "Green LED turned off after 10 seconds");
}
// Function to initialize UART2
static void uart_init_custom(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Install UART driver with RX/TX buffers.
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART initialized on TX:%d, RX:%d", UART_TX_GPIO_NUM, UART_RX_GPIO_NUM);
}

// Function to send the WiFi credentials via UART.
// The credentials are formatted as "SSID:<ssid>;PWD:<password>\n"
static void uart_send_credentials(const wifi_credentials_t *cred)
{
    char tx_buffer[128];
    snprintf(tx_buffer, sizeof(tx_buffer), "SSID:%s;PWD:%s\n", cred->ssid, cred->password);
    uart_write_bytes(UART_PORT_NUM, tx_buffer, strlen(tx_buffer));
    ESP_LOGI(TAG, "Sent via UART: %s", tx_buffer);
}



static void seed_random_generator(void)
{
    // Using time(NULL) might work if the time has been set,
    // otherwise you can use another source such as esp_timer_get_time().
    srand((unsigned int) time(NULL));
}



void clear_flash(void) {
    esp_err_t err = nvs_flash_erase();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase NVS flash: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "NVS flash erased successfully.");
    }

    err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reinitialize NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "NVS reinitialized successfully.");
    }
}


static void generate_random_camera_secret(char *dest, size_t len)
{
    const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    // Generate len-1 characters; last char will be null terminator.
    for (size_t i = 0; i < len - 1; i++) {
        int key = rand() % (sizeof(charset) - 1);
        dest[i] = charset[key];
    }
    dest[len - 1] = '\0';
}


// ---------- Simulated NVS Functions ----------

// -----------------------------------------
// Utility: URL-decode routine
// -----------------------------------------
static void urldecode(char *dst, const char *src) {
    char a, b;
    while (*src) {
        if ((*src == '%') &&
            ((a = src[1]) && (b = src[2])) &&
            (isxdigit(a) && isxdigit(b))) {
            if (a >= 'a')
                a -= 'a' - 'A';
            if (a >= 'A')
                a = a - 'A' + 10;
            else
                a -= '0';
            if (b >= 'a')
                b -= 'a' - 'A';
            if (b >= 'A')
                b = b - 'A' + 10;
            else
                b -= '0';
            *dst++ = (char)(16 * a + b);
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
}

// -----------------------------------------
// NVS Loading and Saving Functions
// -----------------------------------------

void load_camera_id(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS namespace 'storage' not found. No stored camera ID.");
        stored_camera_id[0] = '\0';
        return;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle for reading camera ID!", esp_err_to_name(err));
        stored_camera_id[0] = '\0';
        return;
    }

    size_t required_size = sizeof(stored_camera_id);
    err = nvs_get_str(nvs_handle, "cameraid", stored_camera_id, &required_size);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No camera ID found in NVS");
        stored_camera_id[0] = '\0';
    } else {
        ESP_LOGI(TAG, "Found camera ID: %s", stored_camera_id);
    }

    nvs_close(nvs_handle);
}

void save_camera_id(const char *camera_id)
{
    if (camera_id == NULL) {
        ESP_LOGW(TAG, "save_camera_id called with NULL pointer");
        return;
    }

    // Decode in case the camera_id is URL-encoded
    char decoded_camera[sizeof(stored_camera_id)];
    urldecode(decoded_camera, camera_id);

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle for writing camera ID!", esp_err_to_name(err));
        return;
    }

    // Copy to global variable
    strncpy(stored_camera_id, decoded_camera, sizeof(stored_camera_id) - 1);
    stored_camera_id[sizeof(stored_camera_id) - 1] = '\0';

    // Save to NVS
    err = nvs_set_str(nvs_handle, "cameraid", stored_camera_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error setting cameraid: %s", esp_err_to_name(err));
    }

    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing camera ID changes to NVS: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Saved camera ID: %s", stored_camera_id);
}

void load_user_id(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS namespace 'storage' not found. No stored user ID.");
        stored_user_id[0] = '\0';
        nvs_close(nvs_handle);
        return;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle for reading user ID!", esp_err_to_name(err));
        stored_user_id[0] = '\0';
        nvs_close(nvs_handle);
        return;
    }

    size_t required_size = sizeof(stored_user_id);
    err = nvs_get_str(nvs_handle, "userid", stored_user_id, &required_size);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No user ID found in NVS");
        stored_user_id[0] = '\0';
    } else {
        ESP_LOGI(TAG, "Found user ID: %s", stored_user_id);
    }

    nvs_close(nvs_handle);
}

void save_user_id(const char *user_id)
{
    if (user_id == NULL) {
        ESP_LOGW(TAG, "save_user_id called with NULL pointer");
        return;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle for writing user ID!", esp_err_to_name(err));
        return;
    }

    // Copy to global variable
    strncpy(stored_user_id, user_id, sizeof(stored_user_id) - 1);
    stored_user_id[sizeof(stored_user_id) - 1] = '\0';

    // Save to NVS
    err = nvs_set_str(nvs_handle, "userid", stored_user_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error setting userid: %s", esp_err_to_name(err));
    }

    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing user ID changes to NVS: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Saved user ID: %s", stored_user_id);
}


static void load_wifi_credentials(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS namespace 'storage' not found. No stored credentials.");
        wifi_credentials_count = 0;
        stored_camera_id[0] = '\0';
        return;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        wifi_credentials_count = 0;
        stored_camera_id[0] = '\0';
        return;
    }
    
    uint8_t count = 0;
    err = nvs_get_u8(nvs_handle, "wificount", &count);
    if (err == ESP_OK) {
        wifi_credentials_count = count;
        ESP_LOGI(TAG, "Found %d stored WiFi credentials", wifi_credentials_count);
    } else {
        ESP_LOGI(TAG, "No WiFi credentials found in NVS (wificount not set)");
        wifi_credentials_count = 0;
    }
    
    for (int i = 0; i < wifi_credentials_count && i < MAX_WIFI_CREDENTIALS; i++) {
        char key_ssid[16];
        char key_pwd[16];
        snprintf(key_ssid, sizeof(key_ssid), "ssid%d", i);
        snprintf(key_pwd, sizeof(key_pwd), "pwd%d", i);
        
        size_t required_size = sizeof(stored_wifi[i].ssid);
        err = nvs_get_str(nvs_handle, key_ssid, stored_wifi[i].ssid, &required_size);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "SSID for index %d not found", i);
            stored_wifi[i].ssid[0] = '\0';
        }
        
        required_size = sizeof(stored_wifi[i].password);
        err = nvs_get_str(nvs_handle, key_pwd, stored_wifi[i].password, &required_size);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Password for index %d not found", i);
            stored_wifi[i].password[0] = '\0';
        }
    }
    
    size_t required_size = sizeof(stored_camera_id);
    err = nvs_get_str(nvs_handle, "cameraid", stored_camera_id, &required_size);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No camera id found in NVS");
        stored_camera_id[0] = '\0';
    } else {
        ESP_LOGI(TAG, "Found camera id: %s", stored_camera_id);
    }
    
    nvs_close(nvs_handle);
}

static void save_wifi_credentials(const char *ssid, const char *password, const char *camera_secret) {
    // Decode incoming strings to handle special characters/spaces.
    char decoded_ssid[33];
    char decoded_password[65];
    char decoded_camera[33];
    urldecode(decoded_ssid, ssid);
    urldecode(decoded_password, password);
    if (camera_secret != NULL) {
        urldecode(decoded_camera, camera_secret);
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle for write", esp_err_to_name(err));
        return;
    }

    if (wifi_credentials_count < MAX_WIFI_CREDENTIALS) {
        strncpy(stored_wifi[wifi_credentials_count].ssid, decoded_ssid, sizeof(stored_wifi[wifi_credentials_count].ssid)-1);
        stored_wifi[wifi_credentials_count].ssid[sizeof(stored_wifi[wifi_credentials_count].ssid)-1] = '\0';
        strncpy(stored_wifi[wifi_credentials_count].password, decoded_password, sizeof(stored_wifi[wifi_credentials_count].password)-1);
        stored_wifi[wifi_credentials_count].password[sizeof(stored_wifi[wifi_credentials_count].password)-1] = '\0';
        wifi_credentials_count++;
    }

    err = nvs_set_u8(nvs_handle, "wificount", wifi_credentials_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error setting wificount: %s", esp_err_to_name(err));
    }

    for (int i = 0; i < wifi_credentials_count; i++) {
        char key_ssid[16];
        char key_pwd[16];
        snprintf(key_ssid, sizeof(key_ssid), "ssid%d", i);
        snprintf(key_pwd, sizeof(key_pwd), "pwd%d", i);
        
        err = nvs_set_str(nvs_handle, key_ssid, stored_wifi[i].ssid);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error setting %s: %s", key_ssid, esp_err_to_name(err));
        }
        err = nvs_set_str(nvs_handle, key_pwd, stored_wifi[i].password);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error setting %s: %s", key_pwd, esp_err_to_name(err));
        }
    }

    if ((stored_camera_id[0] == '\0') && (camera_secret != NULL)) {
        strncpy(stored_camera_id, decoded_camera, sizeof(stored_camera_id)-1);
        stored_camera_id[sizeof(stored_camera_id)-1] = '\0';
    }
    err = nvs_set_str(nvs_handle, "cameraid", stored_camera_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error setting cameraid: %s", esp_err_to_name(err));
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS changes: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Saved WiFi credentials. Count: %d, Camera ID: %s", wifi_credentials_count, stored_camera_id);
}

// -----------------------------------------
// Deletion: Remove a credential from memory and update NVS
// -----------------------------------------
static esp_err_t delete_wifi_credential(int index) {
    if (index < 0 || index >= wifi_credentials_count) {
        return ESP_ERR_INVALID_ARG;
    }
    // Shift remaining credentials down.
    for (int i = index; i < wifi_credentials_count - 1; i++) {
        stored_wifi[i] = stored_wifi[i + 1];
    }
    wifi_credentials_count--;

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS for deletion", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_set_u8(nvs_handle, "wificount", wifi_credentials_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error setting wificount: %s", esp_err_to_name(err));
    }
    // Rewrite all stored credentials.
    for (int i = 0; i < wifi_credentials_count; i++) {
        char key_ssid[16];
        char key_pwd[16];
        snprintf(key_ssid, sizeof(key_ssid), "ssid%d", i);
        snprintf(key_pwd, sizeof(key_pwd), "pwd%d", i);
        err = nvs_set_str(nvs_handle, key_ssid, stored_wifi[i].ssid);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error setting %s: %s", key_ssid, esp_err_to_name(err));
        }
        err = nvs_set_str(nvs_handle, key_pwd, stored_wifi[i].password);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error setting %s: %s", key_pwd, esp_err_to_name(err));
        }
    }
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing deletion to NVS: %s", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
    return ESP_OK;
}


// -----------------------------------------
// Web Server Handlers
// -----------------------------------------

static const char bootstrap_css[] =
"\n"
"body {\n"
"  margin: 0;\n"
"  padding: 0;\n"
"  font-family: Arial, sans-serif;\n"
"  background-color: #f8f9fa;\n"
"  color: #212529;\n"
"}\n"
"\n"
".container {\n"
"  max-width: 960px;\n"
"  margin: auto;\n"
"  padding: 20px;\n"
"  box-sizing: border-box;\n"
"}\n"
"\n"
".mt-4 {\n"
"  margin-top: 1.5rem !important;\n"
"}\n"
"\n"
".form-group {\n"
"  margin-bottom: 1rem;\n"
"}\n"
"\n"
"label {\n"
"  display: inline-block;\n"
"  margin-bottom: 0.5rem;\n"
"  font-weight: 600;\n"
"}\n"
"\n"
"input[type=\"text\"],\n"
"input[type=\"password\"] {\n"
"  display: block;\n"
"  width: 100%;\n"
"  padding: 0.375rem 0.75rem;\n"
"  line-height: 1.5;\n"
"  color: #495057;\n"
"  background-color: #fff;\n"
"  border: 1px solid #ced4da;\n"
"  border-radius: 0.25rem;\n"
"  box-sizing: border-box;\n"
"  margin-bottom: 0.5rem;\n"
"}\n"
"\n"
".btn {\n"
"  display: inline-block;\n"
"  font-weight: 400;\n"
"  color: #212529;\n"
"  text-align: center;\n"
"  white-space: nowrap;\n"
"  vertical-align: middle;\n"
"  user-select: none;\n"
"  background-color: transparent;\n"
"  border: 1px solid transparent;\n"
"  padding: 0.375rem 0.75rem;\n"
"  font-size: 1rem;\n"
"  line-height: 1.5;\n"
"  border-radius: 0.25rem;\n"
"  transition: color 0.15s, background-color 0.15s;\n"
"}\n"
"\n"
".btn-danger {\n"
"  color: #fff;\n"
"  background-color: #dc3545;\n"
"  border-color: #dc3545;\n"
"}\n"
"\n"
".btn-sm {\n"
"  padding: 0.25rem 0.5rem;\n"
"  font-size: 0.875rem;\n"
"  line-height: 1.5;\n"
"  border-radius: 0.2rem;\n"
"}\n"
"\n"
".table-responsive {\n"
"  width: 100%;\n"
"  overflow-x: auto;\n"
"}\n"
"\n"
".table {\n"
"  width: 100%;\n"
"  margin-bottom: 1rem;\n"
"  color: #212529;\n"
"  border-collapse: collapse;\n"
"}\n"
"\n"
".table th, .table td {\n"
"  padding: 0.75rem;\n"
"  vertical-align: top;\n"
"  border-top: 1px solid #dee2e6;\n"
"}\n"
"\n"
".table-striped tbody tr:nth-of-type(odd) {\n"
"  background-color: rgba(0, 0, 0, 0.05);\n"
"}\n"
"\n"
"@media (max-width: 768px) {\n"
"  .container {\n"
"    padding: 10px;\n"
"  }\n"
"  body {\n"
"    font-size: 14px;\n"
"  }\n"
"  .btn {\n"
"    padding: 0.35rem 0.7rem;\n"
"    font-size: 0.9rem;\n"
"  }\n"
"  .table th, .table td {\n"
"    padding: 0.5rem;\n"
"  }\n"
"}\n"
"\n";

/* HTTP GET handler for serving bootstrap.css */
static esp_err_t bootstrap_css_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/css");
    httpd_resp_send(req, bootstrap_css, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
// GET handler for the root URI.
static esp_err_t root_get_handler(httpd_req_t *req)
{
    char html_response[2048];
    // If no credentials stored (first phase), show the initial configuration form.
    bool first_phase = (wifi_credentials_count == 0) && (stored_camera_id[0] == '\0');
	bool not_configured = sizeof(stored_camera_id[0]) <= 12;
    if (first_phase) {
        // Generate a random camera secret.
        char camera_secret[9];
        generate_random_camera_secret(camera_secret, sizeof(camera_secret));
        
        snprintf(html_response, sizeof(html_response),
            "<!DOCTYPE html>"
            "<html>"
            "<head>"
              "<title>WiFi Manager</title>"
              "<link rel=\"stylesheet\" href=\"/bootstrap.css\">"
              "<link rel=\"icon\" type=\"image/png\" href=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAUAAAAFCAYAAACNbyblAAAAHElEQVQI12P4//8/w38GIAXDIBKE0DHxgljNBAAO9TXL0Y4OHwAAAABJRU5ErkJggg==\" />"
            "</head>"
            "<body class=\"container\">"
              "<h1 class=\"mt-4\">Configure WiFi &amp; Camera</h1>"
              "<form action=\"/save\" method=\"post\">"
                "<div class=\"form-group\">"
                  "<label for=\"ssid\">SSID:</label>"
                  "<input type=\"text\" class=\"form-control\" name=\"ssid\" id=\"ssid\" required>"
                "</div>"
                "<div class=\"form-group\">"
                  "<label for=\"password\">Password:</label>"
                  "<input type=\"password\" class=\"form-control\" name=\"password\" id=\"password\" required>"
                "</div>"
                "<div class=\"form-group\">"
                  "<label for=\"camera_secret\">Camera Secret:</label>"
                  "<input type=\"text\" class=\"form-control\" name=\"camera_secret\" id=\"camera_secret\" value=\"%s\" readonly>"
                "</div>"
                "<button type=\"submit\" class=\"btn btn-primary\">Save</button>"
              "</form>"
            "</body>"
            "</html>", camera_secret);
    } else if(not_configured){
		char camera_secret[9];
        generate_random_camera_secret(camera_secret, sizeof(camera_secret));
        char list_html[1024] = {0};
        strcat(list_html,
            "<table class=\"table table-striped mt-4\">"
            "<thead><tr><th>#</th><th>SSID</th><th>Action</th></tr></thead><tbody>");
        for (int i = 0; i < wifi_credentials_count; i++) {
            char row[256];
            snprintf(row, sizeof(row),
                "<tr><td>%d</td><td>%s</td>"
                "<td><a href=\"/delete?index=%d\" class=\"btn btn-danger btn-sm\">Delete</a></td></tr>",
                i, stored_wifi[i].ssid, i);
            strcat(list_html, row);
        }
        strcat(list_html, "</tbody></table>");
        
        snprintf(html_response, sizeof(html_response),
            "<!DOCTYPE html>"
            "<html>"
            "<head>"
              "<title>WiFi Manager</title>"
              "<link rel=\"stylesheet\" href=\"/bootstrap.css\">"
              "<link rel=\"icon\" type=\"image/png\" href=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAUAAAAFCAYAAACNbyblAAAAHElEQVQI12P4//8/w38GIAXDIBKE0DHxgljNBAAO9TXL0Y4OHwAAAABJRU5ErkJggg==\" />"
            "</head>"
            "<body class=\"container\">"
              "<h2>Home Security</h2>"
              "<h3>Add New Wifi Credentails</h3>"
              "<form action=\"/save\" method=\"post\">"
                "<div class=\"form-group\">"
                  "<label for=\"ssid\">SSID:</label>"
                  "<input type=\"text\" class=\"form-control\" name=\"ssid\" id=\"ssid\" required>"
                "</div>"
                "<div class=\"form-group\">"
                  "<label for=\"password\">Password:</label>"
                  "<input type=\"password\" class=\"form-control\" name=\"password\" id=\"password\" required>"
                "</div>"
				"<div class=\"form-group\">"
                  "<label for=\"camera_secret\">Camera Secret:</label>"
                  "<input type=\"text\" class=\"form-control\" name=\"camera_secret\" id=\"camera_secret\" value=\"%s\" readonly>"
                "</div>"
                "<button type=\"submit\" class=\"btn btn-primary\">Save</button>"
              "</form>"
			  "<h3 class=\"mt-4\">Stored WiFi Credentials</h3>"
              	"%s" // Insert table of stored networks
              "<hr>"
            "</body>"
            "</html>", camera_secret,list_html);
    httpd_resp_send(req, html_response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
	}else {
        // When credentials exist, display them in a table and allow deletion,
        // and also provide a form to add a new WiFi credential.
        char list_html[1024] = {0};
        strcat(list_html,
            "<table class=\"table table-striped mt-4\">"
            "<thead><tr><th>#</th><th>SSID</th><th>Action</th></tr></thead><tbody>");
        for (int i = 0; i < wifi_credentials_count; i++) {
            char row[256];
            snprintf(row, sizeof(row),
                "<tr><td>%d</td><td>%s</td>"
                "<td><a href=\"/delete?index=%d\" class=\"btn btn-danger btn-sm\">Delete</a></td></tr>",
                i, stored_wifi[i].ssid, i);
            strcat(list_html, row);
        }
        strcat(list_html, "</tbody></table>");
        
        snprintf(html_response, sizeof(html_response),
            "<!DOCTYPE html>"
            "<html>"
            "<head>"
              "<title>Home Security</title>"
              "<link rel=\"stylesheet\" href=\"/bootstrap.css\">"
              "<link rel=\"icon\" type=\"image/png\" href=\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAUAAAAFCAYAAACNbyblAAAAHElEQVQI12P4//8/w38GIAXDIBKE0DHxgljNBAAO9TXL0Y4OHwAAAABJRU5ErkJggg==\" />"
            "</head>"
            "<body class=\"container\">"
              "<h2>Home Security</h2>"
              "<h3>Add New Wifi Credentails</h3>"
              "<form action=\"/save\" method=\"post\">"
                "<div class=\"form-group\">"
                  "<label for=\"ssid\">SSID:</label>"
                  "<input type=\"text\" class=\"form-control\" name=\"ssid\" id=\"ssid\" required>"
                "</div>"
                "<div class=\"form-group\">"
                  "<label for=\"password\">Password:</label>"
                  "<input type=\"password\" class=\"form-control\" name=\"password\" id=\"password\" required>"
                "</div>"
                "<button type=\"submit\" class=\"btn btn-primary\">Save</button>"
              "</form>"
			  "<h3 class=\"mt-4\">Stored WiFi Credentials</h3>"
              	"%s" // Insert table of stored networks
              "<hr>"
            "</body>"
            "</html>", list_html);
    }
    httpd_resp_send(req, html_response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// POST handler for saving new credentials.
static esp_err_t save_post_handler(httpd_req_t *req)
{
    char buf[100];
    int total_len = req->content_len;
    int received = 0, ret;

    while (received < total_len) {
        ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) {
            return ESP_FAIL;
        }
        received += ret;
    }
    buf[total_len] = '\0';

    char ssid[33] = {0};
    char password[65] = {0};
    char camera_secret[33] = {0};

    char *p = strstr(buf, "ssid=");
    if (p) {
        p += 5;
        sscanf(p, "%32[^&]", ssid);
    }
    p = strstr(buf, "password=");
    if (p) {
        p += 9;
        sscanf(p, "%64[^&]", password);
    }
    p = strstr(buf, "camera_secret=");
    if (p) {
        p += 14;
        sscanf(p, "%32[^&]", camera_secret);
    }

    ESP_LOGI(TAG, "Received POST: ssid=%s, password=%s, camera_secret=%s", ssid, password, camera_secret);
    save_wifi_credentials(ssid, password, (strlen(camera_secret) > 0) ? camera_secret : NULL);

    const char* resp = "<!DOCTYPE html><html><body><h1>Configuration Saved! Rebooting...</h1></body></html>";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    esp_restart();
    return ESP_OK;
}

// GET handler for deletion of a credential.
static esp_err_t delete_get_handler(httpd_req_t *req)
{
    char query[32] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char index_str[8] = {0};
        if (httpd_query_key_value(query, "index", index_str, sizeof(index_str)) == ESP_OK) {
            int index = atoi(index_str);
            ESP_LOGI(TAG, "Deleting credential at index %d", index);
            delete_wifi_credential(index);
        }
    }
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// -----------------------------------------
// Webserver Startup
// -----------------------------------------
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
	config.stack_size = 8192;
   

    // Optionally increase stack size if needed, e.g., config.stack_size = 8192;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t save_uri = {
            .uri       = "/save",
            .method    = HTTP_POST,
            .handler   = save_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &save_uri);

        httpd_uri_t delete_uri = {
            .uri       = "/delete",
            .method    = HTTP_GET,
            .handler   = delete_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &delete_uri);

        // Register handler for offline Bootstrap CSS
        httpd_uri_t bootstrap_uri = {
            .uri       = "/bootstrap.css",
            .method    = HTTP_GET,
            .handler   = bootstrap_css_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &bootstrap_uri);
    }
    return server;
}
// ---------- mDNS Setup ----------

void start_mdns() {
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("homesecurity"));  // "wifimanager.local"
    ESP_ERROR_CHECK(mdns_instance_name_set("Home Security"));
}
// ---------- WiFi Connection Task (STA Mode) ----------

static bool try_connect_to_ssid(const wifi_credentials_t* cred) {
    ESP_LOGI(TAG, "Attempting connection to SSID: %s", cred->ssid);

    // Prepare WiFi configuration.
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, cred->ssid, sizeof(wifi_config.sta.ssid)-1);
    strncpy((char*)wifi_config.sta.password, cred->password, sizeof(wifi_config.sta.password)-1);

    // Set the configuration and initiate connection.
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
        ESP_LOGE(TAG, "Failed to initiate connection: %s", esp_err_to_name(err));
        return false;
    }

    // Wait for up to 5 seconds for a successful connection.
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                             WIFI_CONNECTED_BIT,
                                             pdTRUE,   // clear bits on exit
                                             pdTRUE,   // wait for all bits (only one in this case)
                                             5000 / portTICK_PERIOD_MS);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to %s", cred->ssid);
        return true;
    }
    
    // Timed out, so disconnect to cancel the ongoing attempt.
    ESP_LOGI(TAG, "Timeout connecting to %s, disconnecting", cred->ssid);
    esp_wifi_disconnect();
    return false;
}


void uart_request_task(void *pvParameters)
{
    uint8_t data[128];
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; // Null-terminate the received data
            ESP_LOGI(TAG, "UART received: %s", (char*)data);
            // Check if the command is "GET_CRED"
            if (strcmp((char*)data, "GET_CRED") == 0) {
                // Send the current credentials via UART
                uart_send_credentials(&current_wifi);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void wifi_connect_task(void) {
    bool connected = false;
    for (int round = 0; round < 2 && !connected; round++) {
        for (int i = 0; i < wifi_credentials_count && !connected; i++) {
            ESP_LOGI(TAG, "Attempting connection to SSID: %s", stored_wifi[i].ssid);
            wifi_config_t wifi_config = {0};
            strncpy((char*)wifi_config.sta.ssid, stored_wifi[i].ssid, sizeof(wifi_config.sta.ssid)-1);
            strncpy((char*)wifi_config.sta.password, stored_wifi[i].password, sizeof(wifi_config.sta.password)-1);
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
                ESP_LOGE(TAG, "Failed to initiate connection: %s", esp_err_to_name(err));
                continue;
            }
            EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdTRUE, pdTRUE, 5000 / portTICK_PERIOD_MS);
            if (bits & WIFI_CONNECTED_BIT) {
                connected = true;
                memcpy(&current_wifi, &stored_wifi[i], sizeof(wifi_credentials_t));
                ESP_LOGI(TAG, "Connected to %s", stored_wifi[i].ssid);
                break;
            }
            ESP_LOGI(TAG, "Timeout connecting to %s, disconnecting", stored_wifi[i].ssid);
            esp_wifi_disconnect();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    if (!connected) {
        ESP_LOGI(TAG, "Failed to connect to any stored network. Switching to AP mode.");
        esp_wifi_stop();
        wifi_init_softap();
        start_mdns();
        start_webserver();
    }
}



static void wifi_init_sta(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}


// ---------- WiFi Initialization (AP Mode) ----------

static void wifi_init_softap(void) {
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t ap_config = {
        .ap = {
            .ssid = "HomeSecurity",
            .ssid_len = 0,
            .password = "homeadmin",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };
    if (strlen((char*)ap_config.ap.password) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "SoftAP started. SSID:%s password:%s", ap_config.ap.ssid, ap_config.ap.password);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            ESP_LOGI(TAG, "WiFi STA started");
            gpio_set_level(1, 0);
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGI(TAG, "Disconnected");
            gpio_set_level(1, 1);
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        } else if (event_id == WIFI_EVENT_AP_START) {
            ESP_LOGI(TAG, "WiFi AP started");
            gpio_set_level(1, 1); // Turn on blue LED
            gpio_set_level(2, 1); // Turn on green LED (AP mode)
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Got IP");
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        gpio_set_level(1, 0);
        gpio_set_level(2, 1); // Turn on green LED (success)
        esp_timer_start_once(green_led_timer, 10000000); // 10 seconds in microseconds
        ESP_LOGI(TAG, "WiFi connected successfully, green LED on for 10 seconds");
    }
}





void handle_wifi_connect(void) {
    init_leds();

    // Create timer for green LED
    esp_timer_create_args_t timer_args = {
        .callback = &turn_off_green,
        .arg = NULL,
        .name = "green_led_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &green_led_timer));

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    uart_init_custom();
    load_wifi_credentials();
    wifi_event_group = xEventGroupCreate();

    if (wifi_credentials_count == 0 || stored_camera_id[0] == '\0') {
        ESP_LOGI(TAG, "No WiFi credentials found. Starting in AP mode for initial configuration.");
        wifi_init_softap();
        start_mdns();
        start_webserver();
    } else {
        esp_netif_create_default_wifi_sta();
        wifi_init_sta();
        wifi_connect_task();
    }
}


