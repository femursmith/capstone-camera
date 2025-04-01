#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

// Include your peer connection header
#include "peer_connection.h"

// Extern declarations for your WebRTC globals and semaphore
extern PeerConnection *g_pc;
extern int gDataChannelOpened;
extern SemaphoreHandle_t xSemaphore;

static const char *TAG = "MJPEG_Stream";

// Define your AMB82-mini server settings
#define MJPEG_SERVER_IP   "192.168.103.4"
#define MJPEG_SERVER_PORT 80
#define MJPEG_REQUEST     "GET / HTTP/1.1\r\nHost: " MJPEG_SERVER_IP "\r\nConnection: keep-alive\r\n\r\n"

// The boundary string used by the server (as sent in the boundary chunk)
static const char *BOUNDARY = "\r\n--123456789000000000000987654321\r\n";

// Helper: Read a line (ending with "\r\n") from socket into buf (max len bytes)
static int read_line(int sock, char *buf, int maxlen)
{
    int i = 0;
    char c = 0;
    int n;
    while (i < maxlen - 1) {
        n = recv(sock, &c, 1, 0);
        if (n <= 0) break;
        buf[i++] = c;
        if (i >= 2 && buf[i-2] == '\r' && buf[i-1] == '\n') {
            break;
        }
    }
    buf[i] = '\0';
    return i;
}

// Helper: Read exactly n bytes from socket into buf
static int read_exact(int sock, char *buf, int n)
{
    int total = 0;
    int bytes;
    while (total < n) {
        bytes = recv(sock, buf + total, n - total, 0);
        if (bytes <= 0) {
            return total;
        }
        total += bytes;
    }
    return total;
}

// Helper: Read a single chunk from the HTTP chunked transfer stream.
// It reads the chunk-size line, then the chunk data, then the trailing CRLF.
// Returns a pointer to a newly allocated buffer (null-terminated for text chunks)
// and stores the chunk size in out_size. Caller is responsible for freeing it.
static char* read_chunk(int sock, int *out_size)
{
    char chunk_size_line[64] = {0};
    if (read_line(sock, chunk_size_line, sizeof(chunk_size_line)) <= 0) {
        ESP_LOGE(TAG, "Failed to read chunk size line");
        return NULL;
    }
    int chunk_size = (int)strtol(chunk_size_line, NULL, 16);
    if (chunk_size <= 0) {
        // Zero chunk size indicates end of stream.
        *out_size = 0;
        return NULL;
    }
    char *data = malloc(chunk_size + 1);
    if (!data) {
        ESP_LOGE(TAG, "Failed to allocate memory for chunk data");
        return NULL;
    }
    if (read_exact(sock, data, chunk_size) != chunk_size) {
        ESP_LOGE(TAG, "Failed to read complete chunk data");
        free(data);
        return NULL;
    }
    data[chunk_size] = '\0'; // Null-terminate for text (if needed)
    // Read the trailing CRLF
    char dummy[2];
    read_exact(sock, dummy, 2);
    *out_size = chunk_size;
    return data;
}

void mjpeg_stream_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting MJPEG stream task");

    // Resolve host and create socket
    struct addrinfo hints = {0};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo *res;
    if (getaddrinfo(MJPEG_SERVER_IP, "80", &hints, &res) != 0) {
        ESP_LOGE(TAG, "DNS lookup failed");
        vTaskDelete(NULL);
        return;
    }
    int sock = socket(res->ai_family, res->ai_socktype, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to allocate socket");
        freeaddrinfo(res);
        vTaskDelete(NULL);
        return;
    }
    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(TAG, "Socket connect failed: errno %d", errno);
        close(sock);
        freeaddrinfo(res);
        vTaskDelete(NULL);
        return;
    }
    freeaddrinfo(res);

    // Send HTTP GET request
    if (send(sock, MJPEG_REQUEST, strlen(MJPEG_REQUEST), 0) < 0) {
        ESP_LOGE(TAG, "Failed to send request");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    // Read and discard the HTTP response headers
    char line[128];
    do {
        if (read_line(sock, line, sizeof(line)) <= 0) break;
        // Uncomment next line for header debugging:
        // ESP_LOGI(TAG, "Header: %s", line);
    } while (strcmp(line, "\r\n") != 0);

    // Process the stream in groups of three chunks: header, image data, and boundary.
    while (1) {
        int size;

        // 1. Read header chunk (should contain "Content-Type" and "Content-Length")
        char *header_chunk = read_chunk(sock, &size);
        if (!header_chunk) {
            ESP_LOGE(TAG, "No header chunk received; ending stream");
            break;
        }
        ESP_LOGI(TAG, "Header chunk received (%d bytes): %s", size, header_chunk);

        // Parse Content-Length from header
        int content_length = 0;
        char *cl_ptr = strstr(header_chunk, "Content-Length:");
        if (cl_ptr) {
            cl_ptr += strlen("Content-Length:");
            content_length = atoi(cl_ptr);
        }
        free(header_chunk);

        if (content_length <= 0) {
            ESP_LOGE(TAG, "Invalid content length: %d", content_length);
            break;
        }

        // 2. Read image data chunk (expected to be exactly content_length bytes)
        char *image_data = read_chunk(sock, &size);
        if (!image_data || size != content_length) {
            ESP_LOGE(TAG, "Image data chunk error: expected %d, got %d", content_length, size);
            if (image_data) free(image_data);
            break;
        }
        ESP_LOGI(TAG, "Image data chunk received, size=%d", size);

        // 3. Read boundary chunk
        char *boundary_chunk = read_chunk(sock, &size);
        if (!boundary_chunk) {
            ESP_LOGE(TAG, "No boundary chunk received; ending stream");
            free(image_data);
            break;
        }
        // Compare with expected boundary string
        if (strncmp(boundary_chunk, BOUNDARY, strlen(BOUNDARY)) != 0) {
            ESP_LOGW(TAG, "Boundary mismatch. Received: %s", boundary_chunk);
        }
        free(boundary_chunk);

        // Send the image over the WebRTC data channel if it is open
        if (gDataChannelOpened && g_pc) {
            if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
                peer_connection_datachannel_send(g_pc, image_data, content_length);
                xSemaphoreGive(xSemaphore);
            }
        }
        free(image_data);

        // Optional: add a short delay if needed
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    close(sock);
    ESP_LOGI(TAG, "MJPEG stream task ending");
    vTaskDelete(NULL);
}
