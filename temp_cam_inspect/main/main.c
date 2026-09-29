#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/jpeg_encode.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_cam_sensor.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "driver/uart_vfs.h"
#include <math.h>

// 32-byte Standalone Target Scoring Metadata Structure (Steganographically injected after JPEG EOI)
typedef struct __attribute__((packed)) {
    uint8_t magic[4];       // 0xDE, 0xAD, 0xBE, 0xEF
    uint8_t laser_found;    // 0 = no, 1 = yes
    uint8_t zone;           // 0 = none, 1 = green, 2 = red
    uint8_t score_ring;     // 0 = miss, 1..10 = ring, 11 = 10X
    uint8_t is_calibrated;  // 0 = no, 1 = yes
    uint16_t laser_x_px;    // Camera pixel coordinates
    uint16_t laser_y_px;
    float laser_x_mm;       // Calibrated coordinates in mm relative to target card
    float laser_y_mm;
    float laser_dist_mm;    // Radial distance from center in mm
    uint8_t padding[8];     // Padding to ensure exactly 32 bytes
} shot_metadata_t;

// Global Calibration State
static float g_homography[9] = {1.0f, 0.0f, 0.0f,  0.0f, 1.0f, 0.0f,  0.0f, 0.0f, 1.0f}; // Identity matrix
static bool g_is_calibrated = false;

static const char *TAG = "P4_UART_STREAM";

// TEST CONFIGURATION MODES (ONLY ONE SHOULD BE 1 AT A TIME FOR ISOLATED TESTING)
#define UART_DIAGNOSTIC_MODE    0
#define CAMERA_TEST_MODE        0
#define SINGLE_FRAME_TEST_MODE  0
#define SINGLE_RAW_FRAME_TEST_MODE 0
#define USB_TRANSPORT_TEST_MODE 0
#define USB_TEST_PAYLOAD_SIZE   4096

// TARGET STREAM RATE (FPS) - Paces the stream to not overload the UART bandwidth
#define STREAM_MAX_FPS          12

// I2C Pin definitions for ESP32-P4 Pico board (Camera)
#define I2C_MASTER_SCL_IO   GPIO_NUM_8
#define I2C_MASTER_SDA_IO   GPIO_NUM_7
#define I2C_MASTER_NUM      I2C_NUM_0

// UART Pin definitions for Bridge (Using clean, isolated GPIO 21)
#define BRIDGE_UART_NUM     UART_NUM_1
#define BRIDGE_TX_PIN       GPIO_NUM_21
#define BRIDGE_RX_PIN       GPIO_NUM_27
#define BRIDGE_BAUD         115200

#if !UART_DIAGNOSTIC_MODE
static int g_video_fd = -1;
static uint8_t *g_buffers[2] = {NULL, NULL};
static uint32_t g_buf_len[2] = {0, 0};

static jpeg_encoder_handle_t g_jpeg_handle = NULL;
static uint8_t *g_jpeg_out_buf = NULL;
static uint32_t g_jpeg_out_size = 0;

// Diagnostics counters
static uint32_t __attribute__((unused)) stat_captured = 0;
static uint32_t __attribute__((unused)) stat_encoded = 0;
static uint64_t __attribute__((unused)) stat_jpeg_bytes_total = 0;
static uint32_t __attribute__((unused)) stat_transmitted = 0;
static uint64_t __attribute__((unused)) stat_tx_bytes_total = 0;
#endif

// Calculate CRC32 (Standard IEEE 802.3 ethernet representation)
static uint32_t calculate_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

// Send binary packet over UART with Sync, Version, Frame ID, Length, and CRC32
static void __attribute__((unused)) send_packet(uint32_t frame_id, const uint8_t *payload, uint32_t payload_len)
{
    uint8_t header[15];
    header[0] = 0xAA;
    header[1] = 0x55;
    header[2] = 0x01; // Version

    // Little-endian frame ID
    header[3] = (uint8_t)(frame_id & 0xFF);
    header[4] = (uint8_t)((frame_id >> 8) & 0xFF);
    header[5] = (uint8_t)((frame_id >> 16) & 0xFF);
    header[6] = (uint8_t)((frame_id >> 24) & 0xFF);

    // Little-endian payload length
    header[7] = (uint8_t)(payload_len & 0xFF);
    header[8] = (uint8_t)((payload_len >> 8) & 0xFF);
    header[9] = (uint8_t)((payload_len >> 16) & 0xFF);
    header[10] = (uint8_t)((payload_len >> 24) & 0xFF);

    // Calculate CRC32 of the payload
    uint32_t crc32 = calculate_crc32(payload, payload_len);
    header[11] = (uint8_t)(crc32 & 0xFF);
    header[12] = (uint8_t)((crc32 >> 8) & 0xFF);
    header[13] = (uint8_t)((crc32 >> 16) & 0xFF);
    header[14] = (uint8_t)((crc32 >> 24) & 0xFF);

    // Send over USB-Serial-JTAG (if driver is installed and connected) - Commented out because we are using standard UART0 console over COM15
    // if (usb_serial_jtag_is_driver_installed() && usb_serial_jtag_is_connected()) {
    //     usb_serial_jtag_write_bytes(header, sizeof(header), pdMS_TO_TICKS(100));
    //     usb_serial_jtag_write_bytes(payload, payload_len, pdMS_TO_TICKS(100));
    //     usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(100));
    // }

    // Send directly over raw UART0 (COM15) to bypass VFS console overhead
    uart_write_bytes(UART_NUM_0, header, sizeof(header));
    uart_write_bytes(UART_NUM_0, payload, payload_len);
    // ESP_LOGI(TAG, "stdout write return: w1=%d, w2=%d", (int)w1, (int)w2);

    // Send header (Commented out to bypass the slow 115200 baud UART bridge bottleneck)
    // uart_write_bytes(BRIDGE_UART_NUM, (const char *)header, sizeof(header));
    // Send payload
    // uart_write_bytes(BRIDGE_UART_NUM, (const char *)payload, payload_len);

    // Block until UART TX is completed to regulate throughput & avoid buffer overflow
    // uart_wait_tx_done(BRIDGE_UART_NUM, pdMS_TO_TICKS(150));
}

// Calculate ring score and red/green zone based on 50x50mm target specs (2cm diameter center circle)
static void get_target_score(float target_x, float target_y, uint8_t *ring, uint8_t *zone, float *distance)
{
    float dx = target_x - 25.0f; // Center is at 25.0mm on a 50x50mm target
    float dy = target_y - 25.0f;
    float r = sqrtf(dx * dx + dy * dy);
    *distance = r;
    
    // Zone classification: White circle is Green Zone (diameter 2cm -> radius 10mm)
    if (r <= 10.0f) {
        *zone = 1; // Green Zone
    } else {
        *zone = 2; // Red Zone
    }
    
    // Ring classification (Rings 7-10 map to Green Zone, Rings 1-6 map to Red Zone)
    if (r <= 0.25f) {
        *ring = 11; // 10X
    } else if (r <= 1.5f) {
        *ring = 10;
    } else if (r <= 4.0f) {
        *ring = 9;
    } else if (r <= 7.0f) {
        *ring = 8;
    } else if (r <= 10.0f) {
        *ring = 7;
    } else if (r <= 13.0f) {
        *ring = 6;
    } else if (r <= 16.0f) {
        *ring = 5;
    } else if (r <= 19.0f) {
        *ring = 4;
    } else if (r <= 22.0f) {
        *ring = 3;
    } else if (r <= 25.0f) {
        *ring = 2;
    } else if (r <= 28.0f) {
        *ring = 1;
    } else {
        *ring = 0; // Miss
    }
}

// =========================================================================
// STANDALONE DETERMINISTIC NERF PROJECTILE DETECTOR STATE & BUFFERS
// =========================================================================
typedef enum {
    P4_STATE_IDLE = 0,
    P4_STATE_SETTLING,
    P4_STATE_CALIBRATING,
    P4_STATE_ARMED,
    P4_STATE_HIT_CONFIRMED,
    P4_STATE_LOCKED
} p4_state_t;

static volatile p4_state_t g_p4_state = P4_STATE_ARMED; // Armed by default for instant readiness
static int64_t g_p4_state_start_us = 0;
static int64_t g_p4_cooldown_until_us = 0;

#define NERF_GRID_SIZE 130
static uint16_t s_p4_baseline_roi[NERF_GRID_SIZE][NERF_GRID_SIZE] = {0};
static int s_p4_baseline_samples = 0;
static int s_p4_confirm_frames = 0;

// Background task to receive calibration and target pop commands over UART0
static void uart_rx_task(void *arg)
{
    uint8_t buffer[128];
    int idx = 0;
    int64_t last_byte_time = 0;
    char text_cmd_buf[32];
    int text_cmd_idx = 0;
    
    ESP_LOGI("STANDALONE_CV", "UART RX task started. Listening for calibration & pop commands...");
    
    while (1) {
        uint8_t byte;
        int len = uart_read_bytes(UART_NUM_0, &byte, 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            int64_t now = esp_timer_get_time();
            
            // Check for ASCII text commands: UP, ARM, DOWN, STATUS
            if (byte == '\n' || byte == '\r') {
                if (text_cmd_idx > 0) {
                    text_cmd_buf[text_cmd_idx] = '\0';
                    if (strstr(text_cmd_buf, "UP") || strstr(text_cmd_buf, "ARM")) {
                        g_p4_state = P4_STATE_SETTLING;
                        g_p4_state_start_us = now;
                        s_p4_baseline_samples = 0;
                        s_p4_confirm_frames = 0;
                        ESP_LOGI("P4_NERF", "🎯 Received UP/ARM command. Target settling for 650ms...");
                    } else if (strstr(text_cmd_buf, "DOWN")) {
                        g_p4_state = P4_STATE_IDLE;
                        ESP_LOGI("P4_NERF", "Received DOWN command. Target idle.");
                    }
                    text_cmd_idx = 0;
                }
            } else if (text_cmd_idx < (int)sizeof(text_cmd_buf) - 1 && byte >= 32 && byte <= 126) {
                text_cmd_buf[text_cmd_idx++] = (char)byte;
            }

            // Binary homography calibration packet parsing (0xCC 0x01 ...)
            if (idx > 0 && (now - last_byte_time > 100000)) { // 100ms packet timeout
                idx = 0;
            }
            last_byte_time = now;
            
            if (idx == 0) {
                if (byte == 0xCC) {
                    buffer[idx++] = byte;
                }
            } else if (idx == 1) {
                if (byte == 0x01) {
                    buffer[idx++] = byte;
                } else {
                    idx = 0;
                }
            } else if (idx >= 2 && idx < 38) {
                buffer[idx++] = byte;
            } else if (idx == 38) {
                buffer[idx] = byte;
                
                // Verify XOR checksum
                uint8_t check = 0;
                for (int i = 1; i < 38; i++) {
                    check ^= buffer[i];
                }
                
                if (check == buffer[38]) {
                    memcpy(g_homography, &buffer[2], 36);
                    g_is_calibrated = true;
                    ESP_LOGI("STANDALONE_CV", "Homography calibrated successfully!");
                } else {
                    ESP_LOGE("STANDALONE_CV", "Checksum error! Calculated: 0x%02X, Recv: 0x%02X", check, buffer[38]);
                }
                idx = 0; // Reset packet state
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

// Initialize USB-Serial-JTAG driver
static void init_usb_serial_jtag(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.tx_buffer_size = 65536; // 64KB TX buffer to hold larger JPEG frames
    cfg.rx_buffer_size = 1024;
    
    esp_err_t ret = usb_serial_jtag_driver_install(&cfg);
    if (ret == ESP_OK) {
        if (usb_serial_jtag_is_connected()) {
            usb_serial_jtag_vfs_use_driver();
            usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
            ESP_LOGI(TAG, "USB-Serial-JTAG connected. VFS switched to driver mode.");
        } else {
            ESP_LOGI(TAG, "USB-Serial-JTAG driver installed. Keeping console on UART0.");
        }
    } else {
        ESP_LOGE(TAG, "Failed to install USB-Serial-JTAG driver: %s", esp_err_to_name(ret));
    }
}

// Initialize UART for bridging
static void init_bridge_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = BRIDGE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(BRIDGE_UART_NUM, 4096, 32768, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BRIDGE_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BRIDGE_UART_NUM, BRIDGE_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "Bridge UART%d initialized at %d baud on TX:%d RX:DISABLED", BRIDGE_UART_NUM, BRIDGE_BAUD, BRIDGE_TX_PIN);
}

#if UART_DIAGNOSTIC_MODE
// UART Diagnostic Mode Task
static void uart_diagnostic_task(void *arg)
{
    uint32_t frame_id = 0;
    const char *payload = "UART_TEST_PACKET_123456";
    uint32_t payload_len = 23;

    ESP_LOGI(TAG, "UART Diagnostic Mode active. Sending test packets...");

    while (true) {
        frame_id++;
        send_packet(frame_id, (const uint8_t *)payload, payload_len);
        ESP_LOGI(TAG, "Sent UART diagnostic packet %lu", frame_id);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

#if !UART_DIAGNOSTIC_MODE
// Helper to send data over USB-Serial-JTAG in chunks with a finite timeout
static esp_err_t send_via_usb_serial_jtag(const uint8_t *buffer, size_t length, TickType_t timeout_per_write)
{
    size_t offset = 0;
    const size_t chunk_size = 1024;
    int timeout_retries = 0;
    const int max_retries = 10; // Abort after 10 consecutive timeouts with no progress

    while (offset < length) {
        size_t bytes_to_write = length - offset;
        if (bytes_to_write > chunk_size) {
            bytes_to_write = chunk_size;
        }

        int written = usb_serial_jtag_write_bytes(buffer + offset, bytes_to_write, timeout_per_write);
        if (written < 0) {
            ESP_LOGE(TAG, "usb_serial_jtag_write_bytes returned error: %d", written);
            return ESP_FAIL;
        }
        if (written == 0) {
            timeout_retries++;
            if (timeout_retries >= max_retries) {
                ESP_LOGE(TAG, "USB JTAG write timeout: Host stopped consuming data.");
                return ESP_ERR_TIMEOUT;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        timeout_retries = 0; // Reset retries on successful write progress
        offset += written;
    }
    return ESP_OK;
}

static esp_err_t send_via_uart0(const uint8_t *buffer, size_t length)
{
    size_t offset = 0;
    const size_t chunk_size = 1024;
    while (offset < length) {
        size_t bytes_to_write = length - offset;
        if (bytes_to_write > chunk_size) {
            bytes_to_write = chunk_size;
        }
        int written = uart_write_bytes(UART_NUM_0, buffer + offset, bytes_to_write);
        if (written < 0) {
            ESP_LOGE(TAG, "uart_write_bytes returned error: %d", written);
            return ESP_FAIL;
        }
        offset += written;
    }
    return ESP_OK;
}

static void set_camera_exposure_target(int value)
{
#if !SINGLE_FRAME_TEST_MODE
    struct v4l2_ext_control ext_ctrl = {
        .id = V4L2_CID_EXPOSURE,
        .value = value,
    };
    struct v4l2_ext_controls ext_ctrls = {
        .ctrl_class = V4L2_CTRL_CLASS_USER,
        .count = 1,
        .controls = &ext_ctrl,
    };
    if (ioctl(g_video_fd, VIDIOC_S_EXT_CTRLS, &ext_ctrls) == 0) {
        ESP_LOGI("CAMERA", "Successfully set camera AE exposure target to %d", value);
    } else {
        ESP_LOGW("CAMERA", "Failed to set camera AE exposure target");
    }
#endif
}

// Camera Capture and Stream Task
static void camera_stream_task(void *arg)
{
#if USB_TRANSPORT_TEST_MODE
    ESP_LOGI(TAG, "USB_TRANSPORT_TEST_MODE active. Testing USB transport with %d bytes...", USB_TEST_PAYLOAD_SIZE);
    
    // Allocate a buffer for the test pattern
    uint8_t *test_payload = malloc(USB_TEST_PAYLOAD_SIZE);
    if (!test_payload) {
        ESP_LOGE(TAG, "Failed to allocate test payload");
        vTaskDelete(NULL);
        return;
    }
    // Fill the payload with a simple predictable test pattern
    for (int i = 0; i < USB_TEST_PAYLOAD_SIZE; i++) {
        test_payload[i] = (uint8_t)(i & 0xFF);
    }
    
    // Allow host USB port initialization to stabilize
    vTaskDelay(pdMS_TO_TICKS(500));

    uint32_t out_len = USB_TEST_PAYLOAD_SIZE;
    extern uint32_t esp_rom_crc32_le(uint32_t crc, const uint8_t *buf, uint32_t len);
    uint32_t crc32 = esp_rom_crc32_le(0xFFFFFFFF, test_payload, out_len) ^ 0xFFFFFFFF;

    printf("RAW CRC: %08lX\n", (unsigned long)crc32);
    fflush(stdout);

    uint8_t header[15];
    header[0] = 0xAA;
    header[1] = 0x55;
    header[2] = 0x01;
    
    uint32_t frame_id = 1;
    header[3] = (uint8_t)(frame_id & 0xFF);
    header[4] = (uint8_t)((frame_id >> 8) & 0xFF);
    header[5] = (uint8_t)((frame_id >> 16) & 0xFF);
    header[6] = (uint8_t)((frame_id >> 24) & 0xFF);

    header[7] = (uint8_t)(out_len & 0xFF);
    header[8] = (uint8_t)((out_len >> 8) & 0xFF);
    header[9] = (uint8_t)((out_len >> 16) & 0xFF);
    header[10] = (uint8_t)((out_len >> 24) & 0xFF);

    header[11] = (uint8_t)(crc32 & 0xFF);
    header[12] = (uint8_t)((crc32 >> 8) & 0xFF);
    header[13] = (uint8_t)((crc32 >> 16) & 0xFF);
    header[14] = (uint8_t)((crc32 >> 24) & 0xFF);

    printf("Sending RAW Bayer packet (frame=1, len=%lu) exclusively over COM15...\n", (unsigned long)out_len);
    fflush(stdout);
    
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t err = send_via_uart0(header, sizeof(header));
    if (err == ESP_OK) {
        err = send_via_uart0(test_payload, out_len);
    }

    if (err == ESP_OK) {
        esp_err_t tx_done_err = uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(5000));
        if (tx_done_err == ESP_OK) {
            printf("RAW TRANSMISSION COMPLETE\n");
            fflush(stdout);
        } else {
            ESP_LOGE(TAG, "uart_wait_tx_done timed out: %s", esp_err_to_name(tx_done_err));
        }
    } else {
        ESP_LOGE(TAG, "Failed to transmit test packet over UART0: %s", esp_err_to_name(err));
    }

    free(test_payload);
    vTaskDelete(NULL);
    return;
#else // !USB_TRANSPORT_TEST_MODE
#if SINGLE_RAW_FRAME_TEST_MODE
    struct v4l2_buffer buf = {0};
    
    // Discard the first 100 frames to let the sensor wake up, stabilize exposure, and output valid pixels
    ESP_LOGI(TAG, "SINGLE_RAW_FRAME_TEST_MODE: Skipping first 100 frames for sensor stabilization...");
    for (int i = 0; i < 100; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(g_video_fd, VIDIOC_DQBUF, &buf) == 0) {
            ioctl(g_video_fd, VIDIOC_QBUF, &buf);
        }
    }

    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    ESP_LOGI(TAG, "SINGLE_RAW_FRAME_TEST_MODE: Dequeuing stabilized raw frame (Frame 26)...");
    if (ioctl(g_video_fd, VIDIOC_DQBUF, &buf) == 0) {
        struct v4l2_format fmt = {0};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(g_video_fd, VIDIOC_G_FMT, &fmt);

        char fmt_str[5] = {0};
        memcpy(fmt_str, &fmt.fmt.pix.pixelformat, 4);

        printf("RAW DIAGNOSTICS: width=%ld, height=%ld, bytesperline=%ld, pixelformat=%s (0x%lX), bytesused=%ld, expected_size=%d, buffer_length=%ld, buffer_index=%ld\n",
               (long)fmt.fmt.pix.width, 
               (long)fmt.fmt.pix.height, 
               (long)fmt.fmt.pix.bytesperline, 
               fmt_str, 
               (long)fmt.fmt.pix.pixelformat, 
               (long)buf.bytesused, 
               800*800, 
               (long)buf.length, 
               (long)buf.index);

        uint8_t *frame_ptr = g_buffers[buf.index];
        uint32_t out_len = buf.bytesused;

        if (frame_ptr != NULL && out_len > 0) {
            extern uint32_t esp_rom_crc32_le(uint32_t crc, const uint8_t *buf, uint32_t len);
            uint32_t crc32 = esp_rom_crc32_le(0xFFFFFFFF, frame_ptr, out_len) ^ 0xFFFFFFFF;

            printf("RAW CRC: %08lX\n", (unsigned long)crc32);

            uint8_t header[15];
            header[0] = 0xAA;
            header[1] = 0x55;
            header[2] = 0x01;
            
            uint32_t frame_id = 1;
            header[3] = (uint8_t)(frame_id & 0xFF);
            header[4] = (uint8_t)((frame_id >> 8) & 0xFF);
            header[5] = (uint8_t)((frame_id >> 16) & 0xFF);
            header[6] = (uint8_t)((frame_id >> 24) & 0xFF);

            header[7] = (uint8_t)(out_len & 0xFF);
            header[8] = (uint8_t)((out_len >> 8) & 0xFF);
            header[9] = (uint8_t)((out_len >> 16) & 0xFF);
            header[10] = (uint8_t)((out_len >> 24) & 0xFF);

            header[11] = (uint8_t)(crc32 & 0xFF);
            header[12] = (uint8_t)((crc32 >> 8) & 0xFF);
            header[13] = (uint8_t)((crc32 >> 16) & 0xFF);
            header[14] = (uint8_t)((crc32 >> 24) & 0xFF);

            printf("Sending RAW Bayer packet (frame=1, len=%lu) exclusively over COM15...\n", (unsigned long)out_len);
            fflush(stdout);
            
            // Allow previous stdout/console logs to fully flush
            vTaskDelay(pdMS_TO_TICKS(100));

            // Write raw bytes directly to USB-Serial-JTAG driver in chunked fashion to avoid VFS/console corruption
            esp_err_t err = send_via_uart0(header, sizeof(header));
            if (err == ESP_OK) {
                err = send_via_uart0(frame_ptr, out_len);
            }

            if (err == ESP_OK) {
                esp_err_t tx_done_err = uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(5000));
                if (tx_done_err == ESP_OK) {
                    printf("RAW TRANSMISSION COMPLETE\n");
                    fflush(stdout);
                } else {
                    ESP_LOGE(TAG, "uart_wait_tx_done timed out: %s", esp_err_to_name(tx_done_err));
                }
            } else {
                ESP_LOGE(TAG, "Failed to transmit raw packet over UART0: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGE(TAG, "Invalid buffer or length (ptr=%p, len=%lu)", frame_ptr, (unsigned long)out_len);
        }

        ioctl(g_video_fd, VIDIOC_QBUF, &buf);
    } else {
        ESP_LOGE(TAG, "Failed to dequeue buffer from /dev/video0");
    }

    ESP_LOGI(TAG, "SINGLE_RAW_FRAME_TEST_MODE: Test complete. Deleting task.");
    vTaskDelete(NULL);
#else
    struct v4l2_buffer buf = {0};
    int frame_count = 0;
    int64_t last_transmit_time = 0;
    int64_t last_stat_time = esp_timer_get_time();
    const int64_t frame_interval_us = 1000000 / STREAM_MAX_FPS;

    ESP_LOGI(TAG, "Starting camera capture loop (Max Target FPS: %d)...", STREAM_MAX_FPS);

    while (true) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        // Dequeue frame from V4L2 camera driver (blocks until a frame is ready)
        if (ioctl(g_video_fd, VIDIOC_DQBUF, &buf) == 0) {
            frame_count++;
            
            // Pace the frame stream to target FPS
            int64_t now = esp_timer_get_time();
            if (now - last_transmit_time < frame_interval_us) {
                // Drop this frame to limit FPS in software without encoding / transmitting it
                ioctl(g_video_fd, VIDIOC_QBUF, &buf);
                continue;
            }
            last_transmit_time = now;
            stat_captured++;

            uint8_t *frame_ptr = g_buffers[buf.index];
            uint32_t out_len = buf.bytesused;

            if (frame_ptr != NULL && out_len > 0) {
                // Keep camera exposure balanced for vibrant yellow foam and orange tip detection
                static bool s_cam_exposure_set = false;
                if (!s_cam_exposure_set) {
                    set_camera_exposure_target(65);
                    s_cam_exposure_set = true;
                }

                // =========================================================================
                // STANDALONE DETERMINISTIC NERF PROJECTILE DETECTOR FOR ESP32-P4
                // =========================================================================
                int64_t now_us = esp_timer_get_time();
                uint16_t *pixels = (uint16_t *)frame_ptr;

                // 1. Post-hit cooldown timer
                if (g_p4_cooldown_until_us > 0) {
                    if (now_us >= g_p4_cooldown_until_us) {
                        g_p4_cooldown_until_us = 0;
                        g_p4_state = P4_STATE_ARMED;
                        s_p4_baseline_samples = 0;
                        s_p4_confirm_frames = 0;
                        ESP_LOGI("P4_NERF", "🔄 Cooldown elapsed. Target ARMED for next shot.");
                    }
                }

                // 2. Target Settle Timer: wait 650ms after UP command to reject servo swing
                if (g_p4_state == P4_STATE_SETTLING) {
                    if (now_us - g_p4_state_start_us >= 650000ULL) {
                        g_p4_state = P4_STATE_CALIBRATING;
                        s_p4_baseline_samples = 0;
                        ESP_LOGI("P4_NERF", "Settle complete. Establishing clean black target baseline...");
                    }
                }

                // 3. Baseline Acquisition: Multi-frame average of stationary black target ROI
                if (g_p4_state == P4_STATE_CALIBRATING || s_p4_baseline_samples < 6) {
                    for (int gy = 0; gy < NERF_GRID_SIZE; gy++) {
                        int y = 140 + gy * 4;
                        int row = y * 800;
                        for (int gx = 0; gx < NERF_GRID_SIZE; gx++) {
                            int x = 140 + gx * 4;
                            s_p4_baseline_roi[gy][gx] = pixels[row + x];
                        }
                    }
                    s_p4_baseline_samples++;
                    if (s_p4_baseline_samples >= 6) {
                        g_p4_state = P4_STATE_ARMED;
                        ESP_LOGI("P4_NERF", "🎯 Clean black target baseline established! ARMED for orange/yellow impact.");
                    }
                }

                bool hit_confirmed = false;
                int best_x = 400;
                int best_y = 400;

                // 4. ARMED State: 4-Check Deterministic Event Detection on Optimal ROI
                if (g_p4_state == P4_STATE_ARMED) {
                    int diff_count = 0;
                    int color_count = 0;
                    uint32_t sum_x = 0;
                    uint32_t sum_y = 0;
                    int min_x = 800, max_x = 0, min_y = 800, max_y = 0;

                    // Scan optimal circular ROI (Center 400, 400, Radius 260px)
                    for (int gy = 0; gy < NERF_GRID_SIZE; gy++) {
                        int y = 140 + gy * 4;
                        int dy = y - 400;
                        int row = y * 800;

                        for (int gx = 0; gx < NERF_GRID_SIZE; gx++) {
                            int x = 140 + gx * 4;
                            int dx = x - 400;
                            if (dx * dx + dy * dy > 260 * 260) continue;

                            uint16_t p = pixels[row + x];
                            uint16_t base = s_p4_baseline_roi[gy][gx];

                            // Extract RGB components (0..255 range)
                            int r = ((p >> 11) & 0x1F) * 8;
                            int g = ((p >> 5) & 0x3F) * 4;
                            int b = (p & 0x1F) * 8;

                            int br = ((base >> 11) & 0x1F) * 8;
                            int bg = ((base >> 5) & 0x3F) * 4;
                            int bb = (base & 0x1F) * 8;

                            // Check 1: Baseline difference
                            int delta = (abs(r - br) + abs(g - bg) + abs(b - bb)) / 3;
                            bool base_was_dark = (br <= 110 && bg <= 110 && bb <= 110);

                            if (delta >= 22) diff_count++;

                            // Check 2: Nerf Orange/Yellow Chromatic Spectrum
                            bool is_nerf = (r >= 105 && b <= 75 && (r - b) >= 38 && 
                                            (r + g) >= ((b + 1) * 2) && (g - b) >= 10 && r >= (g - 28));

                            // Genuine projectile transition: dark baseline -> bright orange/yellow
                            if (is_nerf && delta >= 18 && base_was_dark) {
                                color_count++;
                                sum_x += x;
                                sum_y += y;
                                if (x < min_x) min_x = x;
                                if (x > max_x) max_x = x;
                                if (y < min_y) min_y = y;
                                if (y > max_y) max_y = y;
                            }
                        }
                    }

                    // Check 4: Spatial Coherence / Clustering
                    int bbox_w = (max_x >= min_x) ? (max_x - min_x + 1) : 0;
                    int bbox_h = (max_y >= min_y) ? (max_y - min_y + 1) : 0;
                    int area = (bbox_w / 4) * (bbox_h / 4);
                    float density = area > 0 ? ((float)color_count / area) : 0.0f;

                    // Multi-check confirmation: coherent blob of >= 18 grid points
                    if (color_count >= 18 && density >= 0.12f && bbox_w >= 16 && bbox_h >= 16) {
                        if (color_count >= 32 || s_p4_confirm_frames >= 1) {
                            hit_confirmed = true;
                            best_x = color_count > 0 ? (int)(sum_x / color_count) : 400;
                            best_y = color_count > 0 ? (int)(sum_y / color_count) : 400;

                            // DISPATCH IMMEDIATE REVERSE COMMAND TO SERVO!
                            const char *down_cmd = "DOWN,1\n";
                            uart_write_bytes(BRIDGE_UART_NUM, down_cmd, strlen(down_cmd));
                            uart_write_bytes(UART_NUM_0, down_cmd, strlen(down_cmd));

                            ESP_LOGI("P4_NERF", "🎯 NERF HIT CONFIRMED! Dispatched DOWN,1 (color: %d px, density: %.0f%%) -> Servo Reversed!",
                                     color_count, density * 100);

                            g_p4_state = P4_STATE_LOCKED;
                            g_p4_cooldown_until_us = now_us + 3000000ULL;
                            s_p4_confirm_frames = 0;
                        } else {
                            s_p4_confirm_frames++;
                        }
                    } else {
                        if (s_p4_confirm_frames > 0) s_p4_confirm_frames--;
                    }
                }

                // Encode RAW8 to JPEG (Grayscale)
                jpeg_encode_cfg_t enc_config = {
                    .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
                    .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
                    .image_quality = 35,
                    .width = 800,
                    .height = 800,
                };
                
                uint32_t jpeg_encoded_size = 0;
                esp_err_t ret = jpeg_encoder_process(g_jpeg_handle, &enc_config, frame_ptr, out_len, g_jpeg_out_buf, g_jpeg_out_size, &jpeg_encoded_size);
                
                if (ret == ESP_OK && jpeg_encoded_size > 0) {
                    // Verify JPEG integrity
                    bool jpeg_ok = (jpeg_encoded_size >= 4 &&
                                    g_jpeg_out_buf[0] == 0xFF &&
                                    g_jpeg_out_buf[1] == 0xD8);
                    
                    bool has_ffd9 = false;
                    if (jpeg_ok) {
                        for (int idx = (int)jpeg_encoded_size - 1; idx >= (int)jpeg_encoded_size - 10 && idx >= 0; idx--) {
                            if (idx > 0 && g_jpeg_out_buf[idx-1] == 0xFF && g_jpeg_out_buf[idx] == 0xD9) {
                                has_ffd9 = true;
                                break;
                            }
                        }
                    }

                    if (jpeg_ok) {
                        stat_encoded++;
                        stat_jpeg_bytes_total += jpeg_encoded_size;

                        if (!has_ffd9) {
                            ESP_LOGW(TAG, "JPEG: FF D9 marker not found in trailing 10 bytes of frame %d", frame_count);
                        }

                        // Inject Trailing 32-byte Metadata Steganographically
                        shot_metadata_t meta = {
                            .magic = {0xDE, 0xAD, 0xBE, 0xEF},
                            .laser_found = (hit_confirmed || g_p4_state == P4_STATE_LOCKED) ? 1 : 0,
                            .zone = 1,
                            .score_ring = 10,
                            .is_calibrated = (g_p4_state == P4_STATE_ARMED) ? 1 : 0,
                            .laser_x_px = (uint16_t)best_x,
                            .laser_y_px = (uint16_t)best_y,
                            .laser_x_mm = 25.0f,
                            .laser_y_mm = 25.0f,
                            .laser_dist_mm = 0.0f
                        };
                        
                        // Append metadata immediately after the JPEG data
                        memcpy(g_jpeg_out_buf + jpeg_encoded_size, &meta, sizeof(meta));
                        uint32_t total_payload_len = jpeg_encoded_size + sizeof(meta);

                        #if !CAMERA_TEST_MODE
                            #if SINGLE_FRAME_TEST_MODE
                                static bool single_frame_sent = false;
                                if (!single_frame_sent) {
                                    send_packet(frame_count, g_jpeg_out_buf, total_payload_len);
                                    single_frame_sent = true;
                                    stat_transmitted++;
                                    stat_tx_bytes_total += (15 + total_payload_len);
                                    ESP_LOGI(TAG, "SINGLE_FRAME_TEST_MODE: Sent first frame, stopping further transmission.");
                                }
                            #else
                                send_packet(frame_count, g_jpeg_out_buf, total_payload_len);
                                stat_transmitted++;
                                stat_tx_bytes_total += (15 + total_payload_len);
                            #endif
                        #else
                            // In Camera-only test mode, print captured size to USB serial without sending it over UART
                            printf("Frame %d | JPEG size: %lu\n", frame_count, jpeg_encoded_size);
                        #endif
                    } else {
                        ESP_LOGE(TAG, "JPEG check failed (size=%lu, start=%02X%02X)", 
                                 jpeg_encoded_size, g_jpeg_out_buf[0], g_jpeg_out_buf[1]);
                    }
                } else {
                    ESP_LOGE(TAG, "JPEG Encode failed: %s", esp_err_to_name(ret));
                }
            }

            // Re-queue the buffer back to camera driver
            ioctl(g_video_fd, VIDIOC_QBUF, &buf);
        }

        // Reset diagnostics counters every 2 seconds silently to prevent overflow
        int64_t now_stat = esp_timer_get_time();
        int64_t delta_us = now_stat - last_stat_time;
        if (delta_us >= 2000000) {
            stat_captured = 0;
            stat_encoded = 0;
            stat_jpeg_bytes_total = 0;
            stat_transmitted = 0;
            stat_tx_bytes_total = 0;
            last_stat_time = now_stat;
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Small yield
    }
#endif
#endif // !USB_TRANSPORT_TEST_MODE
}
#endif

void app_main(void)
{
    // Initialize UART0 at 3,000,000 baud and bind VFS console
    uart_config_t uart_cfg = {
        .baud_rate = 3000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 4096, 65536, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_cfg));
    uart_vfs_dev_use_driver(0);

    // Configure VFS UART0 to raw LF line endings (disable CRLF conversion)
    uart_vfs_dev_port_set_tx_line_endings(0, ESP_LINE_ENDINGS_LF);

    // Launch standalone target calibration receiver task
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL, 1);

    // Initialize USB-Serial-JTAG Driver
    init_usb_serial_jtag();

    // 1. Initialize UART Bridge
    init_bridge_uart();

#if !UART_DIAGNOSTIC_MODE
    // 2. Initialize Camera & Video Engine (ESP32-P4 v2.x API)
    esp_video_init_csi_config_t csi_config = {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = I2C_MASTER_NUM,
                .scl_pin = I2C_MASTER_SCL_IO,
                .sda_pin = I2C_MASTER_SDA_IO,
            },
            .freq = 100000,
        },
        .reset_pin = -1,
        .pwdn_pin = -1,
    };
    esp_video_init_config_t video_cfg = {
        .csi = &csi_config,
    };
    
    ESP_LOGI(TAG, "Initializing video subsystem...");
    ESP_ERROR_CHECK(esp_video_init(&video_cfg));

    g_video_fd = open("/dev/video0", O_RDWR);
    if (g_video_fd < 0) {
        ESP_LOGE(TAG, "Failed to open video hardware driver!");
        return;
    }
    ESP_LOGI(TAG, "OV5647 detected and initialized successfully.");

    // Configure OV5647 -> RGB565 @ 800x800 via Hardware ISP
    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix = {
            .width = 800,
            .height = 800,
            .pixelformat = V4L2_PIX_FMT_RGB565, // Hardware ISP converts Bayer to RGB565
        }
    };
    if (ioctl(g_video_fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "Failed to set format on /dev/video0");
    }

    // Initialize Hardware JPEG Encoder
    jpeg_encode_engine_cfg_t encode_eng_cfg = {
        .timeout_ms = 5000,
    };
    ESP_ERROR_CHECK(jpeg_new_encoder_engine(&encode_eng_cfg, &g_jpeg_handle));
    
    // Allocate buffer for JPEG output (max size = width * height)
    g_jpeg_out_size = 800 * 800;
    g_jpeg_out_buf = heap_caps_aligned_alloc(64, g_jpeg_out_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_jpeg_out_buf) {
        ESP_LOGE(TAG, "Failed to allocate JPEG buffer");
        return;
    }

    // Request & Queue Memory Buffers
    struct v4l2_requestbuffers req = {
        .count = 2,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    ioctl(g_video_fd, VIDIOC_REQBUFS, &req);

    for (int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i
        };
        ioctl(g_video_fd, VIDIOC_QUERYBUF, &buf);
        
        g_buffers[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, g_video_fd, buf.m.offset);
        g_buf_len[i] = buf.length;
        
        ioctl(g_video_fd, VIDIOC_QBUF, &buf);
    }

    // Start Streaming Engine Hardware
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(g_video_fd, VIDIOC_STREAMON, &type);
#endif

    // 3. Start the Capture or Diagnostic Task
#if UART_DIAGNOSTIC_MODE
    xTaskCreatePinnedToCore(uart_diagnostic_task, "diag_task", 4096, NULL, 5, NULL, 1);
#else
    xTaskCreatePinnedToCore(camera_stream_task, "cam_task", 8192, NULL, 5, NULL, 1);
#endif
}