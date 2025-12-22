/*
 * H-Link Driver Implementation
 * 
 * Handles UART communication with Hitachi HVAC device via H-Link protocol
 * Protocol format:
 *   MT P=XXXX C=YYYY\r  (Read request)
 *   ST P=XXXX,DATA C=YYYY\r  (Write request)
 *   OK P=DATA C=YYYY\r  (Response)
 */

#include "hlink_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "string.h"
#include "stdio.h"

static const char *TAG = "HLINK_DRV";

/* Internal state */
static hlink_state_t s_state = {0};
static bool s_initialized = false;
static SemaphoreHandle_t s_uart_mutex = NULL;
static SemaphoreHandle_t s_state_mutex = NULL;
static void (*s_state_change_callback)(void) = NULL;

/* Response buffer */
static char s_response_buf[HLINK_MSG_READ_BUFFER_SIZE] = {0};
static uint8_t s_response_index = 0;

/* Timing tracking */
static uint32_t s_last_frame_sent_ms = 0;
static uint32_t s_timeout_counter_started_at_ms = 0;
static bool s_timeout_counter_active = false;

/* Helper functions */
static uint16_t calculate_checksum(uint16_t address, const uint8_t *data, size_t data_len);
static esp_err_t hlink_send_mt_request(uint16_t address);
static esp_err_t hlink_send_st_request(uint16_t address, const uint8_t *data, size_t data_len);
static hlink_frame_status_t hlink_read_response(char *buf, size_t buf_size, uint32_t timeout_ms);
static hlink_frame_status_t parse_mt_response(const char *response, uint8_t *data, size_t *data_len);
static uint16_t parse_hex(const char *str, size_t len);
static uint16_t hlink_mode_to_raw(hlink_zb_mode_t mode);
static hlink_zb_mode_t hlink_raw_to_mode(uint16_t raw_mode);
static bool reached_timeout_threshold(void);
static bool can_send_next_frame(void);
static uint32_t millis(void);

/**
 * @brief Calculate H-Link checksum
 */
static uint16_t calculate_checksum(uint16_t address, const uint8_t *data, size_t data_len)
{
    uint16_t checksum = 0xFFFF;
    checksum -= (address >> 8);
    checksum -= (address & 0xFF);
    
    if (data && data_len > 0) {
        for (size_t i = 0; i < data_len; i++) {
            checksum -= data[i];
        }
    }
    
    return checksum & 0xFFFF;
}

/**
 * @brief Get current time in milliseconds (FreeRTOS tick based)
 */
static uint32_t millis(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/**
 * @brief Check if timeout threshold has been reached
 */
static bool reached_timeout_threshold(void)
{
    if (!s_timeout_counter_active) {
        return false;
    }
    return (millis() - s_timeout_counter_started_at_ms) >= HLINK_TIMEOUT_MS;
}

/**
 * @brief Check if enough time has passed to send next frame
 */
static bool can_send_next_frame(void)
{
    if (s_last_frame_sent_ms == 0) {
        return true;  // First frame can be sent immediately
    }
    return (millis() - s_last_frame_sent_ms) >= MIN_INTERVAL_BETWEEN_REQUESTS;
}

/**
 * @brief Parse hexadecimal string to uint16_t
 */
static uint16_t parse_hex(const char *str, size_t len)
{
    uint16_t result = 0;
    for (size_t i = 0; i < len && str[i] != '\0'; i++) {
        result <<= 4;
        if (str[i] >= '0' && str[i] <= '9') {
            result |= (str[i] - '0');
        } else if (str[i] >= 'A' && str[i] <= 'F') {
            result |= (str[i] - 'A' + 10);
        } else if (str[i] >= 'a' && str[i] <= 'f') {
            result |= (str[i] - 'a' + 10);
        }
    }
    return result;
}

/**
 * @brief Convert Zigbee mode to H-Link raw mode
 */
static uint16_t hlink_mode_to_raw(hlink_zb_mode_t mode)
{
    switch (mode) {
        case HLINK_ZB_MODE_HEAT:        return HLINK_MODE_HEAT;
        case HLINK_ZB_MODE_COOL:        return HLINK_MODE_COOL;
        case HLINK_ZB_MODE_DRY:         return HLINK_MODE_DRY;
        case HLINK_ZB_MODE_FAN_ONLY:    return HLINK_MODE_FAN;
        case HLINK_ZB_MODE_AUTO:        return HLINK_MODE_AUTO;
        default:                        return HLINK_MODE_AUTO;
    }
}

/**
 * @brief Convert H-Link raw mode to Zigbee mode
 */
static hlink_zb_mode_t hlink_raw_to_mode(uint16_t raw_mode)
{
    switch (raw_mode) {
        case HLINK_MODE_HEAT:
        case HLINK_MODE_HEAT_AUTO:
            return HLINK_ZB_MODE_HEAT;
        case HLINK_MODE_COOL:
        case HLINK_MODE_COOL_AUTO:
            return HLINK_ZB_MODE_COOL;
        case HLINK_MODE_DRY:
        case HLINK_MODE_DRY_AUTO:
            return HLINK_ZB_MODE_DRY;
        case HLINK_MODE_FAN:
            return HLINK_ZB_MODE_FAN_ONLY;
        case HLINK_MODE_AUTO:
            return HLINK_ZB_MODE_AUTO;
        default:
            return HLINK_ZB_MODE_AUTO;
    }
}

/**
 * @brief Send MT (read) request with frame timing
 */
static esp_err_t hlink_send_mt_request(uint16_t address)
{
    // Wait for minimum interval between frames
    while (!can_send_next_frame()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    char message[32];
    uint16_t checksum = calculate_checksum(address, NULL, 0);
    
    snprintf(message, sizeof(message), "MT P=%04X C=%04X\r", address, checksum);
    
    ESP_LOGI(TAG, "[BUS] TX: %s (length: %d bytes)", message, (int)strlen(message));
    ESP_LOGD(TAG, "TX: %s", message);
    
    int written = uart_write_bytes(HLINK_UART_NUM, message, strlen(message));
    if (written < 0) {
        ESP_LOGE(TAG, "[BUS] UART write failed!");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "[BUS] Successfully wrote %d bytes to UART", written);
    
    // Update last frame sent timestamp
    s_last_frame_sent_ms = millis();
    
    return ESP_OK;
}

/**
 * @brief Send ST (write) request with frame timing
 */
static esp_err_t hlink_send_st_request(uint16_t address, const uint8_t *data, size_t data_len)
{
    // Wait for minimum interval between frames
    while (!can_send_next_frame()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    char message[128];
    char data_hex[64] = {0};
    
    // Convert data to hex string
    for (size_t i = 0; i < data_len && i < 32; i++) {
        sprintf(&data_hex[i * 2], "%02X", data[i]);
    }
    
    uint16_t checksum = calculate_checksum(address, data, data_len);
    
    snprintf(message, sizeof(message), "ST P=%04X,%s C=%04X\r", address, data_hex, checksum);
    
    ESP_LOGI(TAG, "[BUS] TX: %s (length: %d bytes)", message, (int)strlen(message));
    ESP_LOGD(TAG, "TX: %s", message);
    
    int written = uart_write_bytes(HLINK_UART_NUM, message, strlen(message));
    if (written < 0) {
        ESP_LOGE(TAG, "[BUS] UART write failed!");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "[BUS] Successfully wrote %d bytes to UART", written);
    
    // Update last frame sent timestamp
    s_last_frame_sent_ms = millis();
    
    return ESP_OK;
}

/**
 * @brief Read response from UART with enhanced error detection
 */
static hlink_frame_status_t hlink_read_response(char *buf, size_t buf_size, uint32_t timeout_ms)
{
    s_response_index = 0;
    memset(s_response_buf, 0, sizeof(s_response_buf));
    
    uint32_t start_time = millis();
    bool received_any_data = false;
    int bytes_received = 0;
    
    // Start timeout counter if not already active
    if (!s_timeout_counter_active) {
        s_timeout_counter_started_at_ms = start_time;
        s_timeout_counter_active = true;
    }
    
    ESP_LOGI(TAG, "[BUS] Waiting for response (timeout: %lu ms)...", timeout_ms);
    
    while ((millis() - start_time) < timeout_ms) {
        uint8_t byte;
        int len = uart_read_bytes(HLINK_UART_NUM, &byte, 1, pdMS_TO_TICKS(50));
        
        if (len > 0) {
            received_any_data = true;
            bytes_received++;
            
            // Log raw byte (show both hex and ASCII if printable)
            if (byte >= 0x20 && byte <= 0x7E) {
                ESP_LOGI(TAG, "[BUS] Byte %d: 0x%02X '%c'", bytes_received, byte, (char)byte);
            } else if (byte == HLINK_ASCII_CR) {
                ESP_LOGI(TAG, "[BUS] Byte %d: 0x%02X <CR> (end of frame)", bytes_received, byte);
            } else {
                ESP_LOGI(TAG, "[BUS] Byte %d: 0x%02X (non-printable)", bytes_received, byte);
            }
            
            if (byte == HLINK_ASCII_CR) {
                // End of message - complete frame received
                s_response_buf[s_response_index] = '\0';
                strncpy(buf, s_response_buf, buf_size - 1);
                buf[buf_size - 1] = '\0';
                ESP_LOGI(TAG, "[BUS] Complete frame received (%d bytes): %s", bytes_received, buf);
                ESP_LOGD(TAG, "RX: %s", buf);
                
                // Reset timeout counter on successful read
                s_timeout_counter_active = false;
                
                // Check if it's an OK or NG response
                if (strncmp(buf, "OK ", 3) == 0) {
                    return HLINK_FRAME_OK;
                } else if (strncmp(buf, "NG ", 3) == 0) {
                    ESP_LOGW(TAG, "NG response received");
                    return HLINK_FRAME_NG;
                } else {
                    ESP_LOGW(TAG, "Invalid frame format: %s", buf);
                    return HLINK_FRAME_INVALID;
                }
            }
            
            if (s_response_index < (HLINK_MSG_READ_BUFFER_SIZE - 1)) {
                s_response_buf[s_response_index++] = byte;
            } else {
                // Buffer overflow - invalid frame
                ESP_LOGW(TAG, "Response buffer overflow");
                s_timeout_counter_active = false;
                return HLINK_FRAME_INVALID;
            }
        }
        
        // Check for timeout threshold
        if (reached_timeout_threshold()) {
            ESP_LOGW(TAG, "Communication timeout threshold reached");
            s_timeout_counter_active = false;
            if (received_any_data) {
                return HLINK_FRAME_PARTIAL;
            }
            return HLINK_FRAME_NOTHING;
        }
    }
    
    // Timeout expired
    if (received_any_data) {
        ESP_LOGW(TAG, "[BUS] Response timeout after %lu ms (%d bytes received)", timeout_ms, bytes_received);
        if (s_response_index > 0) {
            // Partial frame received
            s_response_buf[s_response_index] = '\0';
            ESP_LOGW(TAG, "[BUS] Partial frame: %s", s_response_buf);
            return HLINK_FRAME_PARTIAL;
        }
    } else {
        ESP_LOGW(TAG, "[BUS] No data received on bus after %lu ms - check wiring!", timeout_ms);
    }
    
    return HLINK_FRAME_NOTHING;
}

/**
 * @brief Parse MT response with checksum validation
 */
static hlink_frame_status_t parse_mt_response(const char *response, uint8_t *data, size_t *data_len)
{
    // Expected format: "OK P=XXXX C=YYYY" or "OK P=XXXX,DATA C=YYYY"
    if (strncmp(response, "OK ", 3) != 0) {
        ESP_LOGW(TAG, "Invalid response prefix: %s", response);
        return HLINK_FRAME_INVALID;
    }
    
    // Find P= and extract address
    const char *p_pos = strstr(response, "P=");
    if (!p_pos) {
        ESP_LOGW(TAG, "Missing P= in response");
        return HLINK_FRAME_INVALID;
    }
    p_pos += 2; // Skip "P="
    
    // Extract address (4 hex digits)
    char addr_str[5] = {0};
    strncpy(addr_str, p_pos, 4);
    uint16_t address = parse_hex(addr_str, 4);
    
    // Check if there's data after the address
    const char *comma_pos = strchr(p_pos, ',');
    const char *space_pos = strchr(p_pos, ' ');
    
    uint8_t temp_data[32] = {0};
    size_t temp_data_len = 0;
    
    if (comma_pos && comma_pos < space_pos) {
        // There's data - parse it
        const char *data_start = comma_pos + 1;
        const char *data_end = space_pos;
        size_t hex_len = data_end - data_start;
        
        if (hex_len > 0 && (hex_len % 2) == 0) {
            temp_data_len = hex_len / 2;
            for (size_t i = 0; i < temp_data_len; i++) {
                char hex_byte[3] = {data_start[i * 2], data_start[i * 2 + 1], '\0'};
                temp_data[i] = (uint8_t)parse_hex(hex_byte, 2);
            }
        }
    }
    
    // Extract and validate checksum
    const char *c_pos = strstr(response, " C=");
    if (!c_pos) {
        ESP_LOGW(TAG, "Missing C= in response");
        return HLINK_FRAME_INVALID;
    }
    c_pos += 3; // Skip " C="
    
    char checksum_str[5] = {0};
    strncpy(checksum_str, c_pos, 4);
    uint16_t received_checksum = parse_hex(checksum_str, 4);
    
    // Calculate expected checksum (responses only checksum the P_VALUE data, not the address)
    uint16_t expected_checksum = 0xFFFF;
    for (size_t i = 0; i < temp_data_len; i++) {
        expected_checksum -= temp_data[i];
    }
    expected_checksum &= 0xFFFF;
    
    if (received_checksum != expected_checksum) {
        ESP_LOGW(TAG, "Checksum mismatch! Expected: 0x%04X, Received: 0x%04X", 
                 expected_checksum, received_checksum);
        return HLINK_FRAME_INVALID;
    }
    
    // Checksum valid - copy data to output
    *data_len = temp_data_len;
    if (temp_data_len > 0) {
        memcpy(data, temp_data, temp_data_len);
    }
    
    ESP_LOGD(TAG, "Parsed response: Addr=0x%04X, DataLen=%u, Checksum=0x%04X (valid)", 
             address, *data_len, received_checksum);
    
    return HLINK_FRAME_OK;
}

/**
 * @brief Initialize H-Link driver
 */
esp_err_t hlink_driver_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing H-Link driver...");
    
    // Create mutexes
    s_uart_mutex = xSemaphoreCreateMutex();
    s_state_mutex = xSemaphoreCreateMutex();
    
    if (!s_uart_mutex || !s_state_mutex) {
        ESP_LOGE(TAG, "Failed to create mutexes");
        return ESP_FAIL;
    }
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = HLINK_UART_BAUD_RATE,
        .data_bits = HLINK_UART_DATA_BITS,
        .parity = HLINK_UART_PARITY,
        .stop_bits = HLINK_UART_STOP_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_param_config(HLINK_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_set_pin(HLINK_UART_NUM, HLINK_UART_TX_PIN, HLINK_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_driver_install(HLINK_UART_NUM, HLINK_UART_BUF_SIZE, HLINK_UART_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize state
    memset(&s_state, 0, sizeof(s_state));
    s_state.power_on = false;
    s_state.mode = HLINK_ZB_MODE_AUTO;
    s_state.target_temperature = 22.0;
    s_state.fan_mode = HLINK_ZB_FAN_AUTO;
    s_state.swing_mode = HLINK_SWING_OFF;
    
    s_initialized = true;
    
    ESP_LOGI(TAG, "H-Link driver initialized successfully");
    ESP_LOGI(TAG, "  UART: %d, TX: GPIO%d, RX: GPIO%d, Baud: %d, Parity: ODD",
             HLINK_UART_NUM, HLINK_UART_TX_PIN, HLINK_UART_RX_PIN, HLINK_UART_BAUD_RATE);
    
    // Perform UART health check
    ESP_LOGI(TAG, "[BUS] Performing UART health check...");
    
    // Check if UART is functional
    size_t uart_buf_len = 0;
    uart_get_buffered_data_len(HLINK_UART_NUM, &uart_buf_len);
    ESP_LOGI(TAG, "[BUS] UART RX buffer status: %d bytes", (int)uart_buf_len);
    
    // Flush any existing data
    if (uart_buf_len > 0) {
        ESP_LOGW(TAG, "[BUS] Flushing %d bytes from RX buffer", (int)uart_buf_len);
        uart_flush_input(HLINK_UART_NUM);
    }
    
    // Monitor bus for 2 seconds to see if there's any activity
    ESP_LOGI(TAG, "[BUS] Monitoring bus for 2 seconds to detect any activity...");
    uint32_t start = millis();
    int bytes_seen = 0;
    bool bus_active = false;
    
    while ((millis() - start) < 2000) {
        uint8_t byte;
        int len = uart_read_bytes(HLINK_UART_NUM, &byte, 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            bytes_seen++;
            bus_active = true;
            if (byte >= 0x20 && byte <= 0x7E) {
                ESP_LOGI(TAG, "[BUS] Detected byte: 0x%02X '%c'", byte, (char)byte);
            } else {
                ESP_LOGI(TAG, "[BUS] Detected byte: 0x%02X", byte);
            }
        }
    }
    
    if (bus_active) {
        ESP_LOGI(TAG, "[BUS] Bus is ACTIVE - detected %d bytes in 2 seconds", bytes_seen);
        ESP_LOGI(TAG, "[BUS] H-Link device appears to be transmitting");
    } else {
        ESP_LOGW(TAG, "[BUS] Bus is SILENT - no activity detected in 2 seconds");
        ESP_LOGW(TAG, "[BUS] Possible issues:");
        ESP_LOGW(TAG, "[BUS]   1. HVAC unit is not connected");
        ESP_LOGW(TAG, "[BUS]   2. Wrong TX/RX pin configuration");
        ESP_LOGW(TAG, "[BUS]   3. Baud rate mismatch (should be 9600)");
        ESP_LOGW(TAG, "[BUS]   4. Wiring issue (check connections)");
        ESP_LOGW(TAG, "[BUS]   5. HVAC unit is powered off");
    }
    
    ESP_LOGI(TAG, "[BUS] Health check complete - driver ready");
    
    return ESP_OK;
}

/**
 * @brief Get current H-Link state
 */
esp_err_t hlink_get_state(hlink_state_t *state)
{
    if (!s_initialized || !state) {
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(state, &s_state, sizeof(hlink_state_t));
        xSemaphoreGive(s_state_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Request status update from AC unit
 */
esp_err_t hlink_request_status_update(void)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    char response[HLINK_MSG_READ_BUFFER_SIZE];
    uint8_t data[32];
    size_t data_len;
    hlink_frame_status_t status;
    
    // Read power state
    if (hlink_send_mt_request(HLINK_FEATURE_POWER_STATE) == ESP_OK) {
        status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            if (parse_mt_response(response, data, &data_len) == HLINK_FRAME_OK && data_len >= 2) {
                uint16_t power = (data[0] << 8) | data[1];
                s_state.power_on = (power != 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Read mode
    if (hlink_send_mt_request(HLINK_FEATURE_MODE) == ESP_OK) {
        status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            if (parse_mt_response(response, data, &data_len) == HLINK_FRAME_OK && data_len >= 2) {
                s_state.hlink_mode = (data[0] << 8) | data[1];
                s_state.mode = hlink_raw_to_mode(s_state.hlink_mode);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Read current temperature
    if (hlink_send_mt_request(HLINK_FEATURE_CURRENT_INDOOR_TEMP) == ESP_OK) {
        status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            if (parse_mt_response(response, data, &data_len) == HLINK_FRAME_OK && data_len >= 2) {
                uint16_t temp = (data[0] << 8) | data[1];
                s_state.current_temperature = (float)temp;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Read target temperature
    if (hlink_send_mt_request(HLINK_FEATURE_TARGET_TEMP) == ESP_OK) {
        status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            if (parse_mt_response(response, data, &data_len) == HLINK_FRAME_OK && data_len >= 2) {
                uint16_t temp = (data[0] << 8) | data[1];
                if (temp >= 16 && temp <= 32) {
                    s_state.target_temperature = (float)temp;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Read fan mode
    if (hlink_send_mt_request(HLINK_FEATURE_FAN_MODE) == ESP_OK) {
        status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            if (parse_mt_response(response, data, &data_len) == HLINK_FRAME_OK && data_len >= 2) {
                uint16_t fan = (data[0] << 8) | data[1];
                s_state.fan_mode = (hlink_zb_fan_t)(fan & 0xFF);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Read swing mode
    if (hlink_send_mt_request(HLINK_FEATURE_SWING_MODE) == ESP_OK) {
        status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            if (parse_mt_response(response, data, &data_len) == HLINK_FRAME_OK && data_len >= 2) {
                uint16_t swing = (data[0] << 8) | data[1];
                s_state.swing_mode = swing & 0xFF;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    xSemaphoreGive(s_uart_mutex);
    
    ESP_LOGI(TAG, "Status: power=%d, mode=%d, temp=%.1f/%.1f, fan=%d, swing=%d",
             s_state.power_on, s_state.mode, s_state.current_temperature,
             s_state.target_temperature, s_state.fan_mode, s_state.swing_mode);
    
    return ESP_OK;
}

/**
 * @brief Set power state
 */
esp_err_t hlink_set_power(bool power_on)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    uint8_t data[2] = {0x00, power_on ? 0x01 : 0x00};
    esp_err_t ret = hlink_send_st_request(HLINK_FEATURE_POWER_STATE, data, 2);
    
    if (ret == ESP_OK) {
        char response[HLINK_MSG_READ_BUFFER_SIZE];
        hlink_frame_status_t status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            s_state.power_on = power_on;
        }
    }
    
    xSemaphoreGive(s_uart_mutex);
    return ret;
}

/**
 * @brief Set mode
 */
esp_err_t hlink_set_mode(hlink_zb_mode_t mode)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    uint16_t raw_mode = hlink_mode_to_raw(mode);
    uint8_t data[2] = {(raw_mode >> 8) & 0xFF, raw_mode & 0xFF};
    esp_err_t ret = hlink_send_st_request(HLINK_FEATURE_MODE, data, 2);
    
    if (ret == ESP_OK) {
        char response[HLINK_MSG_READ_BUFFER_SIZE];
        hlink_frame_status_t status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            s_state.mode = mode;
            s_state.hlink_mode = raw_mode;
        }
    }
    
    xSemaphoreGive(s_uart_mutex);
    return ret;
}

/**
 * @brief Set temperature
 */
esp_err_t hlink_set_temperature(float temp_c)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    
    if (temp_c < 16.0 || temp_c > 32.0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    uint16_t temp = (uint16_t)temp_c;
    uint8_t data[2] = {(temp >> 8) & 0xFF, temp & 0xFF};
    esp_err_t ret = hlink_send_st_request(HLINK_FEATURE_TARGET_TEMP, data, 2);
    
    if (ret == ESP_OK) {
        char response[HLINK_MSG_READ_BUFFER_SIZE];
        hlink_frame_status_t status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            s_state.target_temperature = temp_c;
        }
    }
    
    xSemaphoreGive(s_uart_mutex);
    return ret;
}

/**
 * @brief Set fan mode
 */
esp_err_t hlink_set_fan_mode(hlink_zb_fan_t fan_mode)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    uint8_t data[2] = {0x00, (uint8_t)fan_mode};
    esp_err_t ret = hlink_send_st_request(HLINK_FEATURE_FAN_MODE, data, 2);
    
    if (ret == ESP_OK) {
        char response[HLINK_MSG_READ_BUFFER_SIZE];
        hlink_frame_status_t status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            s_state.fan_mode = fan_mode;
        }
    }
    
    xSemaphoreGive(s_uart_mutex);
    return ret;
}

/**
 * @brief Set swing mode
 */
esp_err_t hlink_set_swing_mode(uint8_t swing_mode)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    
    if (swing_mode > HLINK_SWING_BOTH) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    uint8_t data[2] = {0x00, swing_mode};
    esp_err_t ret = hlink_send_st_request(HLINK_FEATURE_SWING_MODE, data, 2);
    
    if (ret == ESP_OK) {
        char response[HLINK_MSG_READ_BUFFER_SIZE];
        hlink_frame_status_t status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            s_state.swing_mode = swing_mode;
        }
    }
    
    xSemaphoreGive(s_uart_mutex);
    return ret;
}

/**
 * @brief Set remote control lock
 */
esp_err_t hlink_set_remote_lock(bool locked)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    uint8_t data[2] = {0x00, locked ? 0x01 : 0x00};
    esp_err_t ret = hlink_send_st_request(HLINK_FEATURE_REMOTE_CONTROL_LOCK, data, 2);
    
    if (ret == ESP_OK) {
        char response[HLINK_MSG_READ_BUFFER_SIZE];
        hlink_frame_status_t status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            s_state.remote_lock = locked;
        }
    }
    
    xSemaphoreGive(s_uart_mutex);
    return ret;
}

/**
 * @brief Set beeper
 */
esp_err_t hlink_set_beeper(bool enabled)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // Beeper is a trigger - writing any value triggers a beep
    uint8_t data[2] = {0x00, 0x01};
    esp_err_t ret = hlink_send_st_request(HLINK_FEATURE_BEEPER, data, 2);
    
    if (ret == ESP_OK) {
        char response[HLINK_MSG_READ_BUFFER_SIZE];
        hlink_read_response(response, sizeof(response), 500);
    }
    
    xSemaphoreGive(s_uart_mutex);
    return ret;
}

/**
 * @brief Set leave home mode
 */
esp_err_t hlink_set_leave_home(bool enabled)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    uint16_t value = enabled ? HLINK_ENABLE_LEAVE_HOME : HLINK_DISABLE_LEAVE_HOME;
    uint8_t data[2] = {(value >> 8) & 0xFF, value & 0xFF};
    esp_err_t ret = hlink_send_st_request(HLINK_FEATURE_LEAVE_HOME_WRITE, data, 2);
    
    if (ret == ESP_OK) {
        char response[HLINK_MSG_READ_BUFFER_SIZE];
        hlink_frame_status_t status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            s_state.leave_home_enabled = enabled;
        }
    }
    
    xSemaphoreGive(s_uart_mutex);
    return ret;
}

/**
 * @brief Reset filter warning
 */
esp_err_t hlink_reset_filter_warning(void)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    uint8_t data[2] = {0x00, 0x01};
    esp_err_t ret = hlink_send_st_request(HLINK_FEATURE_CLEAN_FILTER_RESET, data, 2);
    
    if (ret == ESP_OK) {
        char response[HLINK_MSG_READ_BUFFER_SIZE];
        hlink_frame_status_t status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            s_state.filter_warning = false;
        }
    }
    
    xSemaphoreGive(s_uart_mutex);
    return ret;
}

/**
 * @brief Read model name from AC unit
 */
esp_err_t hlink_read_model_name(void)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    char response[HLINK_MSG_READ_BUFFER_SIZE];
    uint8_t data[32];
    size_t data_len;
    hlink_frame_status_t status;
    
    // Read model name from feature 0x0900
    if (hlink_send_mt_request(HLINK_FEATURE_MODEL_NAME) == ESP_OK) {
        status = hlink_read_response(response, sizeof(response), 500);
        if (status == HLINK_FRAME_OK) {
            if (parse_mt_response(response, data, &data_len) == HLINK_FRAME_OK) {
                // Copy model name data (ASCII string)
                size_t copy_len = (data_len < sizeof(s_state.model_name) - 1) ? 
                                  data_len : sizeof(s_state.model_name) - 1;
                memcpy(s_state.model_name, data, copy_len);
                s_state.model_name[copy_len] = '\0';
                
                ESP_LOGI(TAG, "Model name: %s", s_state.model_name);
                xSemaphoreGive(s_uart_mutex);
                return ESP_OK;
            }
        }
    }
    
    xSemaphoreGive(s_uart_mutex);
    ESP_LOGW(TAG, "Failed to read model name");
    return ESP_FAIL;
}

/**
 * @brief Check if driver is initialized and ready
 */
bool hlink_is_ready(void)
{
    return s_initialized;
}

/**
 * @brief Run comprehensive bus diagnostics
 */
void hlink_bus_diagnostics(uint8_t duration_sec)
{
    if (duration_sec < 1) duration_sec = 1;
    if (duration_sec > 30) duration_sec = 30;
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "H-LINK BUS DIAGNOSTICS");
    ESP_LOGI(TAG, "========================================");
    
    // 1. UART Configuration
    ESP_LOGI(TAG, "[1] UART Configuration:");
    ESP_LOGI(TAG, "    Port: UART_%d", HLINK_UART_NUM);
    ESP_LOGI(TAG, "    TX Pin: GPIO%d", HLINK_UART_TX_PIN);
    ESP_LOGI(TAG, "    RX Pin: GPIO%d", HLINK_UART_RX_PIN);
    ESP_LOGI(TAG, "    Baud Rate: %d", HLINK_UART_BAUD_RATE);
    ESP_LOGI(TAG, "    Parity: ODD");
    ESP_LOGI(TAG, "    Data Bits: 8");
    ESP_LOGI(TAG, "    Stop Bits: 1");
    
    // 2. UART Buffer Status
    size_t uart_buf_len = 0;
    uart_get_buffered_data_len(HLINK_UART_NUM, &uart_buf_len);
    ESP_LOGI(TAG, "[2] UART Buffer Status:");
    ESP_LOGI(TAG, "    RX buffer: %d bytes", (int)uart_buf_len);
    
    if (uart_buf_len > 0) {
        ESP_LOGW(TAG, "    WARNING: RX buffer not empty! Flushing...");
        uart_flush_input(HLINK_UART_NUM);
    }
    
    // 3. Bus Activity Monitor
    ESP_LOGI(TAG, "[3] Bus Activity Monitor:");
    ESP_LOGI(TAG, "    Monitoring for %d seconds...", duration_sec);
    
    uint32_t start = millis();
    int total_bytes = 0;
    int printable_chars = 0;
    int cr_chars = 0;
    bool saw_mt = false, saw_st = false, saw_ok = false, saw_ng = false;
    char partial_msg[64] = {0};
    int partial_idx = 0;
    
    while ((millis() - start) < (duration_sec * 1000)) {
        uint8_t byte;
        int len = uart_read_bytes(HLINK_UART_NUM, &byte, 1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            total_bytes++;
            
            // Build partial message
            if (byte == HLINK_ASCII_CR) {
                partial_msg[partial_idx] = '\0';
                ESP_LOGI(TAG, "    Frame: %s", partial_msg);
                partial_idx = 0;
                cr_chars++;
            } else if (partial_idx < 63) {
                partial_msg[partial_idx++] = byte;
            }
            
            // Analyze byte
            if (byte >= 0x20 && byte <= 0x7E) {
                printable_chars++;
                
                // Check for protocol keywords
                if (partial_idx >= 2) {
                    if (partial_msg[partial_idx-2] == 'M' && partial_msg[partial_idx-1] == 'T') saw_mt = true;
                    if (partial_msg[partial_idx-2] == 'S' && partial_msg[partial_idx-1] == 'T') saw_st = true;
                    if (partial_msg[partial_idx-2] == 'O' && partial_msg[partial_idx-1] == 'K') saw_ok = true;
                    if (partial_msg[partial_idx-2] == 'N' && partial_msg[partial_idx-1] == 'G') saw_ng = true;
                }
            }
        }
    }
    
    // 4. Analysis Results
    ESP_LOGI(TAG, "[4] Analysis Results:");
    ESP_LOGI(TAG, "    Total bytes received: %d", total_bytes);
    ESP_LOGI(TAG, "    Printable characters: %d", printable_chars);
    ESP_LOGI(TAG, "    Frame terminators (CR): %d", cr_chars);
    ESP_LOGI(TAG, "    Complete frames: ~%d", cr_chars);
    
    if (total_bytes == 0) {
        ESP_LOGE(TAG, "    ❌ NO ACTIVITY DETECTED!");
        ESP_LOGE(TAG, "    ");
        ESP_LOGE(TAG, "    Possible causes:");
        ESP_LOGE(TAG, "    1. HVAC unit is powered OFF");
        ESP_LOGE(TAG, "    2. H-Link cable not connected");
        ESP_LOGE(TAG, "    3. Wrong TX/RX pins (check GPIO%d and GPIO%d)", HLINK_UART_TX_PIN, HLINK_UART_RX_PIN);
        ESP_LOGE(TAG, "    4. Level shifter not working (BSS138/Si2323DS)");
        ESP_LOGE(TAG, "    5. Wrong UART port (should be UART_%d)", HLINK_UART_NUM);
        ESP_LOGE(TAG, "    6. Baud rate mismatch (should be 9600)");
    } else if (printable_chars < (total_bytes / 2)) {
        ESP_LOGW(TAG, "    ⚠️  Mostly non-printable data - possible issues:");
        ESP_LOGW(TAG, "    1. Baud rate mismatch");
        ESP_LOGW(TAG, "    2. Parity setting incorrect");
        ESP_LOGW(TAG, "    3. Electrical noise on line");
    } else {
        ESP_LOGI(TAG, "    ✓ Bus activity detected");
        
        if (saw_mt || saw_st) {
            ESP_LOGI(TAG, "    ✓ Detected MT/ST commands (MASTER present)");
        }
        if (saw_ok || saw_ng) {
            ESP_LOGI(TAG, "    ✓ Detected OK/NG responses (SLAVE responding)");
        }
        
        if (!saw_mt && !saw_st && !saw_ok && !saw_ng) {
            ESP_LOGW(TAG, "    ⚠️  No H-Link protocol frames detected");
            ESP_LOGW(TAG, "    Data present but not H-Link format");
        }
    }
    
    // 5. Recommendations
    ESP_LOGI(TAG, "[5] Recommendations:");
    if (total_bytes == 0) {
        ESP_LOGI(TAG, "    → Check HVAC power and cable connections first");
        ESP_LOGI(TAG, "    → Verify TX=%d and RX=%d pins are correct", HLINK_UART_TX_PIN, HLINK_UART_RX_PIN);
        ESP_LOGI(TAG, "    → Test level shifter with multimeter (should see 5V)");
    } else if (total_bytes > 0 && cr_chars > 0) {
        ESP_LOGI(TAG, "    ✓ Communication appears functional");
        ESP_LOGI(TAG, "    → Try sending a test command (power toggle)");
    } else {
        ESP_LOGI(TAG, "    → Check baud rate setting (must be 9600)");
        ESP_LOGI(TAG, "    → Verify parity is set to ODD");
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "DIAGNOSTICS COMPLETE");
    ESP_LOGI(TAG, "========================================");
}

/**
 * @brief Probe GPIO pin levels to diagnose connection issues
 */
void hlink_probe_gpio_levels(uint8_t duration_sec)
{
    if (duration_sec < 1) duration_sec = 1;
    if (duration_sec > 10) duration_sec = 10;
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "GPIO LEVEL PROBE");
    ESP_LOGI(TAG, "========================================");
    
    ESP_LOGI(TAG, "[INFO] Probing GPIO pins for %d seconds...", duration_sec);
    ESP_LOGI(TAG, "[INFO] This shows raw pin states (HIGH=1, LOW=0)");
    ESP_LOGI(TAG, "");
    
    // Configure RX pin as input to read levels
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HLINK_UART_RX_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    uint32_t start = millis();
    int high_count = 0;
    int low_count = 0;
    int transition_count = 0;
    int last_level = -1;
    int samples = 0;
    int consecutive_high = 0;
    int consecutive_low = 0;
    int max_consecutive_high = 0;
    int max_consecutive_low = 0;
    
    ESP_LOGI(TAG, "[PROBE] Starting GPIO%d (RX) level monitoring...", HLINK_UART_RX_PIN);
    ESP_LOGI(TAG, "[PROBE] Sample every 10ms for %d seconds", duration_sec);
    ESP_LOGI(TAG, "");
    
    while ((millis() - start) < (duration_sec * 1000)) {
        int level = gpio_get_level(HLINK_UART_RX_PIN);
        samples++;
        
        if (level == 1) {
            high_count++;
            consecutive_high++;
            consecutive_low = 0;
            if (consecutive_high > max_consecutive_high) {
                max_consecutive_high = consecutive_high;
            }
        } else {
            low_count++;
            consecutive_low++;
            consecutive_high = 0;
            if (consecutive_low > max_consecutive_low) {
                max_consecutive_low = consecutive_low;
            }
        }
        
        // Detect transitions
        if (last_level >= 0 && level != last_level) {
            transition_count++;
            
            // Log significant transitions (every 10th)
            if (transition_count % 10 == 0) {
                ESP_LOGI(TAG, "[PROBE] Transition #%d: %d->%d (at %lu ms)",
                         transition_count, last_level, level, millis() - start);
            }
        }
        
        last_level = level;
        vTaskDelay(pdMS_TO_TICKS(10));  // Sample every 10ms
    }
    
    // Re-initialize UART after probing
    uart_driver_delete(HLINK_UART_NUM);
    uart_config_t uart_config = {
        .baud_rate = HLINK_UART_BAUD_RATE,
        .data_bits = HLINK_UART_DATA_BITS,
        .parity = HLINK_UART_PARITY,
        .stop_bits = HLINK_UART_STOP_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(HLINK_UART_NUM, &uart_config);
    uart_set_pin(HLINK_UART_NUM, HLINK_UART_TX_PIN, HLINK_UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(HLINK_UART_NUM, HLINK_UART_BUF_SIZE, HLINK_UART_BUF_SIZE, 0, NULL, 0);
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "[RESULTS] GPIO Level Analysis:");
    ESP_LOGI(TAG, "    Total samples: %d", samples);
    ESP_LOGI(TAG, "    HIGH count: %d (%.1f%%)", high_count, (float)high_count * 100.0f / samples);
    ESP_LOGI(TAG, "    LOW count: %d (%.1f%%)", low_count, (float)low_count * 100.0f / samples);
    ESP_LOGI(TAG, "    Transitions: %d", transition_count);
    ESP_LOGI(TAG, "    Max consecutive HIGH: %d samples (~%d ms)", max_consecutive_high, max_consecutive_high * 10);
    ESP_LOGI(TAG, "    Max consecutive LOW: %d samples (~%d ms)", max_consecutive_low, max_consecutive_low * 10);
    ESP_LOGI(TAG, "");
    
    // Analyze results
    if (transition_count == 0) {
        if (high_count == samples) {
            ESP_LOGE(TAG, "[RESULT] ❌ Pin stuck at HIGH (3.3V)");
            ESP_LOGE(TAG, "    Possible causes:");
            ESP_LOGE(TAG, "    1. Nothing connected (floating with internal pull-up)");
            ESP_LOGE(TAG, "    2. Level shifter output stuck HIGH");
            ESP_LOGE(TAG, "    3. H-Link DATA line idle (normal idle state is HIGH)");
            ESP_LOGW(TAG, "    NOTE: If HVAC is OFF, HIGH is expected!");
        } else if (low_count == samples) {
            ESP_LOGE(TAG, "[RESULT] ❌ Pin stuck at LOW (0V)");
            ESP_LOGE(TAG, "    Possible causes:");
            ESP_LOGE(TAG, "    1. Short to ground");
            ESP_LOGE(TAG, "    2. Level shifter not powered");
            ESP_LOGE(TAG, "    3. Wrong pin connected");
        }
    } else if (transition_count < 10) {
        ESP_LOGW(TAG, "[RESULT] ⚠️  Very few transitions detected");
        ESP_LOGW(TAG, "    Pin appears mostly static");
        ESP_LOGW(TAG, "    Possible:");
        ESP_LOGW(TAG, "    1. HVAC is idle (no communication)");
        ESP_LOGW(TAG, "    2. Very slow data rate");
        ESP_LOGW(TAG, "    3. Electrical noise causing occasional glitches");
    } else if (transition_count > 100) {
        ESP_LOGI(TAG, "[RESULT] ✓ Active signal detected!");
        ESP_LOGI(TAG, "    Pin shows %d transitions in %d seconds", transition_count, duration_sec);
        ESP_LOGI(TAG, "    This indicates:");
        ESP_LOGI(TAG, "    ✓ Physical connection is working");
        ESP_LOGI(TAG, "    ✓ Level shifter is passing signals");
        ESP_LOGI(TAG, "    ✓ H-Link device is transmitting");
        ESP_LOGW(TAG, "");
        ESP_LOGW(TAG, "    But UART received no data - possible issues:");
        ESP_LOGW(TAG, "    1. Wrong baud rate (should be 9600)");
        ESP_LOGW(TAG, "    2. Wrong parity setting (should be ODD)");
        ESP_LOGW(TAG, "    3. Inverted signal (level shifter problem)");
        ESP_LOGW(TAG, "    4. Voltage levels too low for UART detection");
    } else {
        ESP_LOGI(TAG, "[RESULT] ⚠️  Some activity detected");
        ESP_LOGI(TAG, "    %d transitions suggests possible data", transition_count);
        ESP_LOGI(TAG, "    But activity level seems low for active H-Link");
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "[RECOMMENDATION] Next steps:");
    if (transition_count == 0 && high_count == samples) {
        ESP_LOGI(TAG, "    1. Turn ON the HVAC unit if it's OFF");
        ESP_LOGI(TAG, "    2. Check if H-Link DATA line is connected");
        ESP_LOGI(TAG, "    3. Verify level shifter is powered (measure 3.3V and 5V)");
    } else if (transition_count == 0 && low_count == samples) {
        ESP_LOGI(TAG, "    1. Check ground connection");
        ESP_LOGI(TAG, "    2. Verify level shifter power supply");
        ESP_LOGI(TAG, "    3. Test with multimeter: should see 3-5V on H-Link DATA");
    } else if (transition_count > 100) {
        ESP_LOGI(TAG, "    1. Verify baud rate is 9600 (check your HVAC manual)");
        ESP_LOGI(TAG, "    2. Try different parity: NONE, EVEN, ODD");
        ESP_LOGI(TAG, "    3. Check if signal is inverted (may need inverting level shifter)");
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "GPIO PROBE COMPLETE");
    ESP_LOGI(TAG, "========================================");
}

/**
 * @brief Register state change callback
 */
void hlink_register_state_change_callback(void (*callback)(void))
{
    s_state_change_callback = callback;
}
