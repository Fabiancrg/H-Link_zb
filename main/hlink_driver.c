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

/* Helper functions */
static uint16_t calculate_checksum(uint16_t address, const uint8_t *data, size_t data_len);
static esp_err_t hlink_send_mt_request(uint16_t address);
static esp_err_t hlink_send_st_request(uint16_t address, const uint8_t *data, size_t data_len);
static esp_err_t hlink_read_response(char *buf, size_t buf_size, uint32_t timeout_ms);
static esp_err_t parse_mt_response(const char *response, uint8_t *data, size_t *data_len);
static uint16_t parse_hex(const char *str, size_t len);
static uint16_t hlink_mode_to_raw(hlink_zb_mode_t mode);
static hlink_zb_mode_t hlink_raw_to_mode(uint16_t raw_mode);

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
 * @brief Send MT (read) request
 */
static esp_err_t hlink_send_mt_request(uint16_t address)
{
    char message[32];
    uint16_t checksum = calculate_checksum(address, NULL, 0);
    
    snprintf(message, sizeof(message), "MT P=%04X C=%04X\r", address, checksum);
    
    ESP_LOGD(TAG, "TX: %s", message);
    
    int written = uart_write_bytes(HLINK_UART_NUM, message, strlen(message));
    if (written < 0) {
        ESP_LOGE(TAG, "UART write failed");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * @brief Send ST (write) request
 */
static esp_err_t hlink_send_st_request(uint16_t address, const uint8_t *data, size_t data_len)
{
    char message[128];
    char data_hex[64] = {0};
    
    // Convert data to hex string
    for (size_t i = 0; i < data_len && i < 32; i++) {
        sprintf(&data_hex[i * 2], "%02X", data[i]);
    }
    
    uint16_t checksum = calculate_checksum(address, data, data_len);
    
    snprintf(message, sizeof(message), "ST P=%04X,%s C=%04X\r", address, data_hex, checksum);
    
    ESP_LOGD(TAG, "TX: %s", message);
    
    int written = uart_write_bytes(HLINK_UART_NUM, message, strlen(message));
    if (written < 0) {
        ESP_LOGE(TAG, "UART write failed");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * @brief Read response from UART
 */
static esp_err_t hlink_read_response(char *buf, size_t buf_size, uint32_t timeout_ms)
{
    s_response_index = 0;
    memset(s_response_buf, 0, sizeof(s_response_buf));
    
    uint32_t start_time = xTaskGetTickCount();
    uint32_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    
    while ((xTaskGetTickCount() - start_time) < timeout_ticks) {
        uint8_t byte;
        int len = uart_read_bytes(HLINK_UART_NUM, &byte, 1, pdMS_TO_TICKS(50));
        
        if (len > 0) {
            if (byte == HLINK_ASCII_CR) {
                // End of message
                s_response_buf[s_response_index] = '\0';
                strncpy(buf, s_response_buf, buf_size - 1);
                buf[buf_size - 1] = '\0';
                ESP_LOGD(TAG, "RX: %s", buf);
                return ESP_OK;
            }
            
            if (s_response_index < (HLINK_MSG_READ_BUFFER_SIZE - 1)) {
                s_response_buf[s_response_index++] = byte;
            }
        }
    }
    
    ESP_LOGW(TAG, "Response timeout");
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Parse MT response
 */
static esp_err_t parse_mt_response(const char *response, uint8_t *data, size_t *data_len)
{
    // Expected format: "OK P=XXXX C=YYYY" or "OK P=XXXX,DATA C=YYYY"
    if (strncmp(response, "OK ", 3) != 0) {
        ESP_LOGW(TAG, "Invalid response: %s", response);
        return ESP_FAIL;
    }
    
    // Find P= and extract data
    const char *p_pos = strstr(response, "P=");
    if (!p_pos) {
        return ESP_FAIL;
    }
    p_pos += 2; // Skip "P="
    
    // Check if there's data after the address
    const char *comma_pos = strchr(p_pos, ',');
    const char *space_pos = strchr(p_pos, ' ');
    
    if (comma_pos && comma_pos < space_pos) {
        // There's data - parse it
        const char *data_start = comma_pos + 1;
        const char *data_end = space_pos;
        size_t hex_len = data_end - data_start;
        
        if (hex_len > 0 && (hex_len % 2) == 0) {
            *data_len = hex_len / 2;
            for (size_t i = 0; i < *data_len; i++) {
                char hex_byte[3] = {data_start[i * 2], data_start[i * 2 + 1], '\0'};
                data[i] = (uint8_t)parse_hex(hex_byte, 2);
            }
        } else {
            *data_len = 0;
        }
    } else {
        // No data, just address
        *data_len = 0;
    }
    
    return ESP_OK;
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
    
    // Read power state
    if (hlink_send_mt_request(HLINK_FEATURE_POWER_STATE) == ESP_OK) {
        if (hlink_read_response(response, sizeof(response), 500) == ESP_OK) {
            if (parse_mt_response(response, data, &data_len) == ESP_OK && data_len >= 2) {
                uint16_t power = (data[0] << 8) | data[1];
                s_state.power_on = (power != 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Read mode
    if (hlink_send_mt_request(HLINK_FEATURE_MODE) == ESP_OK) {
        if (hlink_read_response(response, sizeof(response), 500) == ESP_OK) {
            if (parse_mt_response(response, data, &data_len) == ESP_OK && data_len >= 2) {
                s_state.hlink_mode = (data[0] << 8) | data[1];
                s_state.mode = hlink_raw_to_mode(s_state.hlink_mode);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Read current temperature
    if (hlink_send_mt_request(HLINK_FEATURE_CURRENT_INDOOR_TEMP) == ESP_OK) {
        if (hlink_read_response(response, sizeof(response), 500) == ESP_OK) {
            if (parse_mt_response(response, data, &data_len) == ESP_OK && data_len >= 2) {
                uint16_t temp = (data[0] << 8) | data[1];
                s_state.current_temperature = (float)temp;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Read target temperature
    if (hlink_send_mt_request(HLINK_FEATURE_TARGET_TEMP) == ESP_OK) {
        if (hlink_read_response(response, sizeof(response), 500) == ESP_OK) {
            if (parse_mt_response(response, data, &data_len) == ESP_OK && data_len >= 2) {
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
        if (hlink_read_response(response, sizeof(response), 500) == ESP_OK) {
            if (parse_mt_response(response, data, &data_len) == ESP_OK && data_len >= 2) {
                uint16_t fan = (data[0] << 8) | data[1];
                s_state.fan_mode = (hlink_zb_fan_t)(fan & 0xFF);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Read swing mode
    if (hlink_send_mt_request(HLINK_FEATURE_SWING_MODE) == ESP_OK) {
        if (hlink_read_response(response, sizeof(response), 500) == ESP_OK) {
            if (parse_mt_response(response, data, &data_len) == ESP_OK && data_len >= 2) {
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
        if (hlink_read_response(response, sizeof(response), 500) == ESP_OK) {
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
        if (hlink_read_response(response, sizeof(response), 500) == ESP_OK) {
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
        if (hlink_read_response(response, sizeof(response), 500) == ESP_OK) {
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
        if (hlink_read_response(response, sizeof(response), 500) == ESP_OK) {
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
        if (hlink_read_response(response, sizeof(response), 500) == ESP_OK) {
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
        if (hlink_read_response(response, sizeof(response), 500) == ESP_OK) {
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
        if (hlink_read_response(response, sizeof(response), 500) == ESP_OK) {
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
        if (hlink_read_response(response, sizeof(response), 500) == ESP_OK) {
            s_state.filter_warning = false;
        }
    }
    
    xSemaphoreGive(s_uart_mutex);
    return ret;
}

/**
 * @brief Check if driver is ready
 */
bool hlink_is_ready(void)
{
    return s_initialized;
}

/**
 * @brief Register state change callback
 */
void hlink_register_state_change_callback(void (*callback)(void))
{
    s_state_change_callback = callback;
}
