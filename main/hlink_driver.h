/*
 * H-Link Driver Header
 * 
 * Handles UART communication with Hitachi HVAC device via H-Link protocol
 * Based on esphome-hlink-ac implementation
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* H-Link Protocol Constants */
#define HLINK_MODE_HEAT             0x0010
#define HLINK_MODE_HEAT_AUTO        0x8010
#define HLINK_MODE_COOL             0x0040
#define HLINK_MODE_COOL_AUTO        0x8040
#define HLINK_MODE_DRY              0x0020
#define HLINK_MODE_DRY_AUTO         0x8020
#define HLINK_MODE_FAN              0x0050
#define HLINK_MODE_AUTO             0x8000

/* H-Link Feature Addresses */
#define HLINK_FEATURE_POWER_STATE           0x0000
#define HLINK_FEATURE_MODE                  0x0001
#define HLINK_FEATURE_FAN_MODE              0x0002
#define HLINK_FEATURE_TARGET_TEMP           0x0003
#define HLINK_FEATURE_REMOTE_CONTROL_LOCK   0x0006
#define HLINK_FEATURE_CLEAN_FILTER_RESET    0x0007
#define HLINK_FEATURE_SWING_MODE            0x0014
#define HLINK_FEATURE_CURRENT_INDOOR_TEMP   0x0100
#define HLINK_FEATURE_CURRENT_OUTDOOR_TEMP  0x0102
#define HLINK_FEATURE_LEAVE_HOME_WRITE      0x0300
#define HLINK_FEATURE_ACTIVITY_STATUS       0x0301
#define HLINK_FEATURE_AIR_FILTER_WARNING    0x0302
#define HLINK_FEATURE_LEAVE_HOME_READ       0x0304
#define HLINK_FEATURE_BEEPER                0x0800
#define HLINK_FEATURE_MODEL_NAME            0x0900

/* Fan speeds */
#define HLINK_FAN_AUTO          0x00
#define HLINK_FAN_HIGH          0x01
#define HLINK_FAN_MEDIUM        0x02
#define HLINK_FAN_LOW           0x03
#define HLINK_FAN_QUIET         0x04

/* Swing modes */
#define HLINK_SWING_OFF         0x00
#define HLINK_SWING_VERTICAL    0x01
#define HLINK_SWING_HORIZONTAL  0x02
#define HLINK_SWING_BOTH        0x03

/* Activity status */
#define HLINK_ACTIVE_OFF        0x0000
#define HLINK_ACTIVE_ON         0xFFFF

/* Leave home (away preset) */
#define HLINK_DISABLE_LEAVE_HOME 0x0000
#define HLINK_ENABLE_LEAVE_HOME  0xFFFF

/* HVAC Mode enumeration (for Zigbee) */
typedef enum {
    HLINK_ZB_MODE_OFF = 0,
    HLINK_ZB_MODE_AUTO = 1,
    HLINK_ZB_MODE_COOL = 3,
    HLINK_ZB_MODE_HEAT = 4,
    HLINK_ZB_MODE_DRY = 8,
    HLINK_ZB_MODE_FAN_ONLY = 7
} hlink_zb_mode_t;

/* HVAC Fan speed enumeration (for Zigbee) */
typedef enum {
    HLINK_ZB_FAN_AUTO = 0,
    HLINK_ZB_FAN_HIGH = 1,
    HLINK_ZB_FAN_MEDIUM = 2,
    HLINK_ZB_FAN_LOW = 3,
    HLINK_ZB_FAN_QUIET = 4
} hlink_zb_fan_t;

/* HVAC Running State enumeration */
typedef enum {
    HLINK_RUNNING_IDLE = 0,
    HLINK_RUNNING_HEATING = 1,
    HLINK_RUNNING_COOLING = 2,
    HLINK_RUNNING_FAN_ONLY = 3,
    HLINK_RUNNING_DRYING = 4
} hlink_running_state_t;

/* HVAC State structure */
typedef struct {
    bool power_on;
    hlink_zb_mode_t mode;
    hlink_running_state_t running_state;
    uint16_t hlink_mode;                // Raw H-Link mode value
    float current_temperature;          // Indoor temperature (°C)
    float outdoor_temperature;          // Outdoor temperature (°C)
    float target_temperature;           // Target temperature (°C, 16-32)
    hlink_zb_fan_t fan_mode;
    uint8_t swing_mode;                 // 0=off, 1=vertical, 2=horizontal, 3=both
    bool remote_lock;                   // Remote control lock state
    bool beeper_enabled;                // Beeper on/off
    bool leave_home_enabled;            // Away preset
    bool filter_warning;                // Air filter needs cleaning
    bool activity_status;               // AC is actively heating/cooling
    char model_name[32];               // Model name from AC
} hlink_state_t;

/* UART Configuration */
#define HLINK_UART_NUM           UART_NUM_1
#define HLINK_UART_TX_PIN        1    // GPIO1 (TX) - via BSS138 level shifter to 5V H-Link
#define HLINK_UART_RX_PIN        2    // GPIO2 (RX) - via BSS138 level shifter from 5V H-Link
#define HLINK_UART_BAUD_RATE     9600
#define HLINK_UART_PARITY        UART_PARITY_ODD
#define HLINK_UART_STOP_BITS     UART_STOP_BITS_1
#define HLINK_UART_DATA_BITS     UART_DATA_8_BITS
#define HLINK_UART_BUF_SIZE      1024

/* Frame constants */
#define HLINK_MSG_READ_BUFFER_SIZE  64
#define HLINK_ASCII_CR              0x0D
#define HLINK_STATUS_UPDATE_INTERVAL_MS 5000  // Poll status every 5 seconds
#define MIN_INTERVAL_BETWEEN_REQUESTS 60     // Minimum 60ms between requests
#define HLINK_TIMEOUT_MS              1000   // Response timeout threshold

/* Response frame status */
typedef enum {
    HLINK_FRAME_NOTHING = 0,    // No data received
    HLINK_FRAME_PARTIAL = 1,    // Partial frame received
    HLINK_FRAME_OK = 2,         // Valid OK response
    HLINK_FRAME_NG = 3,         // NG error response
    HLINK_FRAME_INVALID = 4     // Invalid/corrupted frame
} hlink_frame_status_t;

/**
 * @brief Initialize H-Link UART driver
 * 
 * @return ESP_OK on success
 */
esp_err_t hlink_driver_init(void);

/**
 * @brief Get current H-Link state
 * 
 * @param state Pointer to state structure to fill
 * @return ESP_OK on success
 */
esp_err_t hlink_get_state(hlink_state_t *state);

/**
 * @brief Register callback for H-Link state changes
 * 
 * @param callback Function to call when state changes
 */
void hlink_register_state_change_callback(void (*callback)(void));

/**
 * @brief Set HVAC power state
 * 
 * @param power_on true to turn on, false to turn off
 * @return ESP_OK on success
 */
esp_err_t hlink_set_power(bool power_on);

/**
 * @brief Set HVAC mode
 * 
 * @param mode Operating mode (Zigbee format)
 * @return ESP_OK on success
 */
esp_err_t hlink_set_mode(hlink_zb_mode_t mode);

/**
 * @brief Set target temperature
 * 
 * @param temp_c Temperature in Celsius (16-32)
 * @return ESP_OK on success
 */
esp_err_t hlink_set_temperature(float temp_c);

/**
 * @brief Set fan mode
 * 
 * @param fan_mode Fan speed setting
 * @return ESP_OK on success
 */
esp_err_t hlink_set_fan_mode(hlink_zb_fan_t fan_mode);

/**
 * @brief Set swing mode
 * 
 * @param swing_mode 0=off, 1=vertical, 2=horizontal, 3=both
 * @return ESP_OK on success
 */
esp_err_t hlink_set_swing_mode(uint8_t swing_mode);

/**
 * @brief Set remote control lock
 * 
 * @param locked true to lock remote control
 * @return ESP_OK on success
 */
esp_err_t hlink_set_remote_lock(bool locked);

/**
 * @brief Set beeper state
 * 
 * @param enabled true to enable beeper
 * @return ESP_OK on success
 */
esp_err_t hlink_set_beeper(bool enabled);

/**
 * @brief Set leave home (away) mode
 * 
 * @param enabled true to enable leave home mode
 * @return ESP_OK on success
 */
esp_err_t hlink_set_leave_home(bool enabled);

/**
 * @brief Reset air filter cleaning warning
 * 
 * @return ESP_OK on success
 */
esp_err_t hlink_reset_filter_warning(void);

/**
 * @brief Request status update from AC unit
 * 
 * Polls all status features and updates internal state
 * 
 * @return ESP_OK on success
 */
esp_err_t hlink_request_status_update(void);

/**
 * @brief Read model name from AC unit
 * 
 * @return ESP_OK on success
 */
esp_err_t hlink_read_model_name(void);

/**
 * @brief Check if H-Link driver is ready
 * 
 * @return true if initialized and ready
 */
bool hlink_is_ready(void);

#ifdef __cplusplus
}
#endif
