# H-Link Zigbee Bridge for Hitachi HVAC

Zigbee bridge for controlling Hitachi HVAC units via H-Link protocol using ESP32-C6.

## Overview

This project creates a Zigbee router device that bridges H-Link capable Hitachi HVAC units to Zigbee networks (Zigbee2MQTT, Home Assistant ZHA, etc.). It's based on the ESP-Zigbee SDK and implements the H-Link serial protocol for communication with the AC unit.

## Hardware Requirements

- **ESP32-C6** microcontroller (e.g., XIAO ESP32-C6, ESP32-C6-DevKitC)
- **H-Link compatible Hitachi HVAC** unit
- Connection to H-Link port on the AC unit (typically via SPX-WFG adapter port or direct H-Link connection)

## Pin Configuration

Default UART pins (configurable in `hlink_driver.h`):
- **TX**: GPIO 20
- **RX**: GPIO 21
- **GND**: Common ground with AC unit

## Zigbee Endpoints

### Endpoint 1: Thermostat (Main HVAC Control)
- **Cluster**: hvacThermostat (0x0201), hvacFanCtrl (0x0202)
- **Features**:
  - System mode (Off, Auto, Cool, Heat, Dry, Fan Only)
  - Target temperature (16-32°C)
  - Current temperature
  - Fan mode (Auto, Low, Medium, High, Quiet)
  - Running state (Idle, Heating, Cooling, Fan Only, Drying)

### Endpoint 2: Swing Mode
- **Cluster**: On/Off (0x0006)
- **Function**: Control air flow swing (Off, Vertical, Horizontal, Both)

### Endpoint 3: Remote Control Lock
- **Cluster**: On/Off (0x0006)
- **Function**: Lock/unlock the physical remote control

### Endpoint 4: Beeper
- **Cluster**: On/Off (0x0006)
- **Function**: Trigger beep sound on AC unit

### Endpoint 5: Leave Home (Away Preset)
- **Cluster**: On/Off (0x0006)
- **Function**: Enable/disable away preset (sets heating to 10°C)

### Endpoint 6: Air Filter Warning
- **Cluster**: On/Off (0x0006)
- **Function**: Binary sensor indicating filter needs cleaning (read-only)

## H-Link Protocol

The H-Link protocol is a UART-based serial protocol used by Hitachi HVAC systems:
- **Baud Rate**: 9600
- **Data Bits**: 8
- **Parity**: Odd
- **Stop Bits**: 1

### Protocol Format

**Read Request (MT):**
```
MT P=XXXX C=YYYY\r
```

**Write Request (ST):**
```
ST P=XXXX,DATA C=YYYY\r
```

**Response:**
```
OK P=DATA C=YYYY\r
```

Where:
- `XXXX` = Feature address (16-bit hex)
- `DATA` = Data bytes (hex string)
- `YYYY` = Checksum (0xFFFF - address_high - address_low - data_bytes)

### Supported Features

| Address | Feature | Access |
|---------|---------|--------|
| 0x0000 | Power State | R/W |
| 0x0001 | Mode | R/W |
| 0x0002 | Fan Mode | R/W |
| 0x0003 | Target Temperature | R/W |
| 0x0006 | Remote Control Lock | R/W |
| 0x0007 | Clean Filter Reset | W |
| 0x0014 | Swing Mode | R/W |
| 0x0100 | Current Indoor Temp | R |
| 0x0102 | Current Outdoor Temp | R |
| 0x0300 | Leave Home Write | W |
| 0x0301 | Activity Status | R |
| 0x0302 | Air Filter Warning | R |
| 0x0304 | Leave Home Read | R |
| 0x0800 | Beeper | W |
| 0x0900 | Model Name | R |

## Building and Flashing

### Prerequisites

1. Install ESP-IDF v5.1 or later
2. Set up ESP-IDF environment:
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

### Build

```bash
cd H-Link_zb
idf.py set-target esp32c6
idf.py build
```

### Flash

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

Replace `/dev/ttyUSB0` with your serial port.

## Factory Reset

Hold the **BOOT button** for **5 seconds** to perform a factory reset and clear Zigbee network settings.

## Zigbee2MQTT Integration

The device will be automatically recognized as a thermostat once you add support in Zigbee2MQTT. See the separate converter file `hlink-zb.ts` for Zigbee2MQTT integration.

### Pairing

1. Power on the device
2. It will automatically enter pairing mode (LED indicator)
3. In Zigbee2MQTT, click "Permit Join"
4. Device should appear as "Custom devices (DiY) hlink-zb"

## Configuration

### UART Pins

Edit `main/hlink_driver.h`:
```c
#define HLINK_UART_TX_PIN        GPIO_NUM_20
#define HLINK_UART_RX_PIN        GPIO_NUM_21
```

### Status Update Interval

Edit `main/hlink_driver.h`:
```c
#define HLINK_STATUS_UPDATE_INTERVAL_MS 5000  // Poll every 5 seconds
```

## Troubleshooting

### Device Not Responding

1. Check UART connections (TX/RX/GND)
2. Verify H-Link port is accessible on AC unit
3. Check baud rate and parity settings
4. Monitor serial output: `idf.py monitor`

### Zigbee Connection Issues

1. Check Zigbee coordinator is in pairing mode
2. Perform factory reset (hold BOOT 5 seconds)
3. Check power supply (stable 3.3V/5V)
4. Review Zigbee logs in coordinator

### UART Communication Errors

Enable debug logging in `main/hlink_driver.c`:
```c
static const char *TAG = "HLINK_DRV";
// Check ESP_LOGD output in monitor
```

## References

- [ESP-Zigbee SDK Documentation](https://docs.espressif.com/projects/esp-zigbee-sdk/)
- [ESPHome H-Link AC Component](https://github.com/lumixen/esphome-hlink-ac)
- [Hitachi H-Link Protocol](https://github.com/lumixen/esphome-hlink-ac/blob/main/README.md#h-link-protocol)

## License

See LICENSE file for details.

## Credits

Based on:
- ESP-Zigbee SDK examples by Espressif
- ESPHome H-Link AC component by lumixen
- ACW02 Zigbee project structure
Zigbee device to control Hitachi HVAC (using Espressif ESP32-C6)
