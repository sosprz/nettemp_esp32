# Firmware Binaries

This directory contains compiled firmware `.bin` files for Nettemp ESP32.

## File Naming Convention

Firmware files are named with timestamps:
- `nettemp_esp32_standard_YYYYMMDD_HHMMSS.bin` - For generic ESP32 boards (with OLED SSD1306 support)
- `nettemp_esp32_cardputer_YYYYMMDD_HHMMSS.bin` - For M5Stack Cardputer (ESP32-S3 with built-in ST7789 LCD, OLED disabled)

**Key differences:**
- **Standard**: Includes support for external SSD1306 OLED displays
- **Cardputer**: OLED support disabled, uses Cardputer's built-in TFT LCD display

## Flashing Instructions

### Using esptool.py

**Standard ESP32:**
```bash
esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash 0x10000 nettemp_esp32_standard_*.bin
```

**Cardputer ESP32-S3:**
```bash
esptool.py --chip esp32s3 --port /dev/ttyUSB0 write_flash 0x10000 nettemp_esp32_cardputer_*.bin
```

### Using Arduino IDE

1. Go to **Sketch → Export compiled Binary**
2. Or upload via **Tools → Port** and **Sketch → Upload**

### OTA (Over-The-Air) Update

1. Connect to your ESP32's web interface: `http://<device-ip>/`
2. Navigate to the **System** or **OTA** section
3. Upload the appropriate `.bin` file
4. Wait for upload to complete and device to restart

## Building from Source

See the main [README.md](../README.md#building-firmware-binaries-command-line) for build instructions.

Quick build:
```bash
cd ..
make all           # Build both versions
make standard      # Standard ESP32 only
make cardputer     # Cardputer only
```
