# Makefile for Nettemp ESP32 firmware builds
# Usage: make all | make standard | make cardputer | make clean | make flash-standard | make flash-cardputer

SKETCH = nettemp_esp32.ino
FIRMWARE_DIR = firmware
BUILD_DIR = build
TIMESTAMP := $(shell date +"%Y%m%d_%H%M%S")
ESPTOOL ?= esptool

# Board configurations
FQBN_STANDARD = esp32:esp32:esp32:PartitionScheme=huge_app
FQBN_C3 = esp32:esp32:esp32c3:PartitionScheme=huge_app,CDCOnBoot=cdc
#FQBN_CARDPUTER = esp32:esp32:m5stack_cardputer:PartitionScheme=huge_app,CDCOnBoot=cdc
FQBN_CARDPUTER = esp32:esp32:m5stack_cardputer:PartitionScheme=huge_app


# Output file names
OUT_STANDARD = $(FIRMWARE_DIR)/nettemp_esp32_standard_$(TIMESTAMP).bin
OUT_STANDARD_BLE_SERVER = $(FIRMWARE_DIR)/nettemp_esp32_standard_ble_server_$(TIMESTAMP).bin
OUT_STANDARD_BLE_I2C_MQTT = $(FIRMWARE_DIR)/nettemp_esp32_standard_ble_i2c_mqtt_$(TIMESTAMP).bin
OUT_STANDARD_BLE_I2C_WEBHOOK = $(FIRMWARE_DIR)/nettemp_esp32_standard_ble_i2c_webhook_$(TIMESTAMP).bin
OUT_CARDPUTER = $(FIRMWARE_DIR)/nettemp_esp32_cardputer_$(TIMESTAMP).bin

.PHONY: all standard standard-ble-server standard-ble-i2c-mqtt standard-ble-i2c-webhook standard-all-variants cardputer flash-standard flash-cardputer clean help erase-standard erase-cardputer flash-standard-clean flash-cardputer-clean

# Default port (override with PORT=/dev/ttyUSB0 make upload-cardputer)
PORT ?= /dev/cu.usbmodem131201

all: standard cardputer
	@echo ""
	@echo "=========================================="
	@echo "Build Complete!"
	@echo "=========================================="
	@ls -lh $(FIRMWARE_DIR)/*.bin | tail -2

help:
	@echo "Nettemp ESP32 Build Targets:"
	@echo "  make all                       - Build Standard and Cardputer versions"
	@echo "  make standard                  - Build Standard ESP32 (alias for standard-ble-server)"
	@echo "  make standard-all-variants     - Build all Standard ESP32 variants"
	@echo ""
	@echo "Standard ESP32 Variants (all with Web + OTA):"
	@echo "  make standard-ble-server       - Web + OTA + BLE + GPIO + Server + Webhook"
	@echo "  make standard-ble-i2c-mqtt     - Web + OTA + BLE + I2C + MQTT + GPIO"
	@echo "  make standard-ble-i2c-webhook  - Web + OTA + BLE + I2C + Server + Webhook + GPIO"
	@echo ""
	@echo "Other Boards:"
	@echo "  make cardputer                 - Build Cardputer ESP32-S3 version only"
	@echo ""
	@echo "Flashing:"
	@echo "  make flash-standard            - Flash Standard build to device"
	@echo "  make flash-cardputer           - Flash Cardputer build to device"
	@echo ""
	@echo "Erase Flash:"
	@echo "  make erase-standard            - Erase flash (Standard ESP32)"
	@echo "  make erase-cardputer           - Erase flash (Cardputer ESP32-S3)"
	@echo ""
	@echo "Erase + Flash:"
	@echo "  make flash-standard-clean      - Erase then flash Standard ESP32"
	@echo "  make flash-cardputer-clean     - Erase then flash Cardputer ESP32-S3"
	@echo ""
	@echo "Cleanup:"
	@echo "  make clean                     - Remove build artifacts"
	@echo ""
	@echo "Upload options:"
	@echo "  PORT=/dev/ttyUSB0 make flash-standard    - Flash standard to custom port"
	@echo "  PORT=/dev/ttyUSB0 make flash-cardputer   - Flash cardputer to custom port"

# Variant 1: Web + OTA + BLE + GPIO + Server + Webhook (no I2C, no MQTT)
standard-ble-server:
	@echo "Building Standard ESP32 - BLE + Server + Webhook variant..."
	@mkdir -p $(FIRMWARE_DIR) $(BUILD_DIR)/standard-ble-server
	arduino-cli compile \
		--fqbn $(FQBN_STANDARD) \
		--build-property "compiler.c.extra_flags=-Oz -ffunction-sections -fdata-sections -flto" \
		--build-property "compiler.cpp.extra_flags=-Oz -ffunction-sections -fdata-sections -flto -DESP32 -DNETTEMP_CARDPUTER_UI=0 -DSSD1306_NO_SPLASH -DNETTEMP_ENABLE_OLED=0 -DNETTEMP_ENABLE_I2C=0 -DNETTEMP_ENABLE_MQTT=0 -DNETTEMP_ENABLE_BLE=1 -DNETTEMP_ENABLE_SERVER=1 -DNETTEMP_ENABLE_PORTAL=1" \
		--build-property "compiler.c.elf.extra_flags=-Wl,--gc-sections -flto" \
		--output-dir $(BUILD_DIR)/standard-ble-server \
		$(SKETCH)
	@cp $(BUILD_DIR)/standard-ble-server/$(SKETCH).bin $(OUT_STANDARD_BLE_SERVER)
	@echo "✓ BLE + Server build: $(OUT_STANDARD_BLE_SERVER)"

# Variant 2: Web + OTA + BLE + I2C + MQTT + GPIO
standard-ble-i2c-mqtt:
	@echo "Building Standard ESP32 - BLE + I2C + MQTT variant..."
	@mkdir -p $(FIRMWARE_DIR) $(BUILD_DIR)/standard-ble-i2c-mqtt
	arduino-cli compile \
		--fqbn $(FQBN_STANDARD) \
		--build-property "compiler.c.extra_flags=-Oz -ffunction-sections -fdata-sections -flto" \
		--build-property "compiler.cpp.extra_flags=-Oz -ffunction-sections -fdata-sections -flto -DESP32 -DNETTEMP_CARDPUTER_UI=0 -DSSD1306_NO_SPLASH -DNETTEMP_ENABLE_OLED=0 -DNETTEMP_ENABLE_I2C=1 -DNETTEMP_ENABLE_MQTT=1 -DNETTEMP_ENABLE_BLE=1 -DNETTEMP_ENABLE_SERVER=1 -DNETTEMP_ENABLE_PORTAL=1" \
		--build-property "compiler.c.elf.extra_flags=-Wl,--gc-sections -flto" \
		--output-dir $(BUILD_DIR)/standard-ble-i2c-mqtt \
		$(SKETCH)
	@cp $(BUILD_DIR)/standard-ble-i2c-mqtt/$(SKETCH).bin $(OUT_STANDARD_BLE_I2C_MQTT)
	@echo "✓ BLE + I2C + MQTT build: $(OUT_STANDARD_BLE_I2C_MQTT)"

# Variant 3: Web + OTA + BLE + I2C + Server + Webhook + GPIO (no MQTT)
standard-ble-i2c-webhook:
	@echo "Building Standard ESP32 - BLE + I2C + Webhook variant..."
	@mkdir -p $(FIRMWARE_DIR) $(BUILD_DIR)/standard-ble-i2c-webhook
	arduino-cli compile \
		--fqbn $(FQBN_STANDARD) \
		--build-property "compiler.c.extra_flags=-Oz -ffunction-sections -fdata-sections -flto" \
		--build-property "compiler.cpp.extra_flags=-Oz -ffunction-sections -fdata-sections -flto -DESP32 -DNETTEMP_CARDPUTER_UI=0 -DSSD1306_NO_SPLASH -DNETTEMP_ENABLE_OLED=0 -DNETTEMP_ENABLE_I2C=1 -DNETTEMP_ENABLE_MQTT=0 -DNETTEMP_ENABLE_BLE=1 -DNETTEMP_ENABLE_SERVER=1 -DNETTEMP_ENABLE_PORTAL=1" \
		--build-property "compiler.c.elf.extra_flags=-Wl,--gc-sections -flto" \
		--output-dir $(BUILD_DIR)/standard-ble-i2c-webhook \
		$(SKETCH)
	@cp $(BUILD_DIR)/standard-ble-i2c-webhook/$(SKETCH).bin $(OUT_STANDARD_BLE_I2C_WEBHOOK)
	@echo "✓ BLE + I2C + Webhook build: $(OUT_STANDARD_BLE_I2C_WEBHOOK)"

# Build all standard variants
standard-all-variants: standard-ble-server standard-ble-i2c-mqtt standard-ble-i2c-webhook
	@echo ""
	@echo "=========================================="
	@echo "All Standard ESP32 Variants Built!"
	@echo "=========================================="
	@ls -lh $(FIRMWARE_DIR)/*standard*.bin | tail -3

# Default standard target (alias for standard-ble-server)
standard: standard-ble-server

cardputer:
	@echo "Building Cardputer ESP32-S3 version..."
	@mkdir -p $(FIRMWARE_DIR) $(BUILD_DIR)/cardputer
	arduino-cli compile \
		--fqbn $(FQBN_CARDPUTER) \
		--build-property "compiler.cpp.extra_flags=-DESP32 -DNETTEMP_CARDPUTER_UI=1 -DNETTEMP_ENABLE_OLED=0 -DNETTEMP_ENABLE_BLE=1 -DNETTEMP_ENABLE_I2C=1 -DNETTEMP_ENABLE_MQTT=1 -DNETTEMP_ENABLE_SERVER=1 -DNETTEMP_ENABLE_PORTAL=1" \
		--output-dir $(BUILD_DIR)/cardputer \
		$(SKETCH)
	@cp $(BUILD_DIR)/cardputer/$(SKETCH).bin $(OUT_CARDPUTER)
	@echo "✓ Cardputer build: $(OUT_CARDPUTER)"

flash-standard: standard
	@echo "Flashing Standard ESP32 on $(PORT)..."
	arduino-cli upload \
		--fqbn $(FQBN_STANDARD) \
		--port $(PORT) \
		--input-dir $(BUILD_DIR)/standard-ble-server
	@echo "✓ Flash complete!"

flash-cardputer: cardputer
	@echo "Flashing Cardputer ESP32-S3 on $(PORT)..."
	arduino-cli upload \
		--fqbn $(FQBN_CARDPUTER) \
		--port $(PORT) \
		--input-dir $(BUILD_DIR)/cardputer
	@echo "✓ Flash complete!"

erase-standard:
	@echo "Erasing Standard ESP32 flash on $(PORT)..."
	$(ESPTOOL) --chip esp32 --port $(PORT) erase-flash
	@echo "✓ Erase complete!"

erase-cardputer:
	@echo "Erasing Cardputer ESP32-S3 flash on $(PORT)..."
	$(ESPTOOL) --chip esp32s3 --port $(PORT) erase-flash
	@echo "✓ Erase complete!"

flash-standard-clean: erase-standard flash-standard

flash-cardputer-clean: erase-cardputer flash-cardputer

clean:
	@echo "Cleaning build artifacts..."
	@rm -rf $(BUILD_DIR)
	@echo "✓ Build directory cleaned"
	@echo ""
	@echo "To also remove firmware binaries, run:"
	@echo "  rm -rf $(FIRMWARE_DIR)/*.bin"
