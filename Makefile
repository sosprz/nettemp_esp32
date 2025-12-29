# Makefile for Nettemp ESP32 firmware builds
# Usage: make all | make standard | make cardputer | make clean | make flash-standard | make flash-cardputer

SKETCH = nettemp_esp32.ino
FIRMWARE_DIR = firmware
BUILD_DIR = build
TIMESTAMP := $(shell date +"%Y%m%d_%H%M%S")
ESPTOOL ?= esptool

# Board configurations
FQBN_STANDARD = esp32:esp32:esp32:PartitionScheme=huge_app
#FQBN_CARDPUTER = esp32:esp32:m5stack_cardputer:PartitionScheme=huge_app,CDCOnBoot=cdc
FQBN_CARDPUTER = esp32:esp32:m5stack_cardputer:PartitionScheme=huge_app


# Output file names
OUT_STANDARD = $(FIRMWARE_DIR)/nettemp_esp32_standard_$(TIMESTAMP).bin
OUT_CARDPUTER = $(FIRMWARE_DIR)/nettemp_esp32_cardputer_$(TIMESTAMP).bin

.PHONY: all standard cardputer upload-cardputer flash-standard flash-cardputer clean help erase-standard erase-cardputer flash-standard-clean flash-cardputer-clean

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
	@echo "  make all              - Build both Standard and Cardputer versions"
	@echo "  make standard         - Build Standard ESP32 version only"
	@echo "  make cardputer        - Build Cardputer ESP32-S3 version only"
	@echo "  make upload-cardputer - Upload Cardputer firmware to device"
	@echo "  make flash-standard   - Flash Standard build to device"
	@echo "  make flash-cardputer  - Flash Cardputer build to device"
	@echo "  make erase-standard   - Erase flash (Standard ESP32)"
	@echo "  make erase-cardputer  - Erase flash (Cardputer ESP32-S3)"
	@echo "  make flash-standard-clean  - Erase then flash Standard ESP32"
	@echo "  make flash-cardputer-clean - Erase then flash Cardputer ESP32-S3"
	@echo "  make clean            - Remove build artifacts"
	@echo "  make help             - Show this help"
	@echo ""
	@echo "Upload options:"
	@echo "  PORT=/dev/ttyUSB0 make upload-cardputer  - Upload to custom port"
	@echo "  PORT=/dev/ttyUSB0 make flash-standard    - Flash standard to custom port"
	@echo "  PORT=/dev/ttyUSB0 make flash-cardputer   - Flash cardputer to custom port"
	@echo "  PORT=/dev/ttyUSB0 make erase-standard    - Erase standard on custom port"
	@echo "  PORT=/dev/ttyUSB0 make erase-cardputer   - Erase cardputer on custom port"

standard:
	@echo "Building Standard ESP32 version..."
	@mkdir -p $(FIRMWARE_DIR) $(BUILD_DIR)/standard
	arduino-cli compile \
		--fqbn $(FQBN_STANDARD) \
		--build-property "compiler.cpp.extra_flags=-DESP32 -DNETTEMP_CARDPUTER_UI=0 -DSSD1306_NO_SPLASH" \
		--output-dir $(BUILD_DIR)/standard \
		$(SKETCH)
	@cp $(BUILD_DIR)/standard/$(SKETCH).bin $(OUT_STANDARD)
	@echo "✓ Standard build: $(OUT_STANDARD)"

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
		--input-dir $(BUILD_DIR)/standard
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
