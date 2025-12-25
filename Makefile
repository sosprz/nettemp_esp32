# Makefile for Nettemp ESP32 firmware builds
# Usage: make all | make standard | make cardputer | make clean

SKETCH = nettemp_esp32.ino
FIRMWARE_DIR = firmware
BUILD_DIR = build
TIMESTAMP := $(shell date +"%Y%m%d_%H%M%S")

# Board configurations
FQBN_STANDARD = esp32:esp32:esp32:PartitionScheme=huge_app
FQBN_CARDPUTER = esp32:esp32:esp32s3:PartitionScheme=huge_app

# Output file names
OUT_STANDARD = $(FIRMWARE_DIR)/nettemp_esp32_standard_$(TIMESTAMP).bin
OUT_CARDPUTER = $(FIRMWARE_DIR)/nettemp_esp32_cardputer_$(TIMESTAMP).bin

.PHONY: all standard cardputer clean help

all: standard cardputer
	@echo ""
	@echo "=========================================="
	@echo "Build Complete!"
	@echo "=========================================="
	@ls -lh $(FIRMWARE_DIR)/*.bin | tail -2

help:
	@echo "Nettemp ESP32 Build Targets:"
	@echo "  make all        - Build both Standard and Cardputer versions"
	@echo "  make standard   - Build Standard ESP32 version only"
	@echo "  make cardputer  - Build Cardputer ESP32-S3 version only"
	@echo "  make clean      - Remove build artifacts"
	@echo "  make help       - Show this help"

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
		--build-property "compiler.cpp.extra_flags=-DESP32 -DNETTEMP_CARDPUTER_UI=1 -DNETTEMP_ENABLE_OLED=0" \
		--output-dir $(BUILD_DIR)/cardputer \
		$(SKETCH)
	@cp $(BUILD_DIR)/cardputer/$(SKETCH).bin $(OUT_CARDPUTER)
	@echo "✓ Cardputer build: $(OUT_CARDPUTER)"

clean:
	@echo "Cleaning build artifacts..."
	@rm -rf $(BUILD_DIR)
	@echo "✓ Build directory cleaned"
	@echo ""
	@echo "To also remove firmware binaries, run:"
	@echo "  rm -rf $(FIRMWARE_DIR)/*.bin"
