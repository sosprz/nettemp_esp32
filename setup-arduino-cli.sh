#!/bin/bash
# Setup script for arduino-cli with ESP32 support and required libraries

set -e

echo "========================================"
echo "Arduino CLI Setup for Nettemp ESP32"
echo "========================================"

# Check if arduino-cli is installed
if ! command -v arduino-cli &> /dev/null; then
    echo "✗ arduino-cli is not installed"
    echo ""
    echo "Install it:"
    echo "  macOS:   brew install arduino-cli"
    echo "  Linux:   curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
    echo "  Windows: winget install ArduinoSA.Arduino-CLI"
    echo ""
    echo "Or download from: https://arduino.github.io/arduino-cli/"
    exit 1
fi

echo "✓ arduino-cli found"
echo ""

# Update core index
echo "Updating core index..."
arduino-cli core update-index

# Install ESP32 core
echo ""
echo "Installing ESP32 core..."
if arduino-cli core list | grep -q "esp32:esp32"; then
    echo "✓ ESP32 core already installed, updating..."
    arduino-cli core upgrade esp32:esp32
else
    echo "Installing ESP32 core..."
    arduino-cli core install esp32:esp32
fi

# Update library index
echo ""
echo "Updating library index..."
arduino-cli lib update-index

# Install/Update required libraries
echo ""
echo "Installing/Updating required libraries..."

LIBRARIES=(
    "M5Cardputer"
    "NimBLE-Arduino"
    "Adafruit GFX Library"
    "Adafruit SSD1306"
    "PubSubClient"
    "OneWire"
    "DallasTemperature"
    "DHT sensor library"
)

for lib in "${LIBRARIES[@]}"; do
    echo ""
    echo "Processing: $lib"
    if arduino-cli lib list | grep -q "$lib"; then
        echo "  ✓ Already installed, upgrading..."
        arduino-cli lib upgrade "$lib" || echo "  ⚠ Upgrade failed or not needed"
    else
        echo "  Installing..."
        arduino-cli lib install "$lib"
    fi
done

echo ""
echo "========================================"
echo "Setup Complete!"
echo "========================================"
echo ""
echo "Installed cores:"
arduino-cli core list
echo ""
echo "Installed libraries:"
arduino-cli lib list | head -20
echo ""
echo "You can now build firmware using:"
echo "  make all"
echo "  make standard"
echo "  make cardputer"
echo "========================================"
