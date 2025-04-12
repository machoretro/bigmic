#!/bin/bash

set -e

echo "=== RP2040 BigMic Setup Script ==="

PROJECT_NAME="bigmic"
BUILD_DIR="build"
PICO_SDK_DIR="pico-sdk"
UF2_FILE="${PROJECT_NAME}.uf2"

# Optional: install picotool
if ! command -v picotool &> /dev/null; then
  echo "📦 Installing picotool..."

  # Only clone if not already there
  if [[ ! -d "picotool" ]]; then
    git clone https://github.com/raspberrypi/picotool
  fi

  pushd picotool
  mkdir -p build && cd build
  cmake ..
  make -j$(nproc)
  sudo make install
  popd

  # Optionally clean up after install
  echo "🧹 Cleaning up picotool clone..."
  rm -rf picotool
fi

# Fetch submodules
echo "📥 Cloning submodules (SDK, TinyUSB)..."
git submodule update --init --recursive

# Configure build directory
echo "🛠 Building project..."
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake .. -DPICO_SDK_PATH="../${PICO_SDK_DIR}"
make -j$(nproc)

cd ..

# Check for UF2
if [[ ! -f "$BUILD_DIR/$UF2_FILE" ]]; then
  echo "❌ UF2 build failed!"
  exit 1
fi

echo "✅ Build complete: $BUILD_DIR/$UF2_FILE"
echo ""

# Detect + ask to flash
if picotool info &>/dev/null; then
  echo "✅ RP2040 found via USB (picotool)"
  read -p "Flash with picotool? (y/n): " ans
  [[ "$ans" == [Yy]* ]] && sudo picotool load "$BUILD_DIR/$UF2_FILE" -f && exit 0
fi

if mount | grep -q "RPI-RP2"; then
  echo "✅ RP2040 mass storage (BOOTSEL) found"
  read -p "Copy UF2 via drag-and-drop? (y/n): " ans
  [[ "$ans" == [Yy]* ]] && cp "$BUILD_DIR/$UF2_FILE" "/media/$USER/RPI-RP2/" && echo "✅ Copied!" && exit 0
fi

echo "⚠️ No RP2040 board found."
echo "You can flash manually with:"
echo "  cp $BUILD_DIR/$UF2_FILE /media/$USER/RPI-RP2/"
