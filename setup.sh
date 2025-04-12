#!/bin/bash

set -e

echo "=== RP2040 BigMic Setup Script ==="

# Config
PROJECT_NAME="bigmic"
BUILD_DIR="build"
PICO_SDK_PATH="pico-sdk"

# Optional: Install picotool
if ! command -v picotool &> /dev/null; then
  echo "Installing picotool (optional but useful)..."
  sudo apt install -y cmake libusb-1.0-0-dev
  git clone https://github.com/raspberrypi/picotool
  pushd picotool
  mkdir build && cd build
  cmake ..
  make -j4
  sudo make install
  popd
  rm -rf picotool
fi

# Clone submodules
echo "Cloning submodules (Pico SDK, TinyUSB)..."
git submodule update --init --recursive

# Configure build directory
echo "Creating build directory: $BUILD_DIR"
mkdir -p $BUILD_DIR
cd $BUILD_DIR

# Run CMake
echo "Configuring with CMake..."
cmake ..

# Build
echo "Building project..."
make -j$(nproc)

# Success
UF2_PATH="$BUILD_DIR/${PROJECT_NAME}.uf2"
if [[ -f "${PROJECT_NAME}.uf2" ]]; then
  echo "=== Build Complete ==="
  echo "UF2 File: $PWD/${PROJECT_NAME}.uf2"
else
  echo "Build completed but UF2 not found — check for errors."
  exit 1
fi

# Flash hint
echo ""
echo "To flash to your RP2040-Zero:"
echo "1. Hold BOOTSEL while plugging in USB."
echo "2. Copy the file above to the RPI-RP2 drive."
