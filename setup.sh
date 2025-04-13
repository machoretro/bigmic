#!/bin/bash

set -e

echo "=== BigMic RP2040 Setup Script ==="
echo "This script will set up your development environment for the BigMic project."

PROJECT_DIR=$(pwd)
BUILD_DIR="build"
PICO_SDK_PATH="${PROJECT_DIR}/pico-sdk"

# Check for required tools
echo "Checking for required tools..."

# Install dependencies if needed
if ! command -v cmake &> /dev/null || ! command -v arm-none-eabi-gcc &> /dev/null; then
  echo "Installing required dependencies..."
  if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    sudo apt update
    sudo apt install -y cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential libusb-1.0-0-dev
  elif [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    brew install cmake
    brew install --cask gcc-arm-embedded
    brew install libusb
  else
    echo "Unsupported OS. Please install cmake and arm-none-eabi-gcc manually."
    exit 1
  fi
fi

# Initialize/update git submodules
echo "Initializing Pico SDK submodule..."
if [[ ! -d "${PICO_SDK_PATH}" ]]; then
  git submodule update --init --recursive
else
  # Update existing submodules
  git submodule update --recursive
fi

# Verify Pico SDK
if [[ ! -f "${PICO_SDK_PATH}/pico_sdk_init.cmake" ]]; then
  echo "Error: Pico SDK not properly initialized. Please check your submodule configuration."
  exit 1
fi

# Setup environment variable
export PICO_SDK_PATH

# Optional: Install picotool
if ! command -v picotool &> /dev/null; then
  echo "Would you like to install picotool? (y/n)"
  read -r install_picotool
  if [[ "$install_picotool" == [Yy]* ]]; then
    echo "Installing picotool..."
    
    if [[ ! -d "picotool" ]]; then
      git clone https://github.com/raspberrypi/picotool
    fi
    
    pushd picotool
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    popd
    
    # Clean up
    rm -rf picotool
  fi
fi

# Create and configure build directory
echo "Setting up build environment..."
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Configure with CMake
echo "Configuring with CMake..."
cmake .. -DPICO_SDK_PATH="${PICO_SDK_PATH}"

# Build the project
echo "Building project..."
make -j$(nproc)

# Check if build succeeded
if [[ -f "bigmic.uf2" ]]; then
  echo "=== Build Complete ==="
  echo "UF2 File: ${PWD}/bigmic.uf2"
  
  # Check if RP2040 is connected in bootloader mode
  if mount | grep -q "RPI-RP2"; then
    echo "RP2040 detected in bootloader mode."
    read -p "Flash now? (y/n): " flash_now
    if [[ "$flash_now" == [Yy]* ]]; then
      # Find the mounted RP2040
      rp2040_path=$(mount | grep "RPI-RP2" | awk '{print $3}')
      cp bigmic.uf2 "${rp2040_path}/"
      echo "Flashed successfully!"
    fi
  else
    echo ""
    echo "To flash to your RP2040:"
    echo "1. Hold BOOTSEL while plugging in USB"
    echo "2. Run: cp ${PWD}/bigmic.uf2 /path/to/RPI-RP2/"
    
    if command -v picotool &> /dev/null; then
      echo "Or use picotool:"
      echo "picotool load ${PWD}/bigmic.uf2 -f"
    fi
  fi
else
  echo "Error: Build failed or UF2 not found."
  exit 1
fi