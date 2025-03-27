#!/bin/bash
set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo_success() {
    echo -e "${GREEN}$1${NC}"
}

echo_error() {
    echo -e "${RED}$1${NC}"
}

echo_warning() {
    echo -e "${YELLOW}$1${NC}"
}

# Check system
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    if ! command -v brew &> /dev/null; then
        echo_error "Homebrew not found. Please install Homebrew first."
        exit 1
    fi

    # Install/Update dependencies
    echo "=== Checking dependencies ==="

    # Check and install Boost
    if ! brew list boost &>/dev/null; then
        echo_success "Installing Boost..."
        brew install boost
    else
        echo_warning "Boost is already installed"
    fi

    # Check and install Eigen
    if ! brew list eigen &>/dev/null; then
        echo_success "Installing Eigen..."
        brew install eigen
    else
        echo_warning "Eigen is already installed"
    fi

    # Clean any previous CMake cache
    echo "=== Cleaning CMake cache ==="
    rm -rf build
    rm -f ~/.cmake/packages/Eigen3/*/Eigen3Config.cmake
    find . -name "CMakeCache.txt" -delete
    find . -name "CMakeFiles" -type d -exec rm -rf {} +

    # Get Eigen path from Homebrew
    EIGEN3_PATH=$(brew --prefix eigen)
    echo_success "Eigen3 path: ${EIGEN3_PATH}"

elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Linux
    sudo apt-get update
    sudo apt-get install -y \
        build-essential \
        cmake \
        libboost-all-dev \
        libeigen3-dev
else
    echo_error "Unsupported operating system: $OSTYPE"
    exit 1
fi

# Create build directory
echo "=== Creating build directory ==="
mkdir -p build

# Configure CMake
echo "=== Configuring CMake ==="
if [[ "$OSTYPE" == "darwin"* ]]; then
    cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_VERBOSE_MAKEFILE=ON \
        -DEIGEN3_INCLUDE_DIR="${EIGEN3_PATH}/include/eigen3"
else
    cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_VERBOSE_MAKEFILE=ON
fi

# Build
echo "=== Building project ==="
cmake --build build --verbose -j$(nproc 2>/dev/null || sysctl -n hw.ncpu)

if [ $? -eq 0 ]; then
    echo_success "=== Build completed successfully ==="
else
    echo_error "=== Build failed ==="
    exit 1
fi

