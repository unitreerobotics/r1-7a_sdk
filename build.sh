#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
MOTOR_LIB_DIR="$PROJECT_DIR/thirdparty/motor_controller/lib"

show_usage() {
    echo "Usage: $0 [x86|arm64|aarch64]"
    echo ""
    echo "Platforms:"
    echo "  x86   - Link x86 motor_controller library"
    echo "  arm64 - Link aarch64 motor_controller library"
    echo ""
    echo "If no platform is specified, you will be prompted to choose one."
}

select_platform() {
    echo "Select build platform:"
    echo "  1) x86"
    echo "  2) arm64"
    read -rp "Enter choice [1-2]: " choice
    case "$choice" in
        1) PLATFORM="x86" ;;
        2) PLATFORM="arm64" ;;
        *) echo "Invalid choice: $choice" >&2; exit 1 ;;
    esac
}

PLATFORM=""

if [ $# -gt 1 ]; then
    show_usage
    exit 1
elif [ $# -eq 1 ]; then
    case "$1" in
        x86|x86_64|amd64) PLATFORM="x86" ;;
        arm64|aarch64)    PLATFORM="arm64" ;;
        -h|--help)        show_usage; exit 0 ;;
        *)                echo "Unknown platform: $1" >&2; show_usage; exit 1 ;;
    esac
else
    select_platform
fi

echo "Building for platform: $PLATFORM"

case "$PLATFORM" in
    x86)
        MOTOR_LIB_SUBDIR="x86"
        ;;
    arm64)
        MOTOR_LIB_SUBDIR="aarch64"
        ;;
esac

SELECTED_MOTOR_LIB_DIR="$MOTOR_LIB_DIR/$MOTOR_LIB_SUBDIR"

if [ ! -d "$SELECTED_MOTOR_LIB_DIR" ]; then
    echo "Motor controller library directory not found: $SELECTED_MOTOR_LIB_DIR" >&2
    exit 1
fi

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

cmake -S "$PROJECT_DIR" -B "$BUILD_DIR" \
    -DMOTOR_CONTROLLER_LIB_DIR="$SELECTED_MOTOR_LIB_DIR"

cmake --build "$BUILD_DIR"

echo ""
echo "Build completed. Output directory: $BUILD_DIR"
echo "Linked motor_controller library: $SELECTED_MOTOR_LIB_DIR"
