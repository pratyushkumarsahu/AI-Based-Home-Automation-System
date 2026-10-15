#!/usr/bin/env bash
set -e

# Script to download lightweight object detection model weights (ONNX format)
# Requirement 6: Setup script for real YOLO ONNX model (no large binaries in git)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_DIR="${SCRIPT_DIR}/../models"
mkdir -p "${TARGET_DIR}"

MODEL_FILE="${TARGET_DIR}/yolov5n.onnx"

echo "=== Smart Home Detection Model Downloader ==="
echo "Target directory: ${TARGET_DIR}"

if [ -f "${MODEL_FILE}" ] && [ "$(stat -c%s "${MODEL_FILE}" 2>/dev/null || stat -f%z "${MODEL_FILE}" 2>/dev/null)" -gt 1000000 ]; then
    echo "Valid YOLO ONNX model weights already exist at: ${MODEL_FILE}"
    # Copy to install directory if available and not already symlinked
    INSTALL_TARGET="${SCRIPT_DIR}/../../../install/smart_home_detection/share/smart_home_detection/models"
    if [ -d "${INSTALL_TARGET}" ] && ! [ "${MODEL_FILE}" -ef "${INSTALL_TARGET}/yolov5n.onnx" ]; then
        cp -f "${MODEL_FILE}" "${INSTALL_TARGET}/"
    fi
    exit 0
fi

echo "Downloading real YOLO ONNX model weights..."
PRIMARY_URL="https://raw.githubusercontent.com/doleron/yolov5-opencv-cpp-python/main/config_files/yolov5n.onnx"
MIRROR_URL="https://github.com/doleron/yolov5-opencv-cpp-python/raw/main/config_files/yolov5n.onnx"

download_success=false

if command -v curl >/dev/null 2>&1; then
    if curl -f -L --connect-timeout 10 --max-time 120 -o "${MODEL_FILE}" "${PRIMARY_URL}"; then
        download_success=true
    elif curl -f -L --connect-timeout 10 --max-time 120 -o "${MODEL_FILE}" "${MIRROR_URL}"; then
        download_success=true
    fi
elif command -v wget >/dev/null 2>&1; then
    if wget --timeout=15 -O "${MODEL_FILE}" "${PRIMARY_URL}"; then
        download_success=true
    elif wget --timeout=15 -O "${MODEL_FILE}" "${MIRROR_URL}"; then
        download_success=true
    fi
fi

# Verify downloaded file is a valid model (> 1MB)
FILE_SIZE=$(stat -c%s "${MODEL_FILE}" 2>/dev/null || stat -f%z "${MODEL_FILE}" 2>/dev/null || echo 0)
if [ "${FILE_SIZE}" -gt 1000000 ]; then
    echo "Successfully downloaded real YOLO ONNX model (${FILE_SIZE} bytes) to ${MODEL_FILE}!"
    INSTALL_TARGET="${SCRIPT_DIR}/../../../install/smart_home_detection/share/smart_home_detection/models"
    if [ -d "${INSTALL_TARGET}" ] && ! [ "${MODEL_FILE}" -ef "${INSTALL_TARGET}/yolov5n.onnx" ]; then
        cp -f "${MODEL_FILE}" "${INSTALL_TARGET}/"
    fi
else
    echo "Warning: Download failed or incomplete (${FILE_SIZE} bytes)."
    echo "The detection pipeline will engage the built-in Color/Motion Fallback Detector."
    rm -f "${MODEL_FILE}"
fi
