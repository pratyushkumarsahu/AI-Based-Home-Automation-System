#!/usr/bin/env bash
set -e
# Top-level setup script for real YOLO ONNX model
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "${WS_DIR}/src/smart_home_detection/scripts/download_models.sh"
