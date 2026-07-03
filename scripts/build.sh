#!/bin/bash
# 构建脚本 — PID 倒立摆演示
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo ""
echo "=========================================="
echo "  Build succeeded!"
echo "  Run:  $BUILD_DIR/pid_cart_pole_demo"
echo "=========================================="
