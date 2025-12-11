#!/bin/bash

# Docker镜像构建脚本
# 用于构建lightning-lm的Docker镜像

set -e

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="lightning-lm"
TAG="latest"

echo "=========================================="
echo "开始构建Docker镜像: ${IMAGE_NAME}:${TAG}"
echo "=========================================="

# 构建Docker镜像
docker build -t ${IMAGE_NAME}:${TAG} -f ${SCRIPT_DIR}/Dockerfile ${SCRIPT_DIR}/..

echo "=========================================="
echo "Docker镜像构建完成: ${IMAGE_NAME}:${TAG}"
echo "=========================================="



