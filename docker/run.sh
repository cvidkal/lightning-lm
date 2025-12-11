#!/bin/bash

# Docker容器运行脚本
# 用于运行lightning-lm的Docker容器，支持GUI和路径映射

set -e

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

IMAGE_NAME="lightning-lm"
TAG="latest"
CONTAINER_NAME="lightning-lm-container"

# 检查镜像是否存在
if ! docker images | grep -q "${IMAGE_NAME}.*${TAG}"; then
    echo "错误: 镜像 ${IMAGE_NAME}:${TAG} 不存在"
    echo "请先运行 ./build.sh 构建镜像"
    exit 1
fi

# 检查容器是否已存在
if docker ps -a | grep -q "${CONTAINER_NAME}"; then
    echo "容器 ${CONTAINER_NAME} 已存在"
    read -p "是否删除现有容器并重新创建? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "停止并删除现有容器..."
        docker stop ${CONTAINER_NAME} 2>/dev/null || true
        docker rm ${CONTAINER_NAME} 2>/dev/null || true
    else
        echo "使用现有容器..."
        docker start ${CONTAINER_NAME}
        docker exec -it ${CONTAINER_NAME} /bin/bash
        exit 0
    fi
fi

# 设置X11转发（GUI支持）
xhost +local:docker 2>/dev/null || true

# 获取显示环境变量
DISPLAY=${DISPLAY:-:0}

echo "=========================================="
echo "启动Docker容器: ${CONTAINER_NAME}"
echo "工作空间: ${WORKSPACE_DIR}"
echo "=========================================="

# 运行Docker容器
docker run -it \
    --name ${CONTAINER_NAME} \
    --privileged \
    --network host \
    --ipc host \
    --env DISPLAY=${DISPLAY} \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume ${WORKSPACE_DIR}:/root/lighting_lm_ws:rw \
    --volume /media:/media:rw \
    --volume /dev:/dev \
    ${IMAGE_NAME}:${TAG} \
    /bin/bash

echo "=========================================="
echo "容器已退出"
echo "=========================================="



