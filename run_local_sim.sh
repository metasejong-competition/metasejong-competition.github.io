#!/bin/bash
conda activate metacom_nav_python310
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
# ---------- 환경 변수 설정 ----------
export ACCEPT_EULA=Y
export PRIVACY_CONSENT=Y
export NVIDIA_VISIBLE_DEVICES=all
export NVIDIA_DRIVER_CAPABILITIES=all
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export AMENT_PREFIX_PATH=/home/sejung/.local/share/ov/pkg/isaac-sim-4.2.0/exts/omni.isaac.ros2_bridge/humble
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/sejung/.local/share/ov/pkg/isaac-sim-4.2.0/exts/omni.isaac.ros2_bridge/humble/lib
export PYTHONPATH=/home/sejung/.local/share/ov/pkg/isaac-sim-4.2.0/kit/python/lib/python3.10/site-packages:/home/sejung/.local/share/ov/pkg/isaac-sim-4.2.0/exts
export USD_PLUGIN_PATH=/home/sejung/.local/share/ov/pkg/isaac-sim-4.2.0/exts/omni.usd.schema.isaac/plugins
export ROS_DISTRO=humble
export EXTENSIONS=omni.isaac.ros2_bridge
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=./fastdds_config/fastdds.xml
export QT_X11_NO_MITSHM=1
export DISPLAY=$DISPLAY
export XAUTHORITY=$XAUTHORITY
export EXTENSION_PATHS=./resources/extensions


# ---------- 캐시 디렉토리 & 데이터 경로 설정 ----------
# 볼륨 마운트한 경로들을 로컬에서도 동일하게 접근할 수 있게 설정
export KIT_CACHE_DIR=./workspace/cache/kit
export OV_CACHE_DIR=./workspace/cache/ov
export PIP_CACHE_DIR=./workspace/cache/pip
export GLCACHE_DIR=./workspace/cache/glcache
export COMPUTECACHE_DIR=./workspace/cache/computecache
export LOG_DIR=./workspace/logs
export DATA_DIR=./workspace/data
export DOCUMENTS_DIR=./workspace/documents

# 필요시 디렉토리 생성
mkdir -p $KIT_CACHE_DIR $OV_CACHE_DIR $PIP_CACHE_DIR $GLCACHE_DIR $COMPUTECACHE_DIR $LOG_DIR $DATA_DIR $DOCUMENTS_DIR

# ---------- 필요 패키지 설치 ----------
pip3 install --no-cache-dir -r ./resources/python/requirements.txt

# ---------- 스크립트 실행 ----------
~/.local/share/ov/pkg/isaac-sim-4.2.0/python.sh simulation_app/codes/run_simulation_app.py
