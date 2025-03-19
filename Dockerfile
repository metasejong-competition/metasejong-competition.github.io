ARG DEBIAN_FRONTEND=noninteractive
ARG ISAACSIM_VERSION=4.2.0

# NVIDIA Isaac Sim 기반 이미지 사용
FROM nvcr.io/nvidia/isaac-sim:${ISAACSIM_VERSION} AS isaac-sim

# 작업 디렉터리 설정
WORKDIR /isaac-sim

# 필요한 패키지 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    x11-xserver-utils \
    lsb-release \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# pip 최신 버전으로 업그레이드
RUN /isaac-sim/kit/python/bin/python3 -m pip install --upgrade pip

# 추가 패키지 설치
# RUN pip install ultralytics && pip install onnx

# 비대화식 패키지 설치 설정
ARG DEBIAN_FRONTEND=noninteractive

# 타임존 설정 (Asia/Seoul로 자동 설정)
RUN apt-get update && apt-get install -y tzdata && \
    ln -fs /usr/share/zoneinfo/Asia/Seoul /etc/localtime && \
    echo "Asia/Seoul" > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata

# ROS2 패키지 설정
RUN echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y ros-humble-ros-base ros-humble-rviz2 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ROS 환경 변수 설정
ENV ROS_DISTRO=humble
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# PYTHONPATH 설정 (export로 수정)
RUN echo "export PYTHONPATH=/isaac-sim/kit/python/lib/python3.10/site-packages:/isaac-sim/exts" >> /root/.bashrc

# USD Plugin Path 설정
ENV USD_PLUGIN_PATH=/isaac-sim/exts/omni.usd.schema.isaac/plugins

# Add symlink for examples
RUN ln -s exts/omni.isaac.examples/omni/isaac/examples extension_examples

# ROS 2 환경이 제대로 설정되도록 ENTRYPOINT 수정
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec $0 $@"]
