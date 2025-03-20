---
layout: guide
title: 개발 환경
permalink: /guide/development-environment/
---

# 개발 환경 설정

## ROS2 Humble 설치

### Ubuntu 22.04에서 ROS2 Humble 설치하기

1. ROS2 저장소 설정
```bash
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

2. ROS2 Humble 설치
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

3. 환경 설정
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## IsaacSim 설치

### NVIDIA Isaac Sim 설치하기

1. NVIDIA Isaac Sim 다운로드
- [NVIDIA Isaac Sim 다운로드 페이지](https://developer.nvidia.com/isaac-sim)에서 최신 버전 다운로드

2. 설치 실행
```bash
chmod +x isaac-sim-*.run
./isaac-sim-*.run
```

3. 환경 변수 설정
```bash
echo "export ISAAC_SIM_PATH=/path/to/isaac-sim" >> ~/.bashrc
source ~/.bashrc
```

## Python 환경 구성

### Python 가상 환경 설정

1. Python 3.8 설치
```bash
sudo apt update
sudo apt install python3.8 python3.8-venv
```

2. 가상 환경 생성 및 활성화
```bash
python3.8 -m venv ~/meta-sejong-env
source ~/meta-sejong-env/bin/activate
```

3. 필요한 패키지 설치
```bash
pip install -r requirements.txt
```

## CUDA 설정

### CUDA 11.0 설치

1. NVIDIA 드라이버 설치
```bash
sudo apt update
sudo apt install nvidia-driver-xxx  # xxx는 최신 버전 번호
```

2. CUDA 11.0 설치
- [NVIDIA CUDA 다운로드 페이지](https://developer.nvidia.com/cuda-11.0-download-archive)에서 설치 파일 다운로드
- 설치 가이드에 따라 설치 진행

3. 환경 변수 설정
```bash
echo "export PATH=/usr/local/cuda-11.0/bin:$PATH" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/usr/local/cuda-11.0/lib64:$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc
```

## 설치 확인

### 환경 설정 확인

1. ROS2 확인
```bash
ros2 --version
```

2. IsaacSim 확인
```bash
isaac-sim --version
```

3. Python 버전 확인
```bash
python --version
```

4. CUDA 확인
```bash
nvidia-smi
nvcc --version
```

## 문제 해결

### 자주 발생하는 문제와 해결 방법

1. ROS2 설치 실패
- 저장소 키가 올바르게 추가되었는지 확인
- 인터넷 연결 상태 확인

2. IsaacSim 실행 오류
- GPU 드라이버가 최신 버전인지 확인
- CUDA 버전 호환성 확인

3. Python 패키지 설치 오류
- 가상 환경이 활성화되어 있는지 확인
- pip가 최신 버전인지 확인 