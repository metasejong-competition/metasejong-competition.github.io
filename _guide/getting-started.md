---
layout: guide
title: 시작하기
permalink: /guide/getting-started/
---

# 시작하기

## 소개

MARC(Meta-Sejong AI Robotics Challenge) 2025는 가상 환경에서 로봇을 위한 체화 인공지능(Embodied AI) 기술의 발전을 촉진하는 것을 목표로 합니다. 이 가이드는 참가자들이 대회에 참여하고 개발을 시작하는 데 필요한 기본적인 정보를 제공합니다.

## 환경 설정

### 시스템 요구사항

- 운영체제: Ubuntu 22.04
- CPU: Intel Xeon Gold 6430 (32코어) 이상
- GPU: GeForce RTX 4090 이상
- RAM: 512GB 이상
- 저장공간: 1TB 이상

### 필수 소프트웨어

1. ROS2 Humble
2. IsaacSim
3. Python 3.8 이상
4. CUDA 11.0 이상

## 프로젝트 구조

```
meta-sejong-competition/
├── src/                    # 소스 코드
│   ├── robot/             # 로봇 관련 코드
│   ├── environment/       # 환경 관련 코드
│   └── utils/            # 유틸리티 함수
├── config/                # 설정 파일
├── docs/                  # 문서
└── tests/                 # 테스트 코드
```

## 기본 사용법

### 1. 저장소 클론

```bash
git clone https://github.com/your-username/meta-sejong-competition.git
cd meta-sejong-competition
```

### 2. 의존성 설치

```bash
pip install -r requirements.txt
```

### 3. 환경 변수 설정

```bash
source setup.sh
```

### 4. 실행 테스트

```bash
python src/main.py
```

## 다음 단계

- [개발 환경](./development-environment.md) 설정하기
- [API 문서](./api-documentation.md) 살펴보기
- [시뮬레이션](./simulation.md) 환경 구성하기 