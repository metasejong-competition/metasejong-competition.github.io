---
layout: default
title: {{ site.data.translations.ko.home.title }}
lang: ko
ref: home
permalink: /ko/
---
# META-SEJONG AI 로보틱스 챌린지 2025

## 개요 (Overview)
제안된 챌린지는 가상 환경에서 로봇을 위한 Embodied AI 기술의 발전을 촉진하는 것을 목표로 합니다. 참가자들은 주어진 가상 환경 내에서 로봇에 적용할 수 있는 다양한 기술적 요소를 통합하여 특정 미션을 수행해야 합니다. 이러한 기술들은 시뮬레이션 환경에서 개발 및 테스트된 후, 실제 응용 분야에서 효과적으로 활용될 예정입니다.

<div class="intro-video"> 안내영상 </div>

## 시작하기 (Getting Started)
### 시스템 요구사항 (System Requirements)
- **운영체제**: Ubuntu 22.04
- **CPU**: Intel Xeon Gold 6430 (32코어)
- **GPU**: GeForce RTX 4090
- **RAM**: 512GB
- **소프트웨어**: ROS2 Humble

### 설치 및 설정 (Installation & Setup)
1. 공식 GitHub 저장소에서 코드 다운로드 (링크 TBD)
2. Docker 기반 시뮬레이션 환경 설정
3. ROS2 및 필수 패키지 설치

## 사용법 (Usage)
### 제공되는 리소스 (Available Resources)
- **3D 모델 (USD 포맷)**
  - 세종대학교 캠퍼스 (META-SEJONG)
  - 자율 이동 로봇 모델
  - 객체 모델 (분류 및 수집 대상)
- **센서 데이터 (ROS2 토픽 기반 제공)**
  - 카메라 영상 스트리밍 데이터
  - 로봇의 위치, 방향, LiDAR 데이터

### 기본 실행 방법 (Basic Execution)
```bash
# 환경 설정
source /opt/ros/humble/setup.bash

# 시뮬레이션 실행
docker-compose up -d
```

## 예제 (Examples)
### AI 모델 학습 및 적용 (Training & Deploying AI Models)
TBD

### 내비게이션 및 경로 계획 (Navigation & Path Planning)
TBD

## 평가 방법 (Evaluation)
### 평가 기준 (Scoring Criteria)
- **내비게이션 정확도**: 경로 계획 및 이동 효율성
- **객체 인식 및 분류 정확도**: AI 알고리즘 성능 평가
- **수집 성공률**: 성공적으로 인식 및 수집한 객체의 수

### 제출 및 검증 (Submission & Verification)
1. 참가자는 최종 AI 솔루션을 Docker 컨테이너 형태로 제출
2. 조직위원회가 평가 시스템에서 자동 실행 및 채점
3. 결과는 공식 웹사이트를 통해 발표

## 일정 (Timeline)
- **2025년 3월 31일** – 챌린지 접수 시작
- **2025년 5월 30일** – 제출 마감
- **2025년 6월 16일** – 수상자 발표
- **2025년 8월 27~29일** – IEEE Metacom 2025 (세종대학교)

## 지원 및 문의 (Support & Contact)
- 공식 GitHub 저장소: TBD
- 문의 메일: TBD

이 문서는 지속적으로 업데이트될 예정이므로 최신 정보를 확인하세요!

