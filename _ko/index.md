---
layout: default
title: META-SEJONG AI Robotics Challenge
lang: ko
ref: home
permalink: /ko/
---
{% assign t = site.data.translations.ko %}

<nav class="section-nav">
  <a href="#intro-section">{{ t.nav.home }}</a>
  <a href="#environment-section">경진대회 환경</a>
  <a href="#objectives-section">목표</a>
  <a href="#api-section">APIs</a>
  <a href="#evaluation-section">평가</a>
</nav>

<div class="title">{{ t.home.event.title }}</div>

<div class="title-event"><a href="https://ieee-metacom.org">IEEE MetaCom 2025</a></div>

<section id="intro-section">
  <div class="title-level-1">{{ t.home.intro-section.title }}</div>

  {% assign intro = t.home.intro-section.content %}
  <p>{{ intro.text-1 }}</p>
  <p>{{ intro.text-2 }}</p>
  <p>{{ intro.text-3 }}</p>

  <p>{{ intro.text-4 }}</p>
  <ul>
    <li>{{ intro.text-4-1 }}</li>
    <li>{{ intro.text-4-2 }}</li>
    <li>{{ intro.text-4-3 }}</li>
    <li>{{ intro.text-4-4 }}</li>
  </ul>

  <p>{{ intro.text-5 }}</p>

  <div class="intro-video">
    <img src="/assets/images/meta-sejong.png" alt="Competition Introduction Video" style="width: 100%; height: 100%; object-fit: cover;">
  </div>

  <div class="links">
    <div class="link-button">
      <a href=""><i class="fab fa-github"></i> Code</a>
    </div>
    <div class="link-button">
      <a href=""><i class="fas fa-book"></i> Docs</a>
    </div>
    <div class="link-button">
      <a href=""><i class="fas fa-database"></i> Data</a>
    </div>
    <div class="link-button">
      <a href=""><i class="fas fa-video"></i> Video</a>
    </div>
  </div>
</section>

<section id="environment-section">
  <div class="title-level-1">{{ t.home.environment-section.title }}</div>
  {% assign env = t.home.environment-section.content %}
  <p>{{ env.text-1 }}</p>
  <p>{{ env.text-2 }}</p>
  <p>{{ env.text-3 }}</p>
</section>

<section id="objectives-section">
  <div class="title-level-1">{{ t.home.objectives-section.title }}</div>
  {% assign obj = t.home.objectives-section.content %}
  <p>{{ obj.text-1 }}</p>
  <p>{{ obj.text-2 }}</p>
  <p>{{ obj.text-3 }}</p>
  <p>{{ obj.text-4 }}</p>
  <p>{{ obj.text-5 }}</p>
</section>

<section id="api-section">
  <div class="title-level-1">{{ t.home.api-section.title }}</div>
  {% assign api = t.home.api-section.content %}
  <p>{{ api.text-1 }}</p>
  
  <h3>{{ api.text-2 }}</h3>
  <ul>
    <li>{{ api.text-2-1 }}</li>
    <li>{{ api.text-2-2 }}</li>
    <li>{{ api.text-2-3 }}</li>
  </ul>

  <h3>{{ api.text-3 }}</h3>
  <ul>
    <li>{{ api.text-3-1 }}</li>
    <li>{{ api.text-3-2 }}</li>
    <li>{{ api.text-3-3 }}</li>
    <li>{{ api.text-3-4 }}</li>
  </ul>

  <h3>{{ api.text-4 }}</h3>
  <ul>
    <li>{{ api.text-4-1 }}</li>
    <li>{{ api.text-4-2 }}</li>
  </ul>
</section>

<section id="evaluation-section">
  <div class="title-level-1">{{ t.home.evaluation-section.title }}</div>
  {% assign eval = t.home.evaluation-section.content %}
  <p>{{ eval.text-1 }}</p>
  <p>{{ eval.text-2 }}</p>
  <p>{{ eval.text-3 }}</p>
</section>

<section id="sponsor-section">
  <div class="title-level-1">{{ t.sponsor-section.title }}</div>
  {% assign sponsors = t.sponsor-section.content %}
  <div class="sponsors">
    {% for sponsor in sponsors %}
    <div class="sponsor">
      <a href="{{ sponsor[1].link }}">
        <img src="/assets/images/{{ sponsor[1].image }}" alt="{{ sponsor[1].name }}">
      </a>
    </div>
    {% endfor %}
  </div>
</section>

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

