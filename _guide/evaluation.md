---
layout: guide
title: 평가 시스템
permalink: /guide/evaluation/
---

# 평가 시스템

## 평가 시스템 소개

MARC(Meta-Sejong AI Robotics Challenge) 2025의 평가 시스템은 로봇의 성능을 객관적으로 측정하고 점수를 산출하는 시스템입니다. 이 시스템은 다양한 메트릭을 사용하여 로봇의 태스크 수행 능력을 평가합니다.

## 평가 메트릭

### 기본 메트릭

1. **정확도 (Accuracy)**
   - 목표 위치 도달 정확도
   - 객체 조작 정확도
   - 경로 추적 정확도

2. **효율성 (Efficiency)**
   - 태스크 완료 시간
   - 에너지 소비량
   - 계산 리소스 사용량

3. **안정성 (Stability)**
   - 충돌 횟수
   - 오류 발생 빈도
   - 복구 능력

### 고급 메트릭

1. **적응성 (Adaptability)**
   - 환경 변화 대응 능력
   - 예외 상황 처리 능력
   - 학습 속도

2. **협동성 (Cooperation)**
   - 다중 로봇 협동 능력
   - 인간-로봇 상호작용
   - 리소스 공유 효율성

## 평가 시스템 사용법

### 기본 평가

```python
from meta_sejong.evaluation import EvaluationSystem

# 평가 시스템 초기화
eval_system = EvaluationSystem(
    metrics_config="config/metrics.json",
    weights_config="config/weights.json"
)

# 평가 시작
eval_system.start_evaluation()

# 태스크 수행
task_result = robot.perform_task()

# 결과 평가
score = eval_system.evaluate_task(task_result)
```

### 상세 평가

```python
# 개별 메트릭 평가
accuracy_score = eval_system.evaluate_accuracy(task_result)
efficiency_score = eval_system.evaluate_efficiency(task_result)
stability_score = eval_system.evaluate_stability(task_result)

# 종합 점수 계산
total_score = eval_system.calculate_total_score(
    accuracy=accuracy_score,
    efficiency=efficiency_score,
    stability=stability_score
)
```

## 평가 시나리오

### 기본 태스크

1. **물체 조작**
   ```python
   # 물체 조작 태스크 평가
   manipulation_score = eval_system.evaluate_manipulation(
       target_object="box",
       target_position=[1, 1, 1],
       time_limit=30.0
   )
   ```

2. **이동 태스크**
   ```python
   # 이동 태스크 평가
   navigation_score = eval_system.evaluate_navigation(
       start_position=[0, 0, 0],
       goal_position=[2, 2, 0],
       obstacles=True
   )
   ```

### 고급 태스크

1. **다중 태스크**
   ```python
   # 다중 태스크 평가
   multi_task_score = eval_system.evaluate_multi_task(
       tasks=[
           {"type": "manipulation", "target": "box1"},
           {"type": "navigation", "goal": [2, 2, 0]},
           {"type": "grasping", "target": "box2"}
       ],
       time_limit=60.0
   )
   ```

2. **협동 태스크**
   ```python
   # 협동 태스크 평가
   cooperation_score = eval_system.evaluate_cooperation(
       robots=[robot1, robot2],
       task_type="joint_manipulation",
       time_limit=45.0
   )
   ```

## 결과 분석

### 성능 리포트

```python
# 상세 성능 리포트 생성
report = eval_system.generate_report(
    task_results=task_results,
    include_visualization=True
)

# 리포트 저장
report.save("evaluation_results/report.json")
```

### 시각화

```python
# 성능 그래프 생성
eval_system.plot_performance_metrics(
    metrics=["accuracy", "efficiency", "stability"],
    save_path="evaluation_results/performance.png"
)

# 히트맵 생성
eval_system.generate_heatmap(
    metric="accuracy",
    save_path="evaluation_results/accuracy_heatmap.png"
)
```

## 평가 데이터 관리

### 데이터 저장

```python
# 평가 데이터 저장
eval_system.save_evaluation_data(
    data=task_results,
    filename="evaluation_data.json"
)

# 실험 메타데이터 저장
eval_system.save_metadata(
    experiment_info={
        "date": "2024-03-20",
        "robot_type": "manipulator",
        "environment": "indoor"
    },
    filename="experiment_metadata.json"
)
```

### 데이터 분석

```python
# 통계 분석
stats = eval_system.analyze_statistics(
    data_file="evaluation_data.json",
    metrics=["accuracy", "efficiency"]
)

# 성능 추이 분석
trends = eval_system.analyze_trends(
    data_file="evaluation_data.json",
    time_period="1d"
)
```

## 평가 시스템 커스터마이징

### 새로운 메트릭 추가

```python
# 커스텀 메트릭 정의
class CustomMetric(BaseMetric):
    def __init__(self, name, weight):
        super().__init__(name, weight)
    
    def calculate(self, task_result):
        # 메트릭 계산 로직
        return score

# 메트릭 등록
eval_system.register_metric(CustomMetric("custom_metric", 0.2))
```

### 평가 가중치 조정

```python
# 가중치 설정
weights = {
    "accuracy": 0.4,
    "efficiency": 0.3,
    "stability": 0.3
}

# 가중치 적용
eval_system.set_metric_weights(weights)
``` 