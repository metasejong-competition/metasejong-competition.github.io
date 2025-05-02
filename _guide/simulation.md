---
layout: guide
title: 시뮬레이션
permalink: /guide/simulation/
---

# 시뮬레이션 환경

## 시뮬레이션 환경 소개

MARC(Meta-Sejong AI Robotics Challenge) 2025는 NVIDIA Isaac Sim을 기반으로 한 가상 환경에서 진행됩니다. 이 환경은 실제 로봇과 유사한 물리적 특성과 센서 데이터를 제공하며, 다양한 태스크를 수행할 수 있는 시뮬레이션 환경을 제공합니다.

## 환경 구성

### 기본 씬 구성

```
simulation/
├── scenes/                    # 시뮬레이션 씬 파일
│   ├── default_scene.json    # 기본 씬
│   └── custom_scenes/        # 사용자 정의 씬
├── robots/                   # 로봇 모델
│   ├── manipulator/         # 매니퓰레이터 로봇
│   └── mobile/              # 이동 로봇
├── objects/                  # 환경 객체
│   ├── furniture/           # 가구
│   └── tools/               # 도구
└── config/                   # 설정 파일
    ├── physics.json         # 물리 엔진 설정
    └── lighting.json        # 조명 설정
```

### 물리 엔진 설정

```json
{
  "gravity": [0, 0, -9.81],
  "substeps": 4,
  "solver_type": "iterative",
  "num_solver_iterations": 4,
  "contact_offset": 0.02,
  "friction": 0.5,
  "restitution": 0.1
}
```

### 조명 설정

```json
{
  "ambient_light": {
    "intensity": 0.5,
    "color": [1, 1, 1]
  },
  "directional_lights": [
    {
      "intensity": 1.0,
      "direction": [1, -1, -1],
      "color": [1, 1, 1]
    }
  ]
}
```

## 환경 사용법

### 환경 초기화

```python
from meta_sejong.simulation import SimulationEnvironment

# 환경 초기화
env = SimulationEnvironment(
    scene_path="scenes/default_scene.json",
    physics_config="config/physics.json",
    lighting_config="config/lighting.json"
)

# 환경 시작
env.start()
```

### 로봇 제어

```python
# 로봇 생성
robot = env.create_robot(
    robot_type="manipulator",
    position=[0, 0, 0],
    orientation=[0, 0, 0]
)

# 로봇 제어
robot.move_to(
    target_position=[1, 1, 1],
    target_orientation=[0, 0, 0]
)
```

### 객체 조작

```python
# 객체 생성
object = env.create_object(
    object_type="box",
    position=[2, 2, 0],
    size=[0.1, 0.1, 0.1]
)

# 객체 조작
robot.grasp(object)
robot.move_to([3, 3, 1])
robot.release(object)
```

## 센서 데이터

### 카메라 데이터

```python
# 카메라 설정
camera = env.create_camera(
    position=[0, 0, 2],
    orientation=[0, 0, 0],
    resolution=[640, 480],
    fov=60
)

# 이미지 캡처
image = camera.capture()
```

### 관성 센서 데이터

```python
# IMU 데이터 획득
imu_data = robot.get_imu_data()
print(f"Acceleration: {imu_data['acceleration']}")
print(f"Angular velocity: {imu_data['angular_velocity']}")
print(f"Orientation: {imu_data['orientation']}")
```

## 환경 커스터마이징

### 새로운 씬 생성

1. Isaac Sim 에디터에서 씬 구성
2. 씬 저장
```python
env.save_scene("custom_scenes/my_scene.json")
```

### 새로운 객체 추가

1. 3D 모델 준비 (USD 또는 OBJ 형식)
2. 객체 등록
```python
env.register_object(
    name="custom_object",
    model_path="objects/custom/model.usd",
    properties={
        "mass": 1.0,
        "friction": 0.5,
        "restitution": 0.1
    }
)
```

## 디버깅

### 시각화 도구

```python
# 디버그 시각화 활성화
env.enable_debug_visualization()

# 충돌 박스 표시
env.show_collision_boxes()

# 힘 벡터 표시
env.show_force_vectors()
```

### 로깅

```python
# 환경 상태 로깅
env.log_state()

# 충돌 이벤트 로깅
env.log_collisions()

# 성능 메트릭 로깅
env.log_performance_metrics()
```

## 성능 최적화

### 시뮬레이션 속도 조절

```python
# 시뮬레이션 속도 설정
env.set_simulation_speed(1.0)  # 1.0 = 실시간

# 물리 시뮬레이션 스텝 크기 조정
env.set_physics_timestep(0.01)  # 10ms
```

### 리소스 관리

```python
# 메모리 사용량 최적화
env.optimize_memory_usage()

# GPU 메모리 관리
env.manage_gpu_memory()
``` 