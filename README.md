# Modular-V: 시각장애인 안내 로봇 프레임워크

## 📋 개요
Modular-V는 시각장애인의 안전한 실내외 이동을 지원하는 모듈형 로봇 시스템 프레임워크입니다. ROS2 기반으로 구현되어 있으며, RTAB-Map SLAM, ZED 2i 스테레오 카메라, 음성 인터페이스 등 다양한 모듈을 통합합니다.

## 🎯 주요 특징
- **모듈형 아키텍처**: 독립적인 모듈 개발 및 통합
- **실시간 SLAM**: RTAB-Map 기반 3D 매핑 및 2D occupancy grid 생성
- **안전한 내비게이션**: Nav2 기반 장애물 회피 및 경로 계획
- **3D to 2D 변환**: 3D 포인트클라우드를 2D 맵으로 자동 변환하여 Nav2 호환
- **멀티모달 인터페이스**: 음성 및 햅틱 피드백
- **확장 가능**: 새로운 센서 및 기능 쉽게 추가

## 📁 프로젝트 구조
```
modular-v/
├── core/                  # 핵심 프레임워크
├── modules/              # 기능 모듈
│   ├── perception/       # 인지 (카메라, SLAM, 감지)
│   ├── navigation/       # 내비게이션 (경로 계획, 제어)
│   ├── mobility/         # 이동 (모터 제어, 오도메트리)
│   └── interaction/      # 상호작용 (음성, 햅틱)
├── interfaces/           # ROS2 메시지/서비스
├── launch/              # 런치 파일
├── config/              # 설정 파일
├── docs/                # 문서
│   ├── FRAMEWORK_GUIDE.md      # 프레임워크 구현 가이드
│   ├── SYSTEM_ARCHITECTURE.md  # 시스템 아키텍처 설계서
│   └── MODULE_IMPLEMENTATION.md # 모듈 구현 가이드
├── scripts/             # 유틸리티 스크립트
└── tests/              # 테스트

```

## 🛠️ 시스템 요구사항

### 하드웨어
- **컴퓨터**: NVIDIA Jetson AGX Orin 또는 동급
- **카메라**: ZED 2i 스테레오 카메라
- **모터**: Dynamixel 서보모터
- **기타**: IMU, 초음파 센서, 스피커/마이크

### 소프트웨어
- Ubuntu 22.04 LTS
- ROS2 Humble
- CUDA 11.4+
- OpenCV 4.5+
- PCL 1.12+
- Python 3.10+

## 🚀 빠른 시작

### 1. 의존성 설치
```bash
# ROS2 설치
sudo apt update
sudo apt install ros-humble-desktop

# 필수 패키지 설치
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-rtabmap-ros \
                 ros-humble-zed-ros2-wrapper

# 개발 도구
sudo apt install python3-colcon-common-extensions \
                 python3-rosdep \
                 build-essential \
                 cmake
```

### 2. 빌드
```bash
# 워크스페이스 생성
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your-username/modular-v.git

# 의존성 설치
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --symlink-install
```

### 3. 실행
```bash
# 환경 설정
source ~/ros2_ws/install/setup.bash

# 1. TurtleBot3 모터 시스템 시작
ros2 launch turtlebot3_bringup robot.launch.py

# 2. RTAB-Map SLAM (2D 맵 생성 활성화)
ros2 launch rtabmap_launch rtabmap.launch.py \
  args:="--delete_db_on_start" \
  Grid.FromDepth:=true \
  Grid.CellSize:=0.05 \
  Grid.3D:=false

# 3. Modular-V 시스템
ros2 launch modular_v system_bringup.launch.py

# 개별 모듈 실행
ros2 launch zed_wrapper zed_camera.launch.py  # ZED 카메라만
```

## 📖 문서

상세한 구현 가이드는 다음 문서를 참조하세요:

- **[프레임워크 가이드](docs/FRAMEWORK_GUIDE.md)**: 전체 시스템 구현 및 설정
- **[시스템 아키텍처](docs/SYSTEM_ARCHITECTURE.md)**: 상세 설계 및 구조
- **[모듈 구현 가이드](docs/MODULE_IMPLEMENTATION.md)**: 각 모듈별 구현 방법

## 🔧 설정

### 카메라 설정 (config/sensor_config.yaml)
```yaml
camera:
  resolution: 2  # HD720
  fps: 15
  depth_mode: 1  # ULTRA
  depth_min: 0.3
  depth_max: 20.0
```

### RTAB-Map 설정 (modules/perception/rtabmap_module/config/rtabmap_config.yaml)
```yaml
# 2D Grid 맵 생성 (Nav2용)
grid:
  from_depth: true
  cell_size: 0.05        # 5cm 해상도
  range_max: 5.0         # 5m 범위
  max_obstacle_height: 2.0  # 2m 이하 장애물
  3d: false              # 2D 그리드 생성
```

### 내비게이션 설정 (config/navigation_params.yaml)
```yaml
controller:
  max_vel_x: 1.0  # m/s
  max_vel_theta: 1.0  # rad/s
  xy_goal_tolerance: 0.25  # m
```

## 🧪 테스트

```bash
# 단위 테스트
colcon test --packages-select modular_v

# 통합 테스트
ros2 run modular_v integration_test

# 시뮬레이션 테스트
ros2 launch modular_v simulation.launch.py
```

## 🐳 Docker 지원

```bash
# Docker 이미지 빌드
docker build -t modular-v:latest .

# 컨테이너 실행
docker-compose up
```

## 📊 모니터링

```bash
# 시스템 상태 확인
ros2 topic echo /system/status

# 시각화
rviz2 -d config/modular_v.rviz

# 로그 확인
ros2 topic echo /rosout
```

## 🤝 기여

기여를 환영합니다! Pull Request를 제출하기 전에 다음을 확인해주세요:

1. 코드 스타일 가이드 준수
2. 단위 테스트 통과
3. 문서 업데이트

## 📄 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 📞 문의

- 이슈: [GitHub Issues](https://github.com/your-username/modular-v/issues)
- 이메일: your-email@example.com

## 🙏 감사의 말

이 프로젝트는 다음 오픈소스 프로젝트들을 활용합니다:
- ROS2
- RTAB-Map
- Nav2
- ZED SDK