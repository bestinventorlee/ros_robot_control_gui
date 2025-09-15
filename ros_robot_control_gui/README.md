# 6축 로봇 제어 GUI

ESP32 마스터 컨트롤러와 연동되는 6축 로봇 제어 GUI 시스템입니다.

## 기능

### 1. 각도 제어
- **기본 각도 제어**: 6개 서보의 각도를 직접 입력하여 제어
- **속도/가속도 포함 각도 제어**: 각도와 함께 속도, 가속도를 설정하여 제어
- **프리셋 동작**: 미리 정의된 동작 패턴 실행

### 2. 좌표 제어
- **기본 좌표 제어**: X, Y, Z 위치와 Roll, Pitch, Yaw 자세를 입력하여 제어
- **속도/가속도 포함 좌표 제어**: 좌표와 함께 속도, 가속도를 설정하여 제어
- **좌표 프리셋**: 미리 정의된 좌표 위치로 이동

### 3. 동기화 설정
- **전역 동기화 설정**: 모든 서보의 기본 속도, 가속도, 타임아웃 설정
- **개별 서보 설정**: 각 서보별로 개별적인 속도, 가속도 설정

### 4. 상태 모니터링
- **실시간 서보 상태**: 각 서보의 현재 각도, 목표 각도, 오차 표시
- **동기화 완료 결과**: 마스터로부터 받은 상세한 동기화 결과 표시
  - 현재 각도, 목표 각도, 실제 사용된 속도, 실제 사용된 가속도
  - 각 서보별 정확도 및 상태 표시
- **시스템 로그**: 제어 명령 및 상태 변화 로그 표시

## 설치 및 실행

### 1. 의존성 설치
```bash
sudo apt update
sudo apt install python3-tk
pip3 install rclpy
```

### 2. 패키지 빌드
```bash
cd ~/ros2_ws
colcon build --packages-select ros_robot_control_gui
source install/setup.bash
```

### 3. GUI 실행
```bash
ros2 launch ros_robot_control_gui robot_control.launch.py
```

또는 직접 실행:
```bash
ros2 run ros_robot_control_gui robot_control_gui.py
```

## ROS2 토픽

### 퍼블리시 토픽
- `servo_angles` (std_msgs/Float32MultiArray): 기본 각도 제어
- `robot_coords` (std_msgs/Float32MultiArray): 기본 좌표 제어
- `servo_angles_with_speed` (std_msgs/Float32MultiArray): 속도/가속도 포함 각도 제어
- `robot_coords_with_speed` (std_msgs/Float32MultiArray): 속도/가속도 포함 좌표 제어
- `sync_settings` (std_msgs/Float32MultiArray): 동기화 설정

### 구독 토픽
- `servo_status` (std_msgs/Float32MultiArray): 서보 상태 피드백 및 동기화 완료 결과
  - 기본 상태: 18개 데이터 (각 서보당 3개: 현재각도, 목표각도, 오차)
  - 동기화 결과: 24개 데이터 (각 서보당 4개: 현재각도, 목표각도, 실제속도, 실제가속도)

## 메시지 형식

### 각도 제어 메시지
```
servo_angles: [angle1, angle2, angle3, angle4, angle5, angle6]
servo_angles_with_speed: [angle1, angle2, angle3, angle4, angle5, angle6, speed, acceleration]
```

### 좌표 제어 메시지
```
robot_coords: [x, y, z, roll, pitch, yaw]
robot_coords_with_speed: [x, y, z, roll, pitch, yaw, speed, acceleration]
```

### 동기화 설정 메시지
```
sync_settings: [speed, acceleration, timeout, min_ratio]
```

### 서보 상태 메시지
```
servo_status: [servo1_current, servo1_target, servo1_error, ..., servo6_current, servo6_target, servo6_error]
```

### 동기화 완료 결과 메시지
```
servo_status: [servo1_current, servo1_target, servo1_speed, servo1_accel, ..., servo6_current, servo6_target, servo6_speed, servo6_accel]
```

## 사용법

1. **각도 제어**: "각도 제어" 탭에서 각 서보의 목표 각도를 입력하고 "각도로 이동" 버튼 클릭
2. **좌표 제어**: "좌표 제어" 탭에서 목표 위치와 자세를 입력하고 "좌표로 이동" 버튼 클릭
3. **속도/가속도 설정**: 각 제어 탭에서 속도와 가속도를 설정하여 부드러운 동작 제어
4. **동기화 설정**: "동기화 설정" 탭에서 전역 또는 개별 서보 설정 조정
5. **상태 모니터링**: "상태 모니터링" 탭에서 실시간 서보 상태 및 동기화 완료 결과 확인
6. **결과 분석**: 동기화 완료 후 각 서보의 실제 성능 데이터 분석

## 안전 기능

- **긴급 정지**: 🚨 긴급 정지 버튼으로 모든 서보를 즉시 0도로 이동
- **홈 포지션**: 🏠 홈 포지션 버튼으로 안전한 홈 위치로 이동
- **값 검증**: 입력값 범위 검증 및 자동 보정

## 문제 해결

### GUI가 실행되지 않는 경우
- Python3-tk가 설치되어 있는지 확인
- ROS2 환경이 올바르게 설정되어 있는지 확인

### 서보가 움직이지 않는 경우
- ESP32 마스터 컨트롤러가 실행 중인지 확인
- CAN 통신이 정상적으로 작동하는지 확인
- 서보 전원이 공급되고 있는지 확인

### 연결 오류가 발생하는 경우
- 네트워크 연결 상태 확인
- ROS2 에이전트가 실행 중인지 확인
- 토픽 이름이 올바른지 확인
