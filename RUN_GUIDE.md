# 로봇 SLAM + Nav2 실행 가이드

## 시스템 구성

| 항목 | 내용 |
|------|------|
| 보드 | Raspberry Pi 5 |
| OS | Ubuntu 24.04 + ROS2 Jazzy |
| 모터 | Dynamixel XL430-W250-T x2 (U2D2 연결) |
| LiDAR | SLLidar C1 |
| IMU | BNO085 (I2C) |
| 베이스 | 지름 15cm, 높이 15cm 원기둥 |

---

## 사전 준비 (최초 1회)

```bash
# udev 규칙 설치 (USB 장치 고정 이름)
sudo cp ~/my-ros2/scripts/99-my-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# 시간 동기화 서비스
sudo apt install chrony -y
sudo systemctl enable chrony
sudo systemctl start chrony

# 필수 패키지 설치
sudo apt install ros-jazzy-robot-localization ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server

# 빌드
cd ~/my-ros2
colcon build
source install/setup.bash
```

---

## 실행 순서

### 터미널 1 — 하드웨어 노드

```bash
source ~/my-ros2/install/setup.bash
ros2 launch my_robot bringup.launch.py
```

정상 확인:
- `[left ID:1] initialized — velocity mode, torque ON`
- `[right ID:2] initialized — velocity mode, torque ON`
- `SLLidar health status : OK.`
- `IMU node started — 50.0 Hz`

### 터미널 2 — EKF + SLAM

```bash
source ~/my-ros2/install/setup.bash
ros2 launch my_robot slam.launch.py
```

정상 확인: `Configuring` → `Activating` 출력

### 터미널 3 — Nav2 (자율주행)

```bash
source ~/my-ros2/install/setup.bash
ros2 launch my_robot navigation.launch.py
```

정상 확인: `Managed nodes are active` 출력

### (선택) 데스크탑에서 RViz2 시각화

같은 WiFi + 같은 `ROS_DOMAIN_ID`의 데스크탑에서:

```bash
rviz2
```

RViz2 설정:
1. Fixed Frame → `map`
2. Add → By topic → `/map` (Map)
3. Add → By topic → `/scan` (LaserScan)
4. Add → TF
5. 상단 **"2D Goal Pose"** 버튼 → 지도 위 클릭+드래그로 목표 지정

### (선택) 키보드 수동 조종

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| 키 | 동작 |
|----|------|
| `i` | 전진 |
| `,` | 후진 |
| `j` | 좌회전 |
| `l` | 우회전 |
| `k` | 정지 |

---

## 지도 저장

매핑이 완료되면:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_room_map
```

---

## 종료 순서

각 터미널에서 `Ctrl+C`:

1. Nav2 (navigation.launch.py)
2. SLAM (slam.launch.py)
3. 하드웨어 (bringup.launch.py)

---

## Launch 파일 구조

| 파일 | 용도 | 노드 |
|------|------|------|
| `bringup.launch.py` | 하드웨어 | robot_state_publisher, motor_controller, imu_node, sllidar_node |
| `slam.launch.py` | SLAM | ekf_filter_node, slam_toolbox |
| `navigation.launch.py` | 자율주행 | controller_server, planner_server, behavior_server, bt_navigator, collision_monitor 등 |
| `motor_controller.launch.py` | 모터 단독 테스트 | motor_controller |

---

## 문제 해결

| 증상 | 원인 | 해결 |
|------|------|------|
| 모터 No status packet | 모터 전원 꺼짐 또는 케이블 불량 | 전원 확인, USB 재연결 |
| Serial error (I/O error) | USB 포트 불안정 | U2D2 USB 뽑고 3초 후 재연결 |
| /map 토픽 없음 | slam_toolbox 미활성화 | `Configuring` 메시지 확인 |
| 데스크탑에서 토픽 안 보임 | ROS_DOMAIN_ID 불일치 | 양쪽 `echo $ROS_DOMAIN_ID` 확인 |
| TF delay 큼 | 시간 동기화 불량 | `sudo chronyc makestep` |
| Nav2 collision_monitor 에러 | 설정 누락 | nav2_params.yaml의 polygons 확인 |
