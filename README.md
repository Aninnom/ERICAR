# ERICAR — 2026 AutoRace

LIMO Pro(Ackermann) 자율주행 코드. Ubuntu 22.04 + ROS 2 Humble 기준.

## 구조

| 패키지 | 내용 | 담당 |
|---|---|---|
| `autorace_msgs` | 공용 메시지 (`docs/topic_spec.md` 기준) | 공용 |
| `autorace_camera` | 카메라 인지 | 혁준 |
| `autorace_lidar` | 라이다 인지 + E-stop | 유경 |
| `autorace_decision` | 판단 (미션 FSM) | 소민 |
| `autorace_control` | 제어 (`/limo/ack_cmd` 유일 발행) | 정윤 |
| `autorace_bringup` | 런치 + YAML 파라미터 | 공용 |
| `autorace_sim` | 2D 시뮬레이터 (실차 대신 토픽 발행) | 혁준 |
| `docs/` | 토픽 명세, 실차 체크리스트, 결정 기록 | 공용 |

센서·모터 드라이버는 차량에 기본 탑재 (`ros2 launch wego teleop_launch.py`). 우리 몫은 그 토픽을 구독하는 인지 → 판단 → 제어.

## 처음 한 번

```bash
sudo apt install ros-humble-ackermann-msgs ros-humble-cv-bridge python3-opencv
```

```bash
mkdir -p ~/autorace_ws/src
```

```bash
cd ~/autorace_ws/src && git clone <레포 주소> ericar
```

`~/.bashrc`에 각자 다른 `ROS_DOMAIN_ID` 설정 (같은 와이파이에서 토픽 섞임 방지). 혁준 1, 유경 2, 소민 3, 정윤 4.

```bash
echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc
```

## 빌드·실행

```bash
cd ~/autorace_ws && colcon build --symlink-install
```

```bash
source ~/autorace_ws/install/setup.bash
```

```bash
ros2 launch autorace_bringup race.launch.py
```

새 터미널마다 `source` 필요.

## Git 규칙

- 각자 브랜치(`hyeokjun`, `yugyeong`, `somin`, `jeongyun`)에서 작업, 동작 확인 후 `main`에 merge
- `main`은 항상 빌드되는 상태 유지
- 공용 폴더(`autorace_msgs`, `autorace_bringup`, `docs`) 수정 시 단톡에 먼저 공유
- `build/ install/ log/`는 업로드 제외 (`.gitignore`)
