# ERICAR — 2026 AutoRace

LIMO Pro(Ackermann) 자율주행 코드. Ubuntu 22.04 + ROS 2 Humble, 실차 전용.

## 구조

| 경로 | 내용 |
|---|---|
| `ericar_driver/` | 주행 노드 하나. 멘토 ORDA `answer.py` 기반, 역할별 파일 분리. 파일별 담당은 [`ericar_driver/README.md`](ericar_driver/README.md) |
| `docs/` | 토픽, 실차 체크리스트, 결정 기록 |

담당: 혁준·소민 차선 주행, 정윤 주차, 유경 라이다(라바콘·3차선).

센서·차량 드라이버는 차량 기본 탑재 패키지 사용. 실행 순서는 [`docs/car_checklist.md`](docs/car_checklist.md).

## 처음 한 번 (차)

```bash
mkdir -p ~/autorace_ws/src && cd ~/autorace_ws/src && git clone https://github.com/Aninnom/ERICAR.git ericar
```

YOLO 모델은 레포 제외 (`*.pt`). 차의 `~/models/best.pt`에 둠.

## 빌드·실행 (차, 터미널 환경 3 dl_env)

```bash
cd ~/autorace_ws && colcon build --symlink-install && source install/setup.bash
```

```bash
ros2 launch ericar_driver driver.launch.py model:=/home/wego/models/best.pt
```

새 터미널마다 `source ~/autorace_ws/install/setup.bash` 필요. 조종기 SWB 위(자동)일 때만 주행.

## Git 규칙

- 각자 브랜치(`hyeokjun`, `yugyeong`, `somin`, `jeongyun`)에서 작업, 실차 확인 후 PR로 `main`에 merge
- 자기 담당 파일과 `params.py`의 자기 구역만 수정
- 공통 파일(`node.py`, `driver.py`, `common.py`, `config/robot.yaml`, `docs/`) 수정 시 단톡에 먼저 공유
- `main`은 항상 차에서 빌드되는 상태 유지
