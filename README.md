# ERICAR — 2026 AutoRace

LIMO Pro(Ackermann) 자율주행 코드. Ubuntu 22.04 + ROS 2 Humble, 실차 전용.

## 구조

| 경로 | 내용 |
|---|---|
| `ericar_driver/` | 주행 노드 하나. 멘토 ORDA `answer.py` 기반, 역할별 파일 분리. 파일별 담당은 [`ericar_driver/README.md`](ericar_driver/README.md) |
| `docs/` | 계획(`plan.md`), 차선 설계(`lane.md`), 3차선·갈림길(`three_lane.md`), 토픽, 실차 체크리스트, 결정 기록 |
| `CLAUDE.md` | 팀 작업 안내 (작업 흐름, 차량, 규정 핵심, 현재 상태) |

담당: 혁준·소민 차선 주행, 정윤 주차, 유경 라이다(라바콘·3차선).

현재 상태(11/17, main `2365b70`): 8개 미션 실차 동작 확인(방지턱은 연습 트랙에 없어 미검증), 완주 기록 있음. 미션별 상태와 남은 문제는 [`CLAUDE.md`](CLAUDE.md) "현재 상태", 이슈 #68·#61.

센서·차량 드라이버는 차량 기본 탑재 패키지 사용. 실행 순서는 [`docs/car_checklist.md`](docs/car_checklist.md).

## 차 작업 공간

| 경로 | 용도 |
|---|---|
| `~/ericar_main_ws` | `main` 확정본 (본선용) |
| `~/ericar_dev_ws` | 브랜치 시험용 |

차에서는 받기만 함 (수정·커밋 금지). YOLO 모델은 레포 제외 (`*.pt`), 차의 `~/models/best.pt`에 둠.

## 빌드·실행 (차)

```bash
cd ~/ericar_main_ws/src/ericar && git pull
```

```bash
cd ~/ericar_main_ws && colcon build --base-paths src/ericar --packages-select ericar_driver
```

```bash
source /opt/ros/humble/setup.bash && source ~/wego_ws/install/setup.bash && source ~/ericar_main_ws/install/setup.bash
```

```bash
ros2 launch ericar_driver bringup.launch.py record:=true
```

- 터미널을 열면 뜨는 환경 선택에서 **3 (dl_env)**. YOLO GPU 추론용 패키지(torch·ultralytics)가 `dl_env`에만 있음. 확인: `python3 -c "import torch; print(torch.cuda.is_available())"` → `True`
- 환경이 안 맞으면 노드는 안 죽고 로그에 `YOLO failed`가 찍히며 표지판 미션만 꺼짐 (차선 주행은 계속) → 디버그 화면에 표지판 상자가 뜨는지 확인
- 바퀴·라이다·카메라(노출 600)·주행 노드(YOLO 포함)·디버그 화면을 한 줄로 실행. `record:=true`를 빼면 녹화 안 함
- 조종기 SWB 아래(수동)로 두고 실행 → 로그에 `lane`이 뜨면 차를 차로와 나란히 놓고 SWB 위(자동)
- 재출발은 `start_stage:=<구간>` (`slow`·`pedestrian`·`fork`·`cones`·`traffic`·`parking`). 인자 표·구간 설명은 [`ericar_driver/README.md`](ericar_driver/README.md)
- `params.py`를 바꾸면 다시 빌드해야 반영됨

## Git 규칙

- 작업마다 브랜치를 만들어 PR로 `main`에 merge. `main` 직접 커밋, `push --force`, `reset --hard` 금지
- 자기 담당 파일과 `params.py`의 자기 구역만 수정. 다른 사람 담당은 먼저 알린 뒤 수정
- 공통 파일(`node.py`, `driver.py`, `common.py`, `config/robot.yaml`, `docs/`) 수정 시 단톡에 먼저 공유
- `main`은 항상 차에서 빌드되는 상태 유지
- 동작을 바꾼 PR은 회귀 기준값을 갱신하고 이유를 PR에 기록 (`ericar_driver/README.md` "회귀 시험")
