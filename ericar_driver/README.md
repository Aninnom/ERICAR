> **parking_2 브랜치:** parking 표지판 → 서행 FREE 검색 → 고정 우회전 → 흰 직선 소실 정지. [설치·동작·설정](../docs/parking_2.md). 이전 경로 계획 방식 사용하지 않음.

# ericar_driver

실차 주행 노드. 멘토 ORDA `answer.py`(hello-osy/2026-orda-mentoring, 613c9b4)를 역할별 파일로 분리. 노드 하나 구조 유지.

## 파일과 담당

| 파일 | 내용 | 담당 |
|---|---|---|
| `lane_vision.py` | 차선 인식 (BEV + sliding window, 좌우 판단, 중앙선 목표점) | 혁준, 소민 |
| `lane_driving.py` | 목표점 pure pursuit (목표점 없을 때 PID) | 혁준, 소민 |
| `parking.py` | FREE 1회·고정 우회전·흰 선 소실 정지 | 정윤 |
| `lidar.py` | 라이다 공통 (차체 기준 점, 경로 여유) | 유경 |
| `cones.py` | 라바콘 | 유경 |
| `lane_change.py` | 3차선 정적 장애물 (라이다 감지 후 차선 변경) | 유경 |
| `node.py` | ROS 입출력: 센서 수신, YOLO, `/limo/ack_cmd` 발행 | 공통 |
| `loop.py` | 제어 주기 1회 계산 (센서 상태 → 차선 → 판단), 설정 검사. ROS 의존 없음 | 공통 |
| `driver.py` | 표지판·정지선 상태, 멈춤 방지, 최종 결정 | 공통 |
| `section.py` | 구간 관리자: 우회전 코너를 세어 구간(S0~S4)별 활성 미션 결정 | 공통 |
| `common.py` | 값 제한, PID | 공통 |
| `params.py` | 주행 파라미터. 담당 구역만 수정 | 각자 |
| `config/robot.yaml` | 토픽, 차량·카메라·라이다 치수 | 공통 |

공통 파일은 수정 전 팀 공유.

지그재그(서행 유도선)·보행자·신호등·서행·갈림길은 `driver.py`와 `lane_vision.py` 안에 있음.

## 실차 개발 도구 (`tools/`)

| 도구 | 용도 | 실행 (차, 터미널 환경 1) |
|---|---|---|
| `calibrate_camera.py` | 카메라 높이·기울기 보정 (직선 차로 사진), 줄자로 정지선 거리 검증 | `python3 tools/calibrate_camera.py --capture 5` / `--capture 3 --verify 0.30` |
| `debug_view.py` | 카메라 + BEV 차선 마스크·곡선 + 상태·제어값을 브라우저로 | `bringup.launch.py`가 자동 실행. 단독: `python3 tools/debug_view.py` → `http://<차 IP>:8080` |
| `bag_timeline.py` | 녹화 주행 구간 시계열 (거리·방향·조향·오차·검출 선) | `python3 tools/bag_timeline.py ~/bags/<이름>` |
| `bag_frames.py` | 녹화 지정 시각 카메라 + BEV 창·곡선·목표점 한 장 | `python3 tools/bag_frames.py ~/bags/<이름> 29.0,30.6` → `/tmp/frames.jpg` |
| `replay_bag.py` | 녹화 영상을 지금 코드에 다시 넣어 결과 CSV (수정 전후 비교) | `python3 tools/replay_bag.py ~/bags/<이름> > after.csv` |

차선만 시험할 때는 YOLO 없이 실행 (표지판 미션 비활성, 모델 불필요)

```bash
ros2 launch ericar_driver bringup.launch.py yolo:=false
```

## 회귀 시험 (맥·차, ROS 불필요)

공통 파일 수정 후 동작이 바뀌지 않았는지 확인. 도면 치수로 합성한 3차로 영상·라이다·표지판으로 보행자 → 차선 변경 → 신호등 → 주차까지 노드 전체 실행 후 기준값과 비교 (약 15초)

```bash
python3 ericar_driver/test/test_regression.py
```

numpy, opencv, pyyaml 필요. 시험 장면의 노면 색은 `params.py` 차선 색 범위 중앙값이라 색 설정이 바뀌어도 장면이 따라감. 동작을 의도적으로 바꾼 PR은 `python3 test/scenario.py --write`로 기준값 갱신 후 이유를 PR에 기록.

## 원본 대비 실차 수정

- 시뮬 시계(`use_sim_time`) 고정 제거
- 명령: `/drive`의 `AckermannDriveStamped` → `/limo/ack_cmd`의 `AckermannDrive`
- 카메라 토픽 `/camera/color/image_raw`, 최대 조향 0.42 rad
- 발행 직전 속도·조향 한계 적용
- 설정·모델 경로를 인자로 받음. 기본값 `config/robot.yaml`, `~/models/best.pt`

그 외 로직은 원본과 동일 (분리 시 정의 17개 글자 단위 일치 확인).

## 실행 (차, 터미널 환경 3 dl_env)

```bash
cd ~/autorace_ws && colcon build --packages-select ericar_driver && source install/setup.bash
```

### 한 번에 실행 (권장): `bringup.launch.py`

바퀴(`limo_base_ackerman`)·라이다·카메라(노출 300 자동 설정)·주행 노드·디버그 화면을 한 줄로 실행. 종료는 Ctrl+C 한 번.

```bash
ros2 launch ericar_driver bringup.launch.py
```

| 인자 | 기본값 | 내용 |
|---|---|---|
| `yolo` | `true` | `false`면 YOLO 없이 차선·라이다만 (모델 불필요) |
| `driver` | `true` | `false`면 센서만 (주행 노드 없이, 녹화·보정용) |
| `debug` | `true` | 디버그 화면 (포트 8080) |
| `record` | `false` | `true`면 `~/bags/run_<월일_시분>` 녹화 (`bag:=` 로 경로 지정) |
| `exposure` | `300` | 카메라 수동 노출 |
| `model`, `config` | `~/models/best.pt`, `config/robot.yaml` | 주행 노드로 전달 |

예: 차선만 + 녹화 `ros2 launch ericar_driver bringup.launch.py yolo:=false record:=true`

- 디버그 화면: 맥에서 `ssh -L 8080:localhost:8080 wego@<차 IP>` 후 `http://localhost:8080`
- 시작 로그에 `[bringup] 카메라 노출: data=300` 확인
- 카메라 `Failed to setup devices` 오류 한 줄은 단독 실행 때도 나오는 메시지, 영상은 정상
- wego `teleop_launch.py`와 같이 켜지 말 것 (차동 모드 드라이버라 바퀴 드라이버 중복)

### 따로 실행

센서·드라이버를 직접 띄운 경우 주행 노드만:

```bash
ros2 launch ericar_driver driver.launch.py model:=/home/wego/models/best.pt
```

조종기 SWB 위(자동)일 때만 차가 움직임. 상태는 `ros2 topic echo /answer/status`.

### 구간 관리자 (미션 순서)

미션은 표지판 순서가 아니라 **지금 구간**으로 켜짐 (`section.py`). 출발 뒤 누적 회전이 −65°·−155°·−245°·−335°를 넘을 때마다 구간 +1 (코너 4개가 모두 우회전 90°, 여유 25°). 회전은 `/imu` 자이로 (0.3초 넘게 안 오면 `/odom` 회전 × 0.93). 표지판 하나를 놓쳐도 다음 구간 미션은 그대로 동작.

| 구간 | 위치 | 활성 미션 |
|---|---|---|
| S0 | 출발 직선 | 서행 표지판 |
| S1 | 1번 코너 뒤 | 보행자 (서행 유도선은 카메라로 상시) |
| S2 | 2번 코너 뒤 | 3차선 차선 변경, 갈림길 |
| S3 | 3번 코너 뒤 | 라바콘 (지금은 꺼 둠, 아래 참고) |
| S4 | 4번 코너 뒤 | 신호등, 주차 |

**라바콘은 실차 확인 전이라 기본값에서 꺼 둠** (`params.py` `sections.off: ['cones']`). 담당(유경)이 확인 뒤 목록에서 빼면 S3에서 켜짐. 대회 전 반드시 확인.

주행 노드가 켜진 채로 차를 조종기로 몰거나 들어서 돌리면 그 회전도 누적됨 → 차를 출발 위치에 둔 뒤 노드를 켬 (연속 2바퀴째는 내내 S4).

지금 구간은 `/answer/detections`의 `control.stage`·`section`·`section_distance`(구간 내 주행 거리)·`yaw_source`(`imu`/`odom`)로 확인. 구간과 무관하게 미션을 단독 시험하려면 `params.py` `sections.enabled` False.

### 재출발 (미션 시작 지점에서 다시 출발)

규정상 미션 실패·포기 시 차를 그 미션(또는 다음 미션) 시작 지점으로 옮김. 노드를 다시 켤 때 `start_stage`로 시작 구간을 지정 (생략 시 `slow` = 처음부터).

```bash
ros2 launch ericar_driver driver.launch.py model:=/home/wego/models/best.pt start_stage:=fork
```

| 차를 놓는 곳 | `start_stage` | 구간 |
|---|---|---|
| 출발선, 서행(방지턱) | `slow` | S0 |
| 서행 유도선, 보행자 표지판 구간 | `pedestrian` | S1 |
| 3차선 장애물, 갈림길 | `fork` | S2 |
| 라바콘 | `cones` | S3 |
| 신호등 | `traffic` | S4 |
| 주차 | `parking` | S4 |

예전과 다른 점: 라바콘은 `traffic`이 아니라 `cones`, 서행 유도선은 `pedestrian`.

### 멈추지 않기 (규정: 15초 넘게 서면 그 미션 0점)

- 정지선 접근 시간 초과(`approach_timeout`): 영구 정지(`fault`) 대신 차선 주행 계속. 그 미션은 포기로 표시, 다음 미션은 구간이 바뀌면 켜짐
- 정지선이 아직 안 보이면 멈추지 않고 `stop_approach_speed`로 다가가며 찾음
- 표지판 검출(YOLO)이 끊김(예외·`detector.timeout` 초과): 표지판 미션만 꺼지고 차선·라이다 주행 계속. 시작 직후 첫 검출만 `detector.startup_wait`(20초)까지 기다림
- 신호등 정지 중 검출이 `stall.seconds`(8초) 넘게 끊기거나 `traffic_max_wait`(60초) 넘게 녹색을 못 보면 포기하고 진행
- 차선 변경 대기(옆 차로 막힘)·차선 변경 중 안전 조향 없음이 8초 넘으면 차선 변경을 1.5 m 동안 끄고 차선 주행
- 전방 비상 정지가 8초 넘게 안 풀리면 라이다 점을 피하는 조향이 있을 때만 0.08 m/s로 비켜 감 (없으면 계속 정지)
- 차선 놓침은 `lost_search`(0.15 m/s로 2 m 탐색 후 정지) 그대로
- parking_2: FREE 1회에 고정 우회전 진입. 관측하던 흰 직선 소실 시 정지
- parking_2: 실제 경로 장애물·센서 끊김이면 정지. 시간 경과로 장애물을 무시하고 진행하지 않음

시험 (맥·차, 약 2분): 표지판 누락·검출 끊김·대기 탈출을 합성 시나리오로 확인

```bash
python3 ericar_driver/test/test_sections.py
```

## 실행 전 확인

- YOLO 모델 클래스 8종 이름이 `params.py`의 `YOLO_CLASSES`와 정확히 같아야 실행됨
- `config/robot.yaml`의 "실측 필요" 값 (카메라 높이·기울기는 차선 인식에 직접 영향)
- 차선 색: `params.py`의 `yellow_min/max`는 노란색만 검출. 정지선·노면 표시(흰색)는 아직 노란 마스크 사용 (분리 작업 예정)
- `/odom`·카메라·라이다 헤더 시간이 시스템 시계와 맞아야 함. 어긋나면 "waiting for fresh sensors"에서 정지
