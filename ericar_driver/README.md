> **parking_auto2:** 최신 main 통합·일반속도 LiDAR 탐색 유지·기본 우회전 유지·흰 선 소실 첫 새 영상 즉시 정지. [업데이트·실행·검증](../docs/parking_auto2.md).


# ericar_driver

실차 주행 노드. 멘토 ORDA `answer.py`(hello-osy/2026-orda-mentoring, 613c9b4)를 역할별 파일로 분리. 노드 하나 구조 유지.

## 파일과 담당

| 파일 | 내용 | 담당 |
|---|---|---|
| `lane_vision.py` | 차선 인식 (BEV + sliding window, 좌우 판단, 중앙선 목표점) | 혁준, 소민 |
| `lane_driving.py` | 목표점 pure pursuit (목표점 없을 때 PID) | 혁준, 소민 |
| `parking.py` | FREE 1회·고정 우회전·흰 선 소실 정지 | 정윤 |
| `parking_control.py`·`parking_gap.py`·`parking_sign.py`·`parking_white.py` | 주차 보조: 흰 경계선 PD, 라이다 빈칸 입구 추정, 표지판 통과 관측, 흰 도색 선 정렬 | 정윤 |
| `lidar.py` | 라이다 공통 (차체 기준 점, 경로 여유, 섹터 거리) | 유경 |
| `cones.py` | 라바콘: 좌·우 콘 쌍(문) 사이로 조향 | 유경 |
| `lane_change.py` | 3차선 정적 장애물: 라이다 감지 → 좌·우 판단 → 원호–직선–원호 이동, 표지판 요청 이동 경로 계획 | 유경, 소민, 혁준 |
| `fork.py` | 갈림길 상태: 방향 기억, 목표 차로, 입구 판단 | 소민, 혁준 |
| `traffic_lamp.py` | 신호등 박스 안 램프 위치로 색 재확인 | 혁준 |
| `node.py` | ROS 입출력: 센서 수신, YOLO, `/limo/ack_cmd` 발행 | 공통 |
| `loop.py` | 제어 주기 1회 계산 (센서 상태 → 차선 → 판단), 설정 검사. ROS 의존 없음 | 공통 |
| `driver.py` | 표지판·정지선 상태, 멈춤 방지, 최종 결정 | 공통 |
| `slow_zone.py` | 서행·방지턱: 표지판·거리로 감속, IMU 기울기로 방지턱 통과 확인 뒤 해제 | 혁준 |
| `section.py` | 구간 관리자: 우회전 코너를 세어 구간(S0~S4)별 활성 미션 결정 | 공통 |
| `common.py` | 값 제한, PID | 공통 |
| `params.py` | 주행 파라미터. 담당 구역만 수정 | 각자 |
| `config/robot.yaml` | 토픽, 차량·카메라·라이다 치수 | 공통 |

공통 파일은 수정 전 팀 공유.

지그재그(서행 유도선)·보행자·신호등 정지는 `driver.py`와 `lane_vision.py` 안에 있음. 서행은 `slow_zone.py`, 갈림길은 `fork.py`.

## 실차 개발 도구 (`tools/`)

| 도구 | 용도 | 실행 (차, 터미널 환경 1) |
|---|---|---|
| `calibrate_camera.py` | 카메라 높이·기울기 보정 (직선 차로 사진), 줄자로 정지선 거리 검증 | `python3 tools/calibrate_camera.py --capture 5` / `--capture 3 --verify 0.30` |
| `debug_view.py` | 미션 전체 디버그 화면: 카메라(YOLO 상자·차선·정지선) + BEV(차선·흰색 노면 표시) + 라이다(군집·명령 경로·3차선/라바콘 판단) + 상태·속도 0 이유 + 구간·미션 8개 진행 + 사건 기록. 화면 글자는 영어, 풀이는 아래 "디버그 화면 용어" | `bringup.launch.py`가 자동 실행. 단독: `python3 tools/debug_view.py` → `http://<차 IP>:8080` |
| `bag_timeline.py` | 녹화 주행 구간 시계열 (거리·방향·조향·오차·검출 선) | `python3 tools/bag_timeline.py ~/bags/<이름>` |
| `bag_frames.py` | 녹화 지정 시각 카메라 + BEV 창·곡선·목표점 한 장 | `python3 tools/bag_frames.py ~/bags/<이름> 29.0,30.6` → `/tmp/frames.jpg` |
| `replay_bag.py` | 녹화 영상을 지금 코드에 다시 넣어 결과 CSV (수정 전후 비교) | `python3 tools/replay_bag.py ~/bags/<이름> > after.csv` |
| `bag_video.py` | 녹화 → 카메라 영상 mp4 (상태·조향·붉은 비율 글자 포함), 작아서 노트북으로 복사 가능 | `/usr/bin/python3 tools/bag_video.py ~/bags/<이름>` |
| `red_check.py` | 라바콘 붉은색 판정 확인 (비율·색 분포, 판정 사진 저장) | `bringup` `driver:=false` 상태에서 `/usr/bin/python3 tools/red_check.py` |
| `lc_random_eval.py` | 3차선 임의 배치(장애물 7대) + 갈림길 합성 평가 (맥, ROS 불필요) | `LC_ONLY=fix45 python3 tools/lc_random_eval.py "$PWD" 50 8` |
| `corn_test.py` | 라바콘 단독 실차 시험 (차선·표지판·구간 없이 `cones.py`만, `sections.off`와 무관). 기록 `~/bags/corn_test_<시각>.csv` | 터미널 1 `ros2 launch ericar_driver bringup.launch.py driver:=false record:=true`, 터미널 2 `python3 tools/corn_test.py` (`--dry`: 명령 없이 인식만). 접속부터 기록 복사까지 칠 명령 전체는 파일 머리말 |

### 디버그 화면 용어

화면 글자는 영어 (OpenCV 글꼴에 한글 없음, 한글 글꼴 없는 차 브라우저에서도 같은 화면)

| 화면 표시 | 뜻 |
|---|---|
| 카메라 상자 초록 `ok` / 노랑 `1/2` | 확정된 표지판 / 확인 중 (본 횟수/필요 횟수) |
| 카메라 상자 회색 `cut`·`small`·`cooldown`·`stale` | 판단에서 제외: 화면 끝에 잘림·너무 작음·재반응 대기·오래된 검출 |
| 카메라 초록·파랑 선, 빨간 점, 자홍 선 | 왼쪽·오른쪽 차선, 조향 목표점, 정지선 (BEV 결과를 카메라 화면에 다시 투영) |
| BEV 자홍 화소, 노란·회색 가로선 | 흰색 노면 표시, 채택한 정지선·후보 (`cut` 잘림, `thick` 너무 두꺼움) |
| 라이다 빨강·주황 점 | 넓은 물체(차량·벽), 라바콘 후보 |
| 라이다 하늘색 선, 붉은 부채꼴 | 명령 조향으로 갈 경로, 비상 정지 범위 |
| 라이다 사각형 3개와 숫자 | 3차선 차로별 빈 거리(m). 초록 = 선택한 차로, 빨강 = 부족 |
| `controller ... for N s` | 내부 상태와 지속 시간 |
| `age image/scan/yolo` | 센서 값이 몇 초 전 것인지. `signs LOST` = 표지판 검출 끊김 |
| `flags` | 지금 걸린 조건: 코너 유지, 서행 표지판, 지그재그 서행, 갈림길, 차로 변경 꺼짐, 차선 놓침 |
| `stall guard` | 의도하지 않은 정지 누적 시간 / 강제 전환 기준 |
| `SPEED 0: ...` | 명령 속도가 0인 이유 |
| `SECTION S2 ... turned / yaw` | 지금 구간, 구간 내 거리, 이 구간에서 우회전한 각도, 출발 뒤 누적 회전 |
| 미션 `ACTIVE`/`ARMED`/`DONE` | 진행 중 / 이 구간에서 대기 / 완료 또는 포기 |
| 미션 `MISSED`/`passed`/`OFF`/`ON` | 구간을 지났는데 표지판을 못 봄 / 지나감(라이다 미션) / `sections.off`로 꺼짐 / 항상 켜짐 |
| `EVENTS` | 상태·구간 전환, 표지판 확정, 센서 끊김, 막힘, 정지 원인 (최신이 위) |

차선만 시험할 때는 YOLO 없이 실행 (표지판 미션 비활성, 모델 불필요)

```bash
ros2 launch ericar_driver bringup.launch.py yolo:=false
```

## 회귀 시험 (맥·차, ROS 불필요)

공통 파일 수정 후 동작이 바뀌지 않았는지 확인. 도면 치수로 합성한 3차로 영상·라이다·표지판으로 보행자 → 차선 변경 → 신호등 → 주차까지 노드 전체 실행 후 기준값과 비교 (약 15초)

```bash
python3 ericar_driver/test/test_regression.py
```

numpy, opencv, pyyaml 필요. 시험 장면의 노면 색은 `params.py` 차선 색 범위 중앙값이라 색 설정이 바뀌어도 장면이 따라감. 동작을 의도적으로 바꾼 PR은 `python3 test/scenario.py --write`로 기준값 갱신 후 이유를 PR에 기록. 지금 기준값은 맥 생성본 (PR #67).

3차선·갈림길을 바꿨으면 아래도 실행 (맥)

```bash
python3 ericar_driver/test/three_lane.py
```

```bash
python3 ericar_driver/test/test_lane_change_heading.py
```

- `three_lane.py`: 합성 사례 21건 (실차 실패 재현 사례 포함). 11/17 기준 21/21
- `tools/lc_random_eval.py`: 임의 배치 50건. 11/17 기준 46/50 (실패 시드 18·21·27·44)
- 합성 시험은 '나빠지지 않음' 정도의 근거. 11/17 새벽 실차 실패 중 합성으로 재현된 것은 1건
- 알려진 실패 2건 (기능 문제 아님, 기준 정리 필요): `test_sections.py` 주차 표지판 누락 1건, `test_lidar_missions.py` 임의 3차선

## 원본 대비 실차 수정

- 시뮬 시계(`use_sim_time`) 고정 제거
- 명령: `/drive`의 `AckermannDriveStamped` → `/limo/ack_cmd`의 `AckermannDrive`
- 카메라 토픽 `/camera/color/image_raw`, 최대 조향 0.42 rad
- 발행 직전 속도·조향 한계 적용
- 설정·모델 경로를 인자로 받음. 기본값 `config/robot.yaml`, `~/models/best.pt`

그 외 로직은 원본과 동일 (분리 시 정의 17개 글자 단위 일치 확인).

## 실행 (차)

작업 공간: `~/ericar_main_ws`(`main` 확정본), `~/ericar_dev_ws`(브랜치 시험). 한 터미널에는 하나만 `source`

```bash
cd ~/ericar_main_ws && colcon build --base-paths src/ericar --packages-select ericar_driver
```

```bash
source /opt/ros/humble/setup.bash && source ~/wego_ws/install/setup.bash && source ~/ericar_main_ws/install/setup.bash
```

`params.py`를 바꾸면 다시 빌드해야 반영됨.

### 한 번에 실행 (권장): `bringup.launch.py`

바퀴(`limo_base_ackerman`)·라이다·카메라(노출 600 자동 설정)·주행 노드(YOLO 포함, GPU)·디버그 화면을 한 줄로 실행. 종료는 Ctrl+C 한 번. 녹화는 `record:=true`를 붙여야 남음.

```bash
ros2 launch ericar_driver bringup.launch.py
```

| 인자 | 기본값 | 내용 |
|---|---|---|
| `yolo` | `true` | `false`면 YOLO 없이 차선·라이다만 (모델 불필요) |
| `driver` | `true` | `false`면 센서만 (주행 노드 없이, 녹화·보정용) |
| `debug` | `true` | 디버그 화면 (포트 8080) |
| `record` | `false` | `true`면 `~/bags/run_<월일_시분>` 녹화 (`bag:=` 로 경로 지정) |
| `exposure` | `600` | 카메라 수동 노출. 600에서 약 16.6 Hz, 300은 약 30 Hz |
| `start_stage` | `slow` | 시작 구간 (아래 "재출발") |
| `model`, `config` | `~/models/best.pt`, `config/robot.yaml` | 주행 노드로 전달 |

예: 차선만 + 녹화 `ros2 launch ericar_driver bringup.launch.py yolo:=false record:=true`

- 디버그 화면: 맥에서 `ssh -L 8080:localhost:8080 wego@<차 IP>` 후 `http://localhost:8080`
- 시작 로그에 `[bringup] 카메라 노출: data=600` 확인
- 카메라 `Failed to setup devices` 오류 한 줄은 단독 실행 때도 나오는 메시지, 영상은 정상
- wego `teleop_launch.py`와 같이 켜지 말 것 (차동 모드 드라이버라 바퀴 드라이버 중복)

### 따로 실행

센서·드라이버를 직접 띄운 경우 주행 노드만:

```bash
ros2 launch ericar_driver driver.launch.py model:=/home/wego/models/best.pt
```

조종기 SWB 위(자동)일 때만 차가 움직임. SWB가 위에 있는 채로 노드를 켜면 켜지자마자 출발 → 아래(수동)로 두고 켠 뒤 로그에 `lane`이 뜨면 올림. 상태는 `ros2 topic echo /answer/status`.

### 구간 관리자 (미션 순서)

미션은 표지판 순서가 아니라 **지금 구간**으로 켜짐 (`section.py`). 출발 뒤 누적 회전이 −65°·−155°·−245°·−335°를 넘을 때마다 구간 +1 (코너 4개가 모두 우회전 90°, 여유 25°). 회전은 `/imu` 자이로 (0.3초 넘게 안 오면 `/odom` 회전 × 0.93). 표지판 하나를 놓쳐도 다음 구간 미션은 그대로 동작. 라바콘 통과 중에는 구간 전환 보류 (콘 배치가 우회전을 강제해도 S3 유지, 끝난 뒤 4번 코너에서 S4).

| 구간 | 위치 | 활성 미션 |
|---|---|---|
| S0 | 출발 직선 | 서행 표지판 |
| S1 | 1번 코너 뒤 | 보행자 (서행 유도선은 카메라로 상시) |
| S2 | 2번 코너 뒤 | 3차선 차선 변경, 갈림길 |
| S3 | 3번 코너 뒤 | 라바콘 |
| S4 | 4번 코너 뒤 | 신호등, 주차 |

라바콘은 켜져 있음 (`params.py` `sections.off: []`). 라바콘이 3번 코너 안에서 시작하면 S3로 먼저 넘김 (`sections.cone_advance`).

주행 노드가 켜진 채로 차를 조종기로 몰거나 들어서 돌리면 그 회전과 주행 거리도 누적됨 → 차를 출발 위치에 **구간 방향과 나란히** 둔 뒤 노드를 켬 (연속 2바퀴째는 내내 S4). 시작 자세가 틀어져 있으면 3차선 진입 정렬(`entry_align`)과 방향 보정이 그만큼 어긋남. 3차선에서 수동 주행이 섞이면 구간 거리가 부풀어 `lane_change_max_distance`(6.0 m)에서 차선 변경이 꺼짐.

지금 구간은 `/answer/detections`의 `control.stage`·`section`·`section_distance`(구간 내 주행 거리)·`yaw_source`(`imu`/`odom`)로 확인. 구간과 무관하게 미션을 단독 시험하려면 `params.py` `sections.enabled` False.

### 재출발 (미션 시작 지점에서 다시 출발)

규정상 미션 실패·포기 시 차를 그 미션(또는 다음 미션) 시작 지점으로 옮김. 노드를 다시 켤 때 `start_stage`로 시작 구간을 지정 (생략 시 `slow` = 처음부터).

```bash
ros2 launch ericar_driver bringup.launch.py record:=true start_stage:=fork
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

### 3차선·갈림길 설정 (11/17, PR #67)

차선 변경은 원호–직선–원호 이동. 표지판 방향이 확정되면 목적 차로 쪽 이동을 출발 전에 경로로 그려 라이다 점과 여유를 확인 (두 칸 떨어진 목적 차로는 45°로 한 번에). 흐름은 [`docs/three_lane.md`](../docs/three_lane.md), 주행 결과는 이슈 #68.

켜 둔 것 (실차·녹화 근거 있음)

| 설정 (`params.py`) | 내용 |
|---|---|
| `lane_change.request_extend` | 한 칸 이동 중 표지판이 확정돼 목적 차로가 두 칸이면 경로 확인 뒤 두 칸으로 넓힘 |
| `lane_change.short_arc_margin` 0.08 | 짧은 S자 복귀 시작 여유. 0.06은 지나침, 0.10은 못 미침 |
| `lane_change.entry_align` | 3차선 진입 뒤 1.5 m 안에서 구간 방향보다 왼쪽 12° 넘게 틀어져 있으면 오른쪽 조향 |
| `lane_change.heading_bias_min_distance` 1.0 | 구간 전환 뒤 1 m 안(코너 출구)은 누적 회전 치우침을 배우지 않음 |
| `missions.fork_sign_bearing` | 20° 넘게 틀어진 자세여도 화살표 박스의 도로 기준 방위가 섬 표지판 쪽이면 인정 |
| `detector.minimum_area_by_class` left·right 0.003 | 멀리서 화살표 방향을 틀리게 읽는 것 방지. 확정은 기둥 약 1.6~2.6 m 전 |

구현만 하고 꺼 둔 것 (실차 근거 없음, 이유는 `params.py` 주석. 켤 때는 실차 확인 후)

`request_reverse`, `request_blocked_escape`, `safe_flip_block`, `back_end`, `lane_memory`, `backup`(후진 재시도), `camera_finish_center`

현장에서 맞출 값: `sections.lane_change_max_distance` 6.0, `missions.fork_enter_distance` 5.4, `lane_change.request_entrance_distance` 5.2 (3차선 진입 ~ 갈림길 기둥 거리 기준, 연습 트랙 실측값). 한계: 차로를 바꿔야 하는 장애물 줄 간격 1.5 m 미만 (#64).

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
- 차선 색: `params.py`의 `yellow_min/max`는 노란색(차선), `white_min/max`는 흰색(정지선·횡단보도·3차선 점선). 현장 바닥(포맥스 + 빛반사 필름)에서 두 색이 구분되는지 디버그 화면으로 확인
- `/odom`·카메라·라이다 헤더 시간이 시스템 시계와 맞아야 함. 어긋나면 "waiting for fresh sensors"에서 정지
