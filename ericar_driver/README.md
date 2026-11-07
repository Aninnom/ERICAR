# ericar_driver

실차 주행 노드. 멘토 ORDA `answer.py`(hello-osy/2026-orda-mentoring, 613c9b4)를 역할별 파일로 분리. 노드 하나 구조 유지.

## 파일과 담당

| 파일 | 내용 | 담당 |
|---|---|---|
| `lane_vision.py` | 차선 인식 (BEV + sliding window) | 혁준, 소민 |
| `lane_driving.py` | 차선 PID | 혁준, 소민 |
| `parking.py` | T 주차 | 정윤 |
| `lidar.py` | 라이다 공통 (차체 기준 점, 경로 여유) | 유경 |
| `cones.py` | 라바콘 | 유경 |
| `lane_change.py` | 3차선 정적 장애물 (라이다 감지 후 차선 변경) | 유경 |
| `node.py` | ROS 입출력: 센서 수신, YOLO, `/limo/ack_cmd` 발행 | 공통 |
| `loop.py` | 제어 주기 1회 계산 (센서 상태 → 차선 → 판단), 설정 검사. ROS 의존 없음 | 공통 |
| `driver.py` | 미션 순서·센서 상태·최종 결정 | 공통 |
| `common.py` | 값 제한, PID | 공통 |
| `params.py` | 주행 파라미터. 담당 구역만 수정 | 각자 |
| `config/robot.yaml` | 토픽, 차량·카메라·라이다 치수 | 공통 |

공통 파일은 수정 전 팀 공유.

지그재그(서행 유도선)·보행자·신호등·서행·갈림길은 `driver.py`와 `lane_vision.py` 안에 있음.

## 회귀 시험 (맥·차, ROS 불필요)

공통 파일 수정 후 동작이 바뀌지 않았는지 확인. 도면 치수로 합성한 3차로 영상·라이다·표지판으로 보행자 → 차선 변경 → 신호등 → 주차까지 노드 전체 실행 후 기준값과 비교 (약 15초)

```bash
python3 ericar_driver/test/test_regression.py
```

numpy, opencv, pyyaml 필요. 동작을 의도적으로 바꾼 PR은 `python3 test/scenario.py --write`로 기준값 갱신 후 이유를 PR에 기록.

## 원본 대비 실차 수정

- 시뮬 시계(`use_sim_time`) 고정 제거
- 명령: `/drive`의 `AckermannDriveStamped` → `/limo/ack_cmd`의 `AckermannDrive`
- 카메라 토픽 `/camera/color/image_raw`, 최대 조향 0.42 rad
- 발행 직전 속도·조향 한계 적용 (주차 시간표 −0.5 rad 등 한계 초과 방지)
- 설정·모델 경로를 인자로 받음. 기본값 `config/robot.yaml`, `~/models/best.pt`

그 외 로직은 원본과 동일 (분리 시 정의 17개 글자 단위 일치 확인).

## 실행 (차, 터미널 환경 3 dl_env)

차량 드라이버·라이다·카메라를 먼저 실행 (`docs/car_checklist.md`).

```bash
cd ~/autorace_ws && colcon build --packages-select ericar_driver && source install/setup.bash
```

```bash
ros2 launch ericar_driver driver.launch.py model:=/home/wego/models/best.pt
```

조종기 SWB 위(자동)일 때만 차가 움직임. 상태는 `ros2 topic echo /answer/status`.

## 실행 전 확인

- YOLO 모델 클래스 8종 이름이 `params.py`의 `YOLO_CLASSES`와 정확히 같아야 실행됨
- `config/robot.yaml`의 "실측 필요" 값 (카메라 높이·기울기는 차선 인식에 직접 영향)
- 차선 색: `params.py`의 `white_min/max`는 흰색만 검출
- `/odom`·카메라·라이다 헤더 시간이 시스템 시계와 맞아야 함. 어긋나면 "waiting for fresh sensors"에서 정지
