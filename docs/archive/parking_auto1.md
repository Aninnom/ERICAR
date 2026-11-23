# parking_auto1 — 일반속도 탐색·원호 유지·주차 PD

이전 PD 버전 `8101a7d`에서 발전한 실험 버전. 기존 main 차선 제어·LiDAR FREE 판정은 유지.

## 동작 순서

1. 신호등 완료 등 기존 주차 진입 조건 충족 후 parking 표지판 확인
2. `parking_cruise`: 기존 main 차선 주행 명령 유지, 동시에 LiDAR 빈칸 탐색. 일반 코스 최대 0.4 m/s, 실제 속도는 기존 차선 제어·센서 최신성·장애물 검사로 제한
3. 가까워진 표지판이 연속 새 YOLO 결과에서 사라지거나, LiDAR 후보가 감속 거리 안에 오면 0.12 m/s 서행으로 전환
4. FREE 목표 저장, `parking_approach`에서 회전 시작점까지 접근. 속도가 0.16 m/s 이하가 된 뒤 회전 진입
5. `parking_turn`의 `arc` 단계: 기본 −0.35 rad 원호 유지. 계획 칸 경계와 카메라 도색 경계의 차이만 작은 범위로 보정. 선이 순간적으로 맞아 보여도 직진으로 전환 금지
6. 충분한 회전각·입구 접근·연속 경계 관측 확인 후 `align` 단계 진입. 약 85도까지 최소 우회전 유지, 그 뒤 PD로 점진 조향 풀기
7. 관측하던 흰 선이 사라지고, 약 85도 회전·차체 후단 입구 통과가 확인되면 정지

## 서행 시점 분리

- 표지판 최초 확인 시 LiDAR 탐색 즉시 시작. main 차선 PID 상태 초기화 없이 같은 제어기 사용
- 가까움 근거: 정규화 상자 면적 0.012 이상, 또는 오른쪽 화면 중심 0.90 이상이고 면적 0.006 이상. 화면 가장자리 잘린 표지판도 통과 관측용으로 사용 가능
- 가까움 근거를 얻은 후 최소 3개 새 추론 결과·최소 0.35초 미검출에서 통과 확정. 같은 결과 반복 조회·오래된 검출·먼 표지판 일시 누락은 통과 근거에서 제외
- `parking_cruise`에서는 YOLO 계속 실행. 서행 탐색·회전 단계에서는 기존대로 추론 휴식
- FREE를 먼저 찾으면 목표 저장 후 계속 추적. UNKNOWN 후보도 회전 시작점이 가까우면 감속 근거로 사용하되 FREE로 승격하지 않음
- 감속 기준 거리 = 현재 속도 × 반응시간 + (현재 속도² − 서행 속도²)/(2 × 감속도) + 여유 거리. 기본 반응 0.25초·감속도 0.30 m/s²·여유 0.10 m
- 표지판이 계속 보여도 목표 감속 거리에 오면 서행 우선. 반대로 가까움 근거를 전혀 얻지 못하고 후보도 없으면 최대 30초/8 m 뒤 실패 정지
- 일반 주행 속도 제한·센서 최신성·실제 차체 예상 궤적의 장애물 정지 유지. 감속 중에는 실제 현재 속도까지 고려한 궤적 검사

## 회전 시작점·원호·정렬

회전 선행 거리 = `wheelbase / abs(tan(turn_steering)) + approach_start_offset` (기본 약 0.548 m).

- 양수 시작 보정은 더 일찍, 음수 보정은 더 늦게 회전. 시작 허용 오차 2 cm, 6 cm 이상 지난 후보는 건너뜀
- `/odom` 속도로 상대 목표 이동 보정. 회전 속도는 기존 main의 IMU 우선·미수신 시 보정된 odom 사용. 절대 pose·TF·엄격한 시각 동기화 관문 없음
- 원호 보정은 카메라 관측 선과 저장한 목표 선의 차이에 비례. 남은 회전각을 기본 원호와 중복 보정하지 않음
- 기본 원호 조향 −0.35 rad, 보정 최대 ±0.12 rad, 우회전 최소 크기 0.23 rad. 최대 조향은 실차 한계 ±0.42 rad
- 정렬 전환: 누적 우회전 최소 약 65도, 관측 선 방향 오차 20도 이내·중앙 오차 18 cm 이내·뒷축이 입구 5 cm 근처까지 접근, 이 조건을 3개 새 영상에서 연속 확인
- 정렬 단계에서도 약 85도 회전·선 방향 및 목표 칸 방향 오차 약 8.6도 이내까지 최소 0.20 rad 우회전 유지. 조건 충족 후에만 완전한 조향 풀기 허용
- PD는 main 차선 PID와 분리. 기본 kp=1.5, ki=0, kd=0.08. 바닥 좌표에서 뒷축 앞 0.55 m 중앙 오차와 선 방향 오차 사용. 부분 한 선도 저장한 칸 방향·도색 폭 0.50 m로 좌우 식별
- 흰 선 조기 소실만으로 완료 금지. 정렬 전 선이 사라지면 제한된 원호를 유지하고, 완료는 회전각·입구 통과 조건까지 확인. 정렬 후 사용하던 경계 관측이 불확실하면 기존 관측 대기 정지 적용
- 최대 회전 12초/1.05 m. 기존 실제 장애물·센서·취소·무이동 정지 유지

추정 자세·도색 소실은 완전한 주차 판정과 별개. 측정 오차·슬립·카메라 장착 변화가 있으면 실차 검증·보정 필요. 이상적인 합성 장면 통과로 실차 선 미침범·완주 보장 불가. 시작점을 4 cm 일찍 잡은 스트레스 장면은 최소 측면 여유 약 0.3 mm로 여유 부족, 실차 시작점 보정 필요. ±2 cm 장면의 최소 측면 여유는 약 1.5 cm 이상인 이상적 모델 결과.

## 설정·디버그

설정: `ericar_driver/ericar_driver/params.py`의 parking 구역.

| 설정 | 기본 | 의미 |
|---|---:|---|
| cruise_search_enabled | True | 표지판 최초 확인 후 일반속도 탐색 |
| cruise_max_speed | 0.4 m/s | 일반 코스 탐색 상한 |
| trial_cruise_max_speed | 0.2 m/s | 주차 단독 launch 탐색 상한 |
| sign_near_area | 0.012 | 가까운 표지판 면적 근거 |
| sign_gone_frames / seconds | 3 / 0.35 s | 새 추론 결과의 연속 미검출 |
| approach_start_offset | 0 m | 회전 시작점 앞뒤 보정 |
| turn_entry_max_speed | 0.16 m/s | 회전 진입 실제 속도 상한 |
| arc_min_angle / release_angle | 약 65 / 85도 | 정렬 진입 / 완전 조향 풀기 최소 회전 |
| arc_align_frames | 3 | 정렬 조건 연속 확인 영상 수 |
| pd.arc_kp | 2.0 | 원호의 계획 경계 대비 보정 비례계수 |
| pd.arc_correction_limit | 0.12 rad | 원호 중 보정 상한 |

화면: `PARK CRUISE LiDAR ON` 일반 주행 중 탐색, `TURN IN` 회전 시작점까지 남은 거리, `ARC` 원호 유지, `ALIGN` 정렬. 누적 회전각과 `released`로 조향 풀기 허용 여부 표시. 하늘색 선·십자는 제어기가 사용한 도색 경계·목표.

텔레메트리 `control.parking`: sign(가까움·통과·미검출 수), slow_reason(서행 전환 이유), cruise_distance_m, approach, turn_phase, turn_angle_deg, arc_released, feedback, white_line.

## 리모 업데이트·실행

기존 센서·주행 종료 후:

```bash
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash
cd ~/parking_main_ws/src/ERICAR
git status --short
# 로컬 변경 없을 때 실행
git switch parking_auto1
git pull --ff-only origin parking_auto1
cd ~/parking_main_ws
colcon build --packages-select ericar_driver
source install/setup.bash
```

전체 코스 (센서·주행·화면 함께 실행, 준비 후 실제 움직임 가능):

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=1
ros2 launch ericar_driver bringup.launch.py driver:=true debug:=true record:=false exposure:=800
```

주차만 시험할 때는 첫 SSH 창에서 위 명령의 `driver:=false`로 센서만 실행. 두 번째 SSH 창에 같은 ROS 환경을 source한 뒤 실행:

```bash
ros2 topic info /limo/ack_cmd
# 발행자 0일 때 실행. 조종기 물리 정지 준비 필요
ros2 launch ericar_driver parking_auto1.launch.py model:=/home/wego/models/best.pt
```

주차 단독 시험은 표지판 전 서행, 표지판 확인 후 탐색 상한 0.2 m/s. 전체 코스의 탐색 상한 0.4 m/s와 구분. 두 주행 launch 동시 실행 금지. exposure:=800은 이전 실험값 지정이며 bringup 기본값은 600.

## 검증

```bash
python3 -m pytest ericar_driver/test/test_parking_2.py ericar_driver/test/test_parking_auto1.py ericar_driver/test/test_parking_pd.py ericar_driver/test/test_parking_cruise_arc.py -q
```

표지판 일시 누락·새 추론 결과 구분·가까운 표지판 통과·UNKNOWN 후보 감속·FREE 저장·실제 속도 감속 확인·main 모터 발행(fake ROS)·원호 유지·정렬 전환·조기 완료 방지·합성 카메라/자전거 모델과 회전 차체의 선 교차 검사 포함.

결과: `ericar_driver/reports/parking_cruise_arc_verification.json`. 이전 버전 보고서는 해당 이전 코드의 기록으로 유지. 실차 성공률은 별도 확인 필요.
