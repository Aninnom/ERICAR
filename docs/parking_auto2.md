# parking_auto2 — 흰 선 소실 즉시 정지·기본 우회전 유지

기준: 최신 main `b1c0387`을 먼저 받아 새 브랜치 생성, parking_auto1 `55d5b9f`의 일반속도 탐색 기능 통합. main의 신호등 박스 선별·라바콘·3번 코너·실측 주차 반경 변경 유지.

## 수정 이유

사용자 실차 2회에서 주차칸 관통 정지, 1회에서 조향 부족·선 침범 보고. 이전 auto1의 흰 선 소실 뒤 85도 회전·입구 통과를 추가로 요구한 조건은 즉시 정지 요구와 충돌. PD가 원호를 느슨하게 만드는 경로도 존재. 해당 두 동작 변경.

## 최우선 정지

- 회전 중 흰 직선을 한 번 이상 실제로 관측한 뒤, 첫 새 영상에서 선이 사라지면 같은 제어 주기에 속도 0 명령
- 회전각·입구 통과·PD 상태·정렬 단계 조건으로 정지 유예 없음. legacy `white_stop_heading`과 auto1 `_finish_pose` 관문 제거
- 선이 사라진 프레임에서는 PD·조향·장애물 진행 판단보다 정지 우선. 정지 후 선이 다시 나타나도 자동 재출발 금지
- 같은 카메라 프레임의 반복 조회를 새 미검출로 처리하지 않음. 오래된 카메라·센서는 기존 최신성 검사로 정지
- 처음부터 흰 선을 한 번도 못 봤으면 완료 처리 금지, 기존 1.5초 미관측 실패 정지 유지
- 선이 계속 보이는 경우에는 최신 main의 칸 방향 정면 정지(`turn_done_heading=0.05 rad`)를 보조 종료로 유지

기본 `white_loss_frames=1`. 한 프레임의 인식 누락에도 일찍 멈출 수 있으나, 이번 버전은 계속 전진해 칸을 관통하는 것보다 정지 우선. 정지 명령 시점과 물리적인 제동 완료는 별개. `parked`는 정지 조건 도달 상태이며 완전한 차체 진입 확인과는 구분.

## 조향과 회전 시작점

- 기본 우회전 −0.35 rad 유지. 회전 중 실제 진행 명령은 이보다 약해지거나 0·좌회전으로 풀리지 않도록 제한
- 최신 main의 실측 회전 반경 0.615 m를 회전 시작점 계산에 사용. 이전 축간거리/tan(조향)의 약 0.548 m와 달라, 기본값 기준 약 6.7 cm 일찍 회전 시작점에 도달
- 기본 조향각을 임의로 키우지 않음. 반경 0.615 m는 −0.35 rad 실측에 맞춘 값으로 보존
- main의 실차 근거에 따라 PD 기본값은 꺼짐 유지. 이유: 경계 연계 실패·연계 끊김 대기 경로의 실차 검증 부족
- PD를 선택적으로 켜도 현재 주차 진행 명령은 기본 우회전보다 강해지는 보정만 허용, 최대 조향 ±0.42 rad 한계 유지
- 따라서 auto2는 정지 전 핸들을 풀며 자유롭게 중앙 정렬하는 버전이 아님. 카메라 관측·PD 계산·진단은 유지하되 진행 명령의 원호 유지가 우선
- 실제 장애물·센서·무이동·취소 정지 유지. main의 최대 회전 13초/1.2 m 유지

실제 조향·반경은 바닥·타이어·차량에 따라 달라질 수 있음. 선 미침범·완전 주차 보장 불가. 다음 실차에서는 명령 조향과 실제 움직임, 회전 시작 위치를 함께 확인 필요.

## 유지한 일반속도 탐색

parking 표지판 최초 확인 → 기존 main 차선 주행+LiDAR 탐색 → 가까운 표지판이 새 YOLO 결과에서 연속으로 사라지거나 후보 감속 거리에 도달 → 서행 → FREE 목표 접근 → 기본 원호 우회전 → 정지.

일반 코스 탐색 상한 0.4 m/s, 주차 단독 시험 상한 0.2 m/s, 서행·회전 0.12 m/s. FREE 판정·군집·흰 선 추출 기준은 그대로 유지. 자세한 일반속도 탐색 기준은 이전 문서의 해당 설명 참고하되, auto1의 회전각 기반 흰 선 정지 유예 설명은 auto2에 적용하지 않음.

## 리모 업데이트·빌드

센서·주행 종료 후, 기존 workspace에서 실행:

```bash
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash
cd ~/parking_main_ws/src/ERICAR
git status --short
# 로컬 변경 없을 때 진행
git fetch origin refs/heads/parking_auto2:refs/remotes/origin/parking_auto2
if git show-ref --verify --quiet refs/heads/parking_auto2; then
  git switch parking_auto2
else
  git switch --no-track -c parking_auto2 origin/parking_auto2
fi
git pull --ff-only origin parking_auto2
cd ~/parking_main_ws
colcon build --packages-select ericar_driver
source install/setup.bash
```

첫 SSH 창 — 센서·화면:

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
ros2 launch ericar_driver bringup.launch.py driver:=false debug:=true record:=false exposure:=800
```

두 번째 SSH 창 — 같은 ROS 환경 source 후, 실제 서행 가능하므로 조종기 정지 준비 필요:

```bash
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash
source ~/parking_main_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=1
ros2 topic info /limo/ack_cmd
# 모터 발행자 0 확인 후 실행
ros2 launch ericar_driver parking_auto2.launch.py model:=/home/wego/models/best.pt
```

전체 코스는 센서 창을 끈 뒤 bringup driver=true로 한 번만 실행. 다른 parking/main 주행 노드 중복 실행 금지. exposure:=800은 이전 실험값, bringup 기본은 최신 main 값 유지.

디버그: `runtime.mode=parking_auto2`, `CALIBRATED_RIGHT_TURN` 기본 우회전, `WHITE_STRAIGHT_LINE_DISAPPEARED_STOP` 흰 선 소실 정지, `TURN_COMPLETE_STOP` 방향 보조 정지. 흰 선 `present/seen/loss_frames`, 실제 출력 속도·조향 함께 확인.

## 검증

```bash
python3 -m pytest ericar_driver/test/test_parking_2.py ericar_driver/test/test_parking_auto1.py ericar_driver/test/test_parking_pd.py ericar_driver/test/test_parking_turn_done.py ericar_driver/test/test_parking_cruise_arc.py ericar_driver/test/test_parking_auto2.py ericar_driver/test/test_traffic.py ericar_driver/test/test_traffic_stop.py -q
```

초기 회전각·입구 위치와 무관한 첫 소실 정지, 동일 프레임 구분, 시간/거리/PD보다 선 소실 우선, 정지 고정, PD 풀기 요청에 대한 실제 명령 제한, main 모터 발행(fake ROS), 최신 main 실측 반경·방향 보조 정지, 일반속도 탐색 및 신호등 단위 시험 포함.

합성 카메라 장면은 첫 선 소실에서의 정지를 검증하며, 주차칸 전체 진입 성공으로 해석하지 않음. 실제 주차 완료·선 미침범은 실차 재검증 필요. 보고서: `ericar_driver/reports/parking_auto2_verification.json`.
