# parking_auto1 — 회전 시작점 접근·주차 전용 PD

최신 main `96ec051` 병합 후 주차 전용 PD 추가. 라바콘 변경·기본 노출 600 포함. FREE 발견 시점과 실제 회전 시작 시점 분리 유지.

흐름: 기존 main 주행 → parking 표지판 → 서행 탐색 → FREE 목표 저장 → 회전 시작점까지 접근 → 초기 우회전 → 흰 경계선 PD 정렬 → 회전 중 관측한 흰 직선 소실 시 정지.

## 추가 동작

- `parking_approach`: 목표 입구 중앙·폴리곤을 차체 상대 좌표로 저장, 후속 FREE/UNKNOWN 판정에 의한 목표 교체 없음
- 접근 거리: 입구 중앙의 전방 좌표 − 회전 선행 거리. 선행 거리 = `wheelbase / abs(tan(turn_steering)) + approach_start_offset` (현재 약 0.548 m)
- `/odom`의 속도·각속도와 제어 주기 시간으로 상대 목표 이동 보정. 절대 odom 위치·TF·엄격한 센서 시각 동기화 관문 미사용
- 스캔 시각부터 현재까지의 이동량도 최신 속도로 짧은 등속 근사 보정
- 남은 거리 2 cm 이내에서 기존 −0.35 rad 우회전 시작. 이미 시작점을 6 cm 넘게 지난 FREE 후보는 건너뛰고 탐색 유지
- 접근 15초/1.2 m 제한, 기존 장애물·센서 최신성·취소·무이동 정지 유지
- 접근 중 흰 선 소실은 완료 조건에서 제외. 회전 시작 이후 관측한 선의 소실만 기존 정지 조건으로 사용

## 주차 전용 PD 추가

- FREE/UNKNOWN 기준·LiDAR 군집·흰 선 추출 기준은 이전과 동일. 인식 실패 개선은 이번 범위에서 제외
- 진입 초반 기존 −0.35 rad 우회전. 칸 방향과의 상대 각도가 약 40도 안쪽으로 들어오면 PD 개입 가능
- 기존 main 카메라 장착 모델로 흰 선을 바닥 좌표로 변환. 뒷축 앞 0.55 m의 중앙 오차(m)와 선 방향 오차(rad) 사용
- 서로 평행하고 도색 칸 폭 약 0.50 m인 경계 쌍 우선. 한 선만 보이면 저장한 칸 방향·폭을 이용한 좌우 식별. 네 경계 동시 관측 불필요
- 장애물 사이 폭 대신 도색 폭 사용. 장애물을 벌려도 카메라가 본 도색 중앙으로 보정
- `error = lateral_m + heading_weight * heading_rad`. 주차 전용 PID 인스턴스 사용, 기본 kp=1.5/ki=0/kd=0.08인 PD 동작. main 차선 PID 상태 공유 없음
- 정렬될수록 고정 우회전에서 PD 출력으로 점진 전환. 중앙·방향 오차가 작으면 조향 0 근처로 복귀
- 조향 변화율 0.8 rad/s 제한. 큰 반대 조향 전환 중 잠시 전진 유보 후 조향 방향이 바뀌면 재개
- 한 번 PD를 사용한 후 유효 경계 관측이 0.30초 이상 없으면 정지 대기, 새 관측에서 재개. PD 개입 전 선을 못 찾으면 기존 고정 우회전 유지
- 흰 직선 소실 정지·실제 장애물 정지·기존 시간/거리 제한 유지. 흰 선 소실은 차체 전체 진입 확인과 별개

PD 설정은 `params.py`의 `parking.pd`. `enabled=False`이면 이전 고정 우회전 방식으로 비교 시험 가능. 카메라 높이·피치·화각이 실제 장착과 맞아야 하며, 실차 gain·시작점 보정 검증 필요.

로그: `PARKING_PD_BOUNDARY_ALIGNMENT` 보정 중, `PARKING_PD_STEERING_TRANSITION` 반대 조향 전환 중, `WAITING_FOR_PARKING_LINE_FEEDBACK` 사용하던 경계 관측 대기.

## 설정

`ericar_driver/ericar_driver/params.py`의 parking 구역.

| 값 | 기본 | 의미 |
|---|---:|---|
| approach_start_offset | 0 m | 양수: 더 일찍 회전, 음수: 더 늦게 회전 |
| approach_start_tolerance | 0.02 m | 회전 시작 허용 오차 |
| approach_passed_tolerance | 0.06 m | 이보다 많이 지난 후보는 건너뜀 |
| approach_timeout | 15 s | 목표 저장 후 최대 접근 시간 |
| approach_distance_limit | 1.2 m | 목표 저장 후 최대 접근 주행 거리 |

회전 선행 거리는 90도 원호의 근사값이며, 실측 회전 반경과 다를 수 있음. 차선 미침범·차체 전체 진입 보장 없음. 장애물의 좌우 이격 변화·슬립·LiDAR 가림·odom 오차는 실차 확인 필요. 특히 FREE가 너무 늦게 보이면 후보를 건너뛸 수 있음.

## 디버그

기존 카메라·흰 선·LiDAR 화면 유지. PD에 사용한 선과 목표 십자는 하늘색, `PARK PD`에 선택 선 종류·중앙/방향 오차·조향 표시. `control.parking.feedback`에서 p/i/d 및 blend 확인. 저장한 목표 칸은 노랑, 접근 중 회전 시작점은 하늘색 십자와 `TURN IN`으로 표시. `control.parking.approach`에서 remaining_m, turn_lead_m, travel_m 확인.

## 리모 다운로드·빌드

주행·센서 노드 종료 후 실행.

```bash
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash
mkdir -p ~/parking_auto1_ws/src
git clone --single-branch --branch parking_auto1 https://github.com/Aninnom/ERICAR.git ~/parking_auto1_ws/src/ERICAR
cd ~/parking_auto1_ws
colcon build --packages-select ericar_driver
source install/setup.bash
```

첫 번째 SSH 창: 센서·디버그.

```bash
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash
source ~/parking_auto1_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
ros2 launch ericar_driver bringup.launch.py driver:=false debug:=true record:=false exposure:=800
```

두 번째 SSH 창: 주차 단독 시험. 실행 직후 서행 가능, 조종기 정지 준비 필요. 모터 발행자 0 확인 후 실행.

```bash
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash
source ~/parking_auto1_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=1
ros2 topic info /limo/ack_cmd
ros2 launch ericar_driver parking_auto1.launch.py model:=/home/wego/models/best.pt
```

전체 코스는 동일 브랜치에서 bringup 기본 driver=true로 실행. 주차 전용 노드와 중복 실행 금지.

## 검증

```bash
python3 -m pytest ericar_driver/test/test_parking_2.py ericar_driver/test/test_parking_auto1.py ericar_driver/test/test_parking_pd.py -q
```

raycast 장면의 빈틈 전방 위치 4종, UNKNOWN→FREE 접근, FREE 목표 유지, 이미 지난 후보, 상대 회전·실측 속도, 지연 스캔 보정, 장애물·시간/거리·취소 정지, fake ROS main 모터 명령 연결 확인. 실차 성공률 검증은 별도 필요.

추가 검증: 카메라 정·역투영, 부분 경계, 장애물 간격과 도색 중앙 분리, PD 방향·조향 풀기·변화율 제한, 관측 만료·재개, main 모터 발행(fake ROS), 합성 카메라+자전거 운동학 폐루프. 폐루프 4장면은 직선 정렬 구간의 이상적 장면이며 전체 주차 성공률·실차 선 미침범 보장 아님. 결과 `ericar_driver/reports/parking_pd_verification.json`.
