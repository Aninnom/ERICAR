# fast_parking1

기준: parking_auto2 e96c660. 사용자가 성공을 보고한 회전·흰 선 소실 정지 보존. 실차 속도 상승 검증은 별도 필요.

## 동작

- parking 확인 즉시 기존 차선 제어 + LiDAR 빈칸 탐색 병행
- 표지판 통과만으로 감속하지 않고 기존 차선 명령 유지. 일반 상한 0.4 m/s, 주차 단독 시험 상한 0.2 m/s. 차선·장애물 정지 조건 우선
- FREE 목표 저장 또는 가까운 UNKNOWN 후보의 회전 시작점까지 남은 거리와 감속 거리 비교. FREE 판정 완화 없음
- 감속 거리 = v×0.25 + max(0,v²−0.12²)/(2×0.30) + 0.10 m. v는 실제 속도와 상한 적용 요구 속도 중 큰 값. 0.2/0.3/0.4 m/s에서 약 0.193/0.301/0.443 m
- 감속 거리 진입 시 0.12 m/s 요구. 실제 속도 0.16 m/s 이하 확인 뒤 회전 시작. 시작점을 이미 지난 후보는 건너뜀
- 가까운 표지판 통과 후 목표·유효 후보가 계속 없으면 1.0초 또는 0.35 m 중 먼저 도달한 조건에서 저속 재탐색. 먼 후보가 있으면 일반속도 탐색 유지, 전체 탐색 시간·거리 상한 유지
- 실측 반경 0.615 m, 기본 조향 −0.35 rad, 회전 속도 0.12 m/s, PD 기본 끔 유지
- 회전 중 본 흰 직선이 첫 새 영상에서 사라지면 즉시 정지·정지 상태 유지. 각도 조건 추가 없음

## 설정

`params.py`의 parking 구역: `slow_on_sign_passed=False`, `no_target_cruise_seconds=1.0`, `no_target_cruise_distance=0.35`.
기존 표지판 통과 즉시 서행 동작은 `slow_on_sign_passed=True`로 복원 가능.

## 규정

v3 PDF 2쪽: 기본 0.3~0.4 m/s 권고, 서행 최대 0.2 m/s. 8~9쪽: 방지턱·서행 유도선은 기본 속도의 절반 수준. 주차의 별도 수치 속도 제한 없음. 서행 구간에서 일반속도 유지 용도로 사용 금지. 점수 우선·동점 시 랩타임, 주차는 차체 네 모서리 기준 판정. 흰 선 소실은 성공 보증이 아닌 정지 휴리스틱.

## 리모 실행

기존 주행 노드 종료 후 센서 1세트와 모터 명령 발행자 1개만 실행 필요.

```bash
cd ~/parking_main_ws/src/ERICAR
git fetch origin refs/heads/fast_parking1:refs/remotes/origin/fast_parking1
git switch fast_parking1 || git switch -c fast_parking1 origin/fast_parking1
git merge --ff-only origin/fast_parking1
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash
cd ~/parking_main_ws
colcon build --packages-select ericar_driver
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
```

센서 창:

```bash
ros2 launch ericar_driver bringup.launch.py driver:=false debug:=true record:=false exposure:=800
```

동일 환경 설정을 한 새 창의 주차 단독 시험(접근 상한 0.2 m/s, 표지판 전 서행):

```bash
ros2 launch ericar_driver fast_parking1.launch.py model:=/home/wego/models/best.pt
```

전체 미션 주행(접근 상한 0.4 m/s, 기존 미션 조건·차선 명령 적용):

```bash
ros2 launch ericar_driver driver.launch.py model:=/home/wego/models/best.pt yolo:=true
```

위 주행 명령 둘 중 하나만 실행. 단독 시험은 전체 코스의 고속 접근 시험과 구분.

## 검증 범위

합성 감속 모델의 가속·감속, 표지판 통과, 목표 저장, 목표 미확인 복귀, 센서 지연 정지와 기존 주차·신호등 회귀 확인. 실제 제동력·LiDAR 누락·차체 최종 위치·랩타임은 실차 검증 필요. 합성 통과를 실차 주차 성공률로 해석 금지.

## 최신 main 병합 검증 (11/15)

main `8cd83d8` → merge `d597565`. 주차 제어 파일 변경 없음. 주차·차선 변경 경계·라바콘 출구 시험 114개 통과, 패키지 빌드 통과. 확대 시험 128개 통과·4개 실패. 신호등 정지 오프셋 시험 2개는 순수 main에서도 동일 실패. 전체 코스 합성 시험 2개는 주차 상태 미진입으로 실패. 이 중 test_missing_signs는 순수 main에서도 같은 상태 목록·주차 미진입 실패 재현. 전체 코스 통과·실차 성공 보장과 구분.
