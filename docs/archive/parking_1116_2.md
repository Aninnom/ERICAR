# parking_1116(2)

기준 parking_1116 58f3722. 사용자 보고: 좌우 정상, 약 4회 중 1회 안쪽 끝선 침범. 가로선 접근 조기 정지 우려 및 fast_parking 이전 정상 정지 이력 반영.

## 비교 및 변경

- parking_auto2 e96c660과 parking_1116 비교: _white·회전 선 소실 종료·고정 조향·속도는 동일. 큰 차이는 일반속도 접근 및 거리 기반 감속. 실제 원인 확정 없이 접근 잔류 속도의 편차를 줄이는 실험
- 감속 거리와 접근 속도 곡선에 회전 진입 안정화 여유 0.05 m 추가. 정지 위치를 5 cm 앞당긴 설정이 아님
- 실제 속도 0.105~0.135 m/s에서 0.25초 연속 확인 후 회전 시작. 이전 기준은 절대 속도 0.16 m/s 이하 한 번 확인. 접근 중부터 연속 확인, 강제 정지·대기 동작 없음
- 순간 속도 이탈·센서 pause 시 확인 시간 초기화. 늦게 감속해 시작점을 이미 0.06 m 넘겨 지난 경우 기존 후보 건너뛰기 유지
- 회전 시작 offset 0, 반경 0.615 m, 조향 −0.35 rad, 고정 0.12 m/s, PD 기본 끔 유지
- 원래 선 소실 정지와 방향 보조 정지 코드 변경 없음. 가로선 가까워짐·새 거리 임계값·시간표 정지 추가 없음. 가로선 전용 검출·4 cm 조기 회전·가변 회전 속도 미도입
- 일반 접근 상한 0.4, 단독 시험 상한 0.2 m/s 및 parking_1116의 단조 감속 유지

## 검증·한계

관련 합성 시험 125개·빌드 통과. 0.2/0.3/0.4 m/s 이상적 감속, 단일 저속 샘플 거부, 안정화 중 전진, 속도 이탈·센서 pause 초기화, 가까운 가로선만으로 정지하지 않음 확인. parking_auto2의 _white 및 회전 종료 코드 AST 비교 동일 확인.

실차 끝선 여유·실제 제동력·odom 속도 정확도는 미검증. 기존 관측 약 1/4 실패는 사용자 보고이며 이번 성공률 수치로 대체 금지. odom 속도가 실제 움직임과 다르면 안정화 조건으로 문제 해결 불가. 저속이 안정되지 않으면 후보를 지나칠 수 있어 entry_stable_seconds·entry_speed_ready·remaining_m 확인 필요. 전체 코스 통과는 이번 검증 범위 아님.

## 리모 실행

브랜치명 괄호 포함으로 따옴표 필요. 기존 주행 노드 종료 후 실행.

```bash
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash
cd ~/parking_main_ws/src/ERICAR
git fetch origin 'refs/heads/parking_1116(2):refs/remotes/origin/parking_1116(2)'
git switch 'parking_1116(2)' || git switch -c 'parking_1116(2)' 'origin/parking_1116(2)'
git merge --ff-only 'origin/parking_1116(2)'
cd ~/parking_main_ws
colcon build --packages-select ericar_driver
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
ros2 launch ericar_driver parking_1116_2.launch.py model:=/home/wego/models/best.pt
```

전용 launch는 접근 상한 0.2 m/s. 일반 주차 구간 주행은 전용 launch 대신 driver.launch.py model:=/home/wego/models/best.pt yolo:=true start_stage:=parking 사용. 센서 한 세트·주행 노드 한 개 유지.
