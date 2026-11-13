# 3차선 회피·갈림길 상태 흐름

A=차량 왼쪽(도면 아래), B=차량 오른쪽(도면 위). 차로 번호 0=오른쪽, 1=중앙, 2=왼쪽.

## 상태와 제어

| 상태 (`fork_phase`) | 목표·전환 조건 | 조향 담당 |
|---|---|---|
| `lane` 일반 주행 | 점선 구간 진입 시 중앙 차로 정렬, 이후 현재 차로 추종. 장애물 발견 시 안전한 차로 회피 | 평소 차선 추종, 이동 중 LaneChange |
| `locked` 방향 확정·기억 | 같은 화살표 3프레임 확인. A→2, B→0 목적지 고정. 즉시 목적지 쪽 이동 가능 여부 검사 | 차선 추종 또는 LaneChange |
| `align` 출구 정렬 | 한 차로씩 이동. 목표 쪽 막힘 시 일반 회피, 최종 목적지는 유지 | 이동 중 LaneChange |
| `follow` 목표 차로 유지 | 목표 차로 도착·이동 종료 후 진입. 회피로 목표 차로 이탈 시 align 복귀. 점선 구간에서는 거리로 끝내지 않음 | 평소 차선 추종 |
| `exit` 선택 출구 진입 | 분기 입구(흰 점선 끝)에 목표 차로로 도착. 차로 번호 무시, 선택한 길 경계 추적. **차선 변경 금지**. fork_distance(3 m) 뒤 done | 평소 차선 추종 + 출구 경계 이름 보정 |
| `manual` 수동 개입 | 분기 입구에 가운데·반대편 차로로 도착 (표지판 없이 가운데 포함). **정지**, 멈춤 방지보다 우선. 재출발로 진행 | 정지 |
| `done` 종료 | exit에서 fork_distance 전진, 또는 구간 관리자 S3 진입. 입구를 지난 뒤면 차선 변경 계속 금지 | 이후 구간의 기존 제어 |

## 분기 입구·늦은 지점 (소민 11/12, 실차 전)

- 분기 입구 = 흰 점선이 끝나는 곳. 카메라 버드아이뷰(앞 0.6~1.7 m)에서 가장 먼 점선까지 거리 `dash_end`를 보고,
  안 보이면 주행 거리만큼 줄여 남은 점선 거리 `fork_dash_left` 추정
- 늦은 지점: 점선 끝이 화면 안에 보임 (`fork_dash_left` < far 1.7 − `fork_dash_gap` 0.3 = 1.4 m). 한 차로 이동(약 1.6~1.8 m)할 거리 부족 →
  새 차선 변경(정렬·회피·중앙 정렬) 시작 금지. 진행 중인 변경은 끝까지
- 입구 도착: 점선이 끝나고 `fork_entrance_confirm` 0.3 m 더 감. 진행 중인 변경이 있으면 끝난 뒤 판단
- 표지판 없이 끝 차로로 입구 도착 → 그 차로의 출구로 exit
- 점선을 한 번도 못 봤으면 예전 동작 (follow에서 fork_distance 뒤 done)
- 반대편 끝에서 두 차로 이동은 표지판을 일찍 읽어야 가능. 점선 구간 길이 안에 두 번 이동(약 3.5 m)이 안 되면 입구에서 manual
- 확인 값: `fork_phase`, `fork_dash_left`, `fork_late`, `dash_end`

- 분기 판단의 원본은 `fork.py`의 ForkMission 한 곳. driver.py는 센서·제어기 연결 담당
- 최초 확정 방향·시각·목표 차로는 종료 후에도 기록 유지. 반대 검출로 덮어쓰기 금지
- 15 cm 대기와 예전 `fork_prepare_distance`, `fork_wait_for_obstacles`, `fork_clear_distance` 설정 제거
- 중앙 차로 정렬은 최초 진입용. 장애물을 피한 뒤 무조건 중앙으로 복귀하지 않음
- 방향 확정 순간 이미 차선 변경 중이면 현재 이동 완료 후 목적지 재검사. 이동 도중 갑작스러운 목표 전환 방지
- 안전한 옆 경로 없음·센서 오래됨·전방 비상 거리 조건에서는 정지 유지
- 노란 경계 횡단 금지와 기존 라이다 최소 여유·안전 원호 검사 유지
- 화살표 면적 0.15%, 서로 다른 3프레임 확인. 원거리 검출 보정 필요
- S2 준비·분기 중 속도 상한 0.18 m/s. 모델의 지속적인 반대 분류는 상태 관리만으로 해결 불가
- 기존 영상 경계 보정 유지. 같은 선의 이름 연속성 우선, 실제 교통섬 지도 인식 기능 아님

## 확인 값

`fork_phase`, `fork_sign`, `fork_sign_stamp`, `fork_destination`, `fork_request`,
`fork_follow_travel`, `lane_index`, `lane_change_direction`, `avoiding` 확인 필요.
기존 fork_goal·fork_remaining 등은 분석 도구 호환용 파생값, 상태 판단에 별도 사용 없음.

## 검증

```bash
python3 ericar_driver/test/test_fork_state.py
python3 ericar_driver/test/test_fork_entrance.py
python3 ericar_driver/test/test_fork.py
python3 ericar_driver/test/test_fork_prepare.py
python3 ericar_driver/test/test_lane_change_boundary.py
python3 ericar_driver/test/test_sections.py
python3 ericar_driver/test/three_lane.py
```

합성 시험은 상태·정해진 배치 검증용. 녹화 재생·Jetson 회귀 확인 후 실차 필요 (`docs/lane.md` 검증 순서).
기준 회귀 시나리오는 구간 전환·흰 정지선 조건 불일치 이력 존재, 기준값 임의 갱신 금지.
중복 주행 노드·카메라 갱신 중단 해결 후 주행 필요.

## 이번 PC 검증 결과

- 상태 전환·제어 연결·방향 고정·경계 시험 통과
- 전체 합성 주행 sign_first·fork_right·normal·no_signs 4/4 통과
- 거리 조건 비교: 100 cm/30 cm와 80 cm/15 cm 모두 33개 중 30개 통과, 실패 배치 동일. 기본값 유지
- test_sections: 구간 단독·멈춤 방지 통과. 전체 미션에서 lane_change·parked 누락으로 실패
- 수정 전 HEAD 제어 코드에서도 같은 누락 재현. 해당 실패는 이번 상태 분리 전부터 존재, 기준값 갱신 없음
- 실제 카메라 녹화 재생·실차 확인은 별도 필요

## 11/12 실차 증상 수정 (소민)

| 증상 | 원인 | 수정 |
|---|---|---|
| 장애물 앞에서 회피 안 함 | 먼 점선 조각을 한 프레임 놓치면 `dash_end`가 순간 짧아짐 → `fork_late` 착각 → 새 차선 변경 전부 금지 | `fork.observe`: 보여도 "이전 추정 − 주행 거리"보다 짧으면 이전 추정 유지. `fork_dash_gap` 0.3 → 0.45 |
| 두 번 연속 차선 변경(0→1→2, 2→1→0) 뒤 노란 바깥선 이탈 | 이동 중 위치를 odom 속도·IMU만으로 추정 → 오차(odom 0.8배, IMU 0.85배)만큼 한 번에 13~19 cm 더 감, 두 번이면 누적 | `lane_change.camera_correct`: 목적 차로 색이 2프레임 연속 + 그 중앙이 반 차로 안이면 위치·방향을 카메라 값으로 교체 (`camera_fix_frames`, `camera_fix_range`) |
| 여러 장애물 뒤 가운데 차로에서 직진 충돌 | 차선 변경이 8초 막히면 멈춤 방지가 1.5 m 동안 차선 변경을 완전히 끔 → 그 안의 다음 장애물을 회피 판단 없이 직진 | 꺼진 동안에도 앞 장애물 회피는 함 (`avoid_only`). 갈림길·중앙 정렬은 계속 끔, 갈 차로 없으면 대기 대신 차선 주행(비상 정지 → 서행 비켜 가기) |

합성 시험(three_lane.py)에 실차 오차 흉내 추가: `odom_speed_scale`, `imu_scale`, `steer_scale`, `steer_lag`. 이탈 판정을 차체 중심 → 차체 가장자리(±0.11 m)로 강화.
새 사례 `right_then_left`, `left_then_right` (두 번 연속 변경). 정상·odom 0.8배+조향 지연·IMU 0.85배 세 조건에서 확인.
실차 확인: 녹화의 `lane_change_debug.camera_fix`(카메라 보정 값), `lane_change_debug.reason`이 `off_no_lane`인지.
