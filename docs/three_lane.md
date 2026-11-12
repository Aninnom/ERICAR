# 3차선 회피·갈림길 상태 흐름

A=차량 왼쪽(도면 아래), B=차량 오른쪽(도면 위). 차로 번호 0=오른쪽, 1=중앙, 2=왼쪽.

## 상태와 제어

| 상태 (`fork_phase`) | 목표·전환 조건 | 조향 담당 |
|---|---|---|
| `lane` 일반 주행 | 점선 구간 진입 시 중앙 차로 정렬, 이후 현재 차로 추종. 장애물 발견 시 안전한 차로 회피 | 평소 차선 추종, 이동 중 LaneChange |
| `locked` 방향 확정·기억 | 같은 화살표 3프레임 확인. A→2, B→0 목적지 고정. 즉시 목적지 쪽 이동 가능 여부 검사 | 차선 추종 또는 LaneChange |
| `align` 출구 정렬 | 한 차로씩 이동. 목표 쪽 막힘 시 일반 회피, 최종 목적지는 유지 | 이동 중 LaneChange |
| `follow` 선택 출구 추종 | 목표 차로 도착·이동 종료 후 진입. 회피로 목표 차로 이탈 시 align 복귀 | 평소 차선 추종 |
| `done` 종료 | follow에서 fork_distance(3 m) 전진 또는 구간 관리자 S3 진입 | 이후 구간의 기존 제어 |

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
