# 3차선 회피·YOLO 분기

- 작업 브랜치: `feat/3lane-yolo-fork`, main 기반
- 기존 라이다 회피·차선 색 기반 차로 번호 사용. 0=오른쪽, 1=중앙, 2=왼쪽
- S2에서 장애물 회피 주행, YOLO `left` 또는 `right` 연속 확인 후 방향 저장
- 두 화살표 동시 확인 시 방향 확정 보류
- 회피 진행 중이거나 전방 넓은 물체 검출 중이면 분기 이동 대기
- 장애물 미검출 상태로 `fork_clear_distance` 이동 후 표지판 쪽 끝 차로까지 한 차로씩 이동 요청
- 차로별 라이다 여유·안전 조향 검사 유지. 목표 차로 도착 또는 `fork_distance` 소진 시 요청 해제
- `fork_distance`는 표지판 확정부터 계산. 조기 인식 시 장애물 구간 안에서 거리 소진 가능, 녹화로 조정 필요
- `fork_wait_for_obstacles=False`는 기존 조향 편향 방식 유지. 기본값 True 사용

## 검증

PC 상태 시험: `python ericar_driver/test/test_fork.py`

차에서 녹화 재생 전후 비교·회귀 시험 후 실차 시험 필요 (`docs/lane.md` 검증 순서).
상태 시험은 차로 이동 완료 입력을 합성한 논리 시험이며 카메라 인식·실제 분기 궤적 검증 미포함.

차에서 단독 시작: 센서·바퀴 실행 후 `ros2 launch ericar_driver driver.launch.py start_stage:=fork` 사용.
센서 실행은 `ros2 launch ericar_driver bringup.launch.py driver:=false record:=true` 사용.
모델 클래스는 `left`, `right` 포함 `params.py`의 `YOLO_CLASSES`와 일치 필요.
