# 실차 수령 첫날 체크리스트

위에서부터 순서대로 진행. 결과는 항목 옆에 기록.

## 1. 소프트웨어 확인

10/12 수령 당일 점검 완료 항목은 [x] 표시. 결과는 아래 "수령 당일 확인 결과" 참고

- [x] OS·ROS 버전: Ubuntu 22.04.5, ROS 2 Humble
- [x] 차체 모드 Ackermann (전면 걸쇠 긴 선 앞쪽, LED 초록)
- [x] Ackermann 드라이버로 `/limo/ack_cmd` 확인 (`teleop_launch.py`는 Differential 드라이버라 사용 금지)
- [x] `/scan` 10 Hz, `/camera/color/image_raw` 약 28 Hz, `/odom` 50 Hz
- [ ] `/odometry/filtered` (EKF) 발행 여부와 입력 odom 토픽 확인
- [ ] 시스템 시계 동기화 (수령 당시 Jan 1, 와이파이 연결 후 확인)

## 2. 하드웨어 보정

- [ ] 조향 영점 보정 (수령 당일 출발 시 앞바퀴 왼쪽 쏠림 확인) → 앱 Calibration 후 아래 명령으로 직진 확인
  ```bash
  ros2 topic pub -r 10 /limo/ack_cmd ackermann_msgs/msg/AckermannDrive "{steering_angle: 0.0, speed: 0.2}"
  ```
- [ ] 카메라 마운트 각도 확인 (-30 / 0 / +30°) → `teleop_launch.py degree:=`에 반영
- [ ] 라이다 유효 각도 확인 (차체에 가려지는 범위)
- [ ] 최대 조향각 기준 실제 회전 반경 측정

## 3. 카메라 보정 (scan line)

- [ ] scan 줄 위치(1~3개) 결정
- [ ] 줄마다 차 앞 거리, m/px, 차로폭 픽셀 수 실측
- [ ] 노출 고정값 결정 (자동 노출 해제)
- [ ] 트랙 조명 기준 황색·백색·적색 임계값 확인

## 4. 베이스라인

- [ ] 위고 예제 그대로 실행 (`limo_ros2_application track_application_launch.py`)
- [ ] 트랙 한 바퀴 rosbag 녹화 (카메라, 라이다, odom)

## 수령 당일 확인 결과 (10/12, 02번 LM-003702)

- 구성품 전부, 외관 이상 없음, 배터리 12.7 V, error_code 0
- 상판 "Renewed Used-Bad" 메모 → 현장 문의 결과 문제없음, 반납 때까지 유지
- 조종기 주행(4륜, 조향, 회전, 후진) 정상, 소음 거의 없음
- `/limo/ack_cmd` 명령으로 전진·후진·조향 정상, steering_angle + = 왼쪽, 명령 끊기면 정지
- 드라이버 실행: `ros2 launch limo_base limo_base_ackerman.launch.py`
- 원격 접속: 휴대폰 핫스팟에 차·노트북 연결 후 `ssh wego@<차 IP>`, 터미널 환경 선택은 1(Global)

## 5. 우리 코드

- [ ] `~/autorace_ws/src`에 레포 clone, 차량에서 직접 빌드
- [ ] `wego_ws` source → `autorace_ws` source 순서로 실행
