# 실차 수령 첫날 체크리스트

위에서부터 순서대로 진행. 결과는 항목 옆에 기록.

## 1. 소프트웨어 확인

- [ ] OS·ROS 버전: `lsb_release -a`, `echo $ROS_DISTRO` (예상: 22.04, humble)
- [ ] 차체 모드 Ackermann 여부 (전면 LED 초록)
- [ ] `ros2 launch wego teleop_launch.py` 후 `ros2 topic list`에 `/limo/ack_cmd` 존재 여부
  - 없으면 통합 런치가 Differential 드라이버 실행 중 → `limo_base_ackerman.launch.py` 확인
- [ ] `/scan`, `/camera/color/image_raw`, `/odom`, `/odometry/filtered` 발행 여부

## 2. 하드웨어 보정

- [ ] 조종기 SWB 상단(자동 모드)에서 직진 명령 → 휘면 앱에서 조향 영점 보정
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

## 5. 우리 코드

- [ ] `~/autorace_ws/src`에 레포 clone, 차량에서 직접 빌드
- [ ] `wego_ws` source → `autorace_ws` source 순서로 실행
