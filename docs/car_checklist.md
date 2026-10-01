# 실차 수령 첫날 체크리스트

위에서부터 순서대로. 결과는 옆에 적어둔다.

## 1. 소프트웨어 확인

- [ ] OS·ROS 버전: `lsb_release -a`, `echo $ROS_DISTRO` (예상: 22.04, humble)
- [ ] 차체 모드가 Ackermann인지 (전면 LED 초록)
- [ ] `ros2 launch wego teleop_launch.py` 후 `ros2 topic list`에 `/limo/ack_cmd`가 있는지
  - 없으면 통합 런치가 Differential 드라이버를 띄우는 것 → `limo_base_ackerman.launch.py` 확인
- [ ] `/scan`, `/camera/color/image_raw`, `/odom`, `/odometry/filtered`가 나오는지

## 2. 하드웨어 보정

- [ ] 조종기 SWB 상단(자동 모드)에서 직진 명령 → 휘면 앱에서 조향 영점 보정
  ```bash
  ros2 topic pub -r 10 /limo/ack_cmd ackermann_msgs/msg/AckermannDrive "{steering_angle: 0.0, speed: 0.2}"
  ```
- [ ] 카메라 마운트 각도 확인 (-30 / 0 / +30°) → `teleop_launch.py degree:=`에 반영
- [ ] 라이다 유효 각도 확인 (차체에 가려지는 범위)
- [ ] 최대 조향각으로 실제 회전 반경 측정

## 3. 카메라 보정

- [ ] 바닥에 실측 사각형을 붙이고 BEV 네 점(`SRC_PTS`, `DST_PTS`)과 픽셀당 거리 측정
- [ ] 노출 고정값 결정 (자동 노출 끄기)
- [ ] 트랙 조명에서 황색·백색·적색 임계값 확인

## 4. 베이스라인

- [ ] 위고 예제 그대로 실행 (`limo_ros2_application track_application_launch.py`)
- [ ] 트랙 한 바퀴 rosbag 녹화 (카메라, 라이다, odom)

## 5. 우리 코드

- [ ] `~/autorace_ws/src`에 레포 clone, 차 위에서 빌드
- [ ] `wego_ws` source → `autorace_ws` source 순서로 실행
