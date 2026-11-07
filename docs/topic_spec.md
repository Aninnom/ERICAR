# 토픽 명세

주행 노드 `ericar_driver` 하나의 입력·출력. 노드 내부 정보는 토픽 없이 함수 호출로 전달. 토픽 이름은 `ericar_driver/config/robot.yaml`의 `topics`.

## 공통 원칙

1. 단위는 미터·라디안·초. 좌표계는 REP-103 (앞 x+, 왼쪽 y+, 왼쪽 회전 +)
2. `/limo/ack_cmd` 발행은 `ericar_driver`만
3. 센서 헤더 시간 기준으로 0.7초 넘은 값은 폐기 후 정지 (`params.py`의 `sensor_timeout`)

## 입력

| 토픽 | 보내는 쪽 | 타입 | 비고 |
|---|---|---|---|
| `/camera/color/image_raw` | 카메라 드라이버 | `sensor_msgs/Image` | 640×480, 약 30 Hz |
| `/scan` | 라이다 드라이버 | `sensor_msgs/LaserScan` | 10 Hz, QoS Best Effort, 유효 각도 약 ±110° |
| `/odom` | 차량 드라이버 (`limo_base_ackerman.launch.py`) | `nav_msgs/Odometry` | 50 Hz |

## 출력

| 토픽 | 받는 쪽 | 타입 | 비고 |
|---|---|---|---|
| `/limo/ack_cmd` | 차량 드라이버 | `ackermann_msgs/AckermannDrive` | speed m/s, steering_angle rad (±0.42 제한, + = 왼쪽) |
| `/answer/status` | 디버그 | `std_msgs/String` | 현재 상태 (`lane`, `parking_align` 등) |
| `/answer/detections` | 디버그 | `std_msgs/String` | YOLO 결과·제어값 JSON |
| `/answer/debug_image` | 디버그 | `sensor_msgs/Image` | 차선 마스크 |

드라이버는 `limo_base_ackerman.launch.py` 사용 (`wego teleop_launch.py`의 기본 드라이버는 Differential용이라 Ackermann에서 오도메트리 미계산).
