# 토픽 명세

노드 간 데이터 약속. 코드와 문서가 다르면 문서 먼저 수정 후 팀에 공유.

**상태: 초안 (회의 확정 전)**

## 공통 원칙

1. 단위는 미터·라디안·초. 좌표계는 REP-103 (앞 x+, 왼쪽 y+, 왼쪽 회전 +)
2. 인지는 측정값과 검출 여부만 발행. 판정은 판단 담당
3. `/limo/ack_cmd` 발행은 제어 노드만
4. 메시지에 타임스탬프 포함. 0.5초 이상 지난 값은 수신 측에서 폐기
5. 임계값·관심영역·게인은 코드 대신 `autorace_bringup/config/race.yaml`에

## 노드 구성

```
카메라(제공) → camera_node ─┐
              (yolo_node)  ─┼→ decision_node → control_node → /limo/ack_cmd → 모터(제공)
라이다(제공) → lidar_node  ─┘                       ↑
            → estop_node ───────── 정지 신호 ───────┘
```

## 토픽 목록

| 토픽 | 보내는 쪽 → 받는 쪽 | 타입 | 내용·단위 |
|---|---|---|---|
| `/camera/color/image_raw` | 드라이버 → camera_node | `sensor_msgs/Image` | 제공 |
| `/scan` | 드라이버 → lidar_node, estop_node | `sensor_msgs/LaserScan` | 제공, QoS Best Effort |
| `/odom` | 드라이버 → decision_node, control_node | `nav_msgs/Odometry` | 제공, 50 Hz (Ackermann 드라이버 기준 실측) |
| `/limo_status` | 드라이버 → (필요한 노드) | `limo_msgs/LimoStatus` | 제공, motion_mode·battery_voltage·error_code |
| (회의 결정) | camera_node → decision_node | | |
| (회의 결정) | lidar_node → decision_node | | |
| (회의 결정) | estop_node → control_node | | |
| (회의 결정) | decision_node → control_node | | |
| `/limo/ack_cmd` | control_node → 드라이버 | `ackermann_msgs/AckermannDrive` | speed m/s, steering_angle rad (약 ±0.42, + = 왼쪽, 실차 확인) |

드라이버는 `limo_base_ackerman.launch.py` 기준 (`wego teleop_launch.py`의 기본 드라이버는 Differential용이라 Ackermann에서 오도메트리 미계산)

## 미정 사항

- 횡오차 전달 경로: 인지 → 제어 직접 vs 판단 중계
- 판단의 E-stop 감지 범위 조절 여부 (라바콘·주차 구간)
- YOLO 노드 사용 여부와 실행 방법 (dl_env)
