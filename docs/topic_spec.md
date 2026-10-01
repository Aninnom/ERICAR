# 토픽 명세

노드끼리 주고받는 데이터의 약속. 코드와 이 문서가 다르면 이 문서를 먼저 고치고 팀에 알린다.

**상태: 초안 (회의에서 확정 전)**

## 공통 원칙

1. 단위는 미터·라디안·초. 좌표계는 REP-103 (앞 x+, 왼쪽 y+, 왼쪽 회전 +)
2. 인지는 측정값과 검출 여부만 보낸다. 판정은 판단이 한다
3. `/limo/ack_cmd` 발행자는 제어 노드 하나뿐
4. 메시지에 타임스탬프를 넣고, 받는 쪽은 오래된 값(0.5초 이상)을 버린다
5. 임계값·관심영역·게인은 코드가 아니라 `autorace_bringup/config/race.yaml`에

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
| `/odom` | 드라이버 → decision_node, control_node | `nav_msgs/Odometry` | 제공 |
| (회의에서 결정) | camera_node → decision_node | | |
| (회의에서 결정) | lidar_node → decision_node | | |
| (회의에서 결정) | estop_node → control_node | | |
| (회의에서 결정) | decision_node → control_node | | |
| `/limo/ack_cmd` | control_node → 드라이버 | `ackermann_msgs/AckermannDrive` | speed m/s, steering_angle rad (약 ±0.42) |

## 미정 사항

- 제어는 횡오차를 인지에서 직접 받나, 판단이 중계하나
- 판단이 E-stop 감지 범위를 조절하나 (라바콘·주차 구간)
- YOLO 노드 사용 여부와 실행 방법 (dl_env)
