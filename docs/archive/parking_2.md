# parking_2: 단순 FREE 트리거·흰 직선 소실 정지

main 단일 노드에 내장. 이전 parking_yolo_demo의 경로 계획·Pure Pursuit·odom 상대경로·가시성 구간 검사·가상 통로 모델을 실행하지 않음. main의 단일 AckermannDrive 발행 경로 유지.

## 요청 동작

```text
차선 주행
  parking 표지판 연속 인식
    parking_scan: 0.12m/s로 서행하며 LiDAR 두 물체 사이 FREE 검색
      FREE 1회
        parking_turn: 같은 제어 주기에 속도 0.12m/s, 조향 -0.35rad 명령
          진입 중 흰 직선을 관측한 후 새 영상에서 사라짐
            parked: 속도 0 명령, 재출발 안 함
```

FREE는 기존 검출과 같은 기준: 오른쪽 두 반사점 군집 사이 폭·행 정렬 검사, 내부 장애물 점 거부, 12개 검사점 중 최소 8개를 넘어가는 유효 반사 확인. 두 물체 사이에 낀 군집이 앞줄보다 깊으면(빈틈 너머로 보이는 가벽 조각) 건너뛰고 앞줄 쌍을 검사. 디버그와 제어가 같은 gap_candidates/classify_gap 사용. UNKNOWN을 FREE로 바꾸지 않음. FREE 이후 UNKNOWN 깜빡임은 진입을 취소하지 않음.

검색 중에는 main의 조향을 사용(차선 valid일 때), 차선을 못 보면 직진 조향으로 서행. 이전처럼 lane.valid=false만으로 검색 전진을 차단하지 않음. 실제 예상 차체 경로에 장애물 점이 있으면 정지. NaN 등 무효 scan 비율이 높으면 정지. 무반사 0 해석은 robot.yaml의 기존 zero_is_no_return 정책 유지.

**고정 조향 실험 버전**이며 칸 중심·차량 자세·전체 차체의 칸 내부 진입을 검증하지 않음. parked는 흰 선 소실 정지 조건 완료를 뜻하며 실제 주차 성공 증명이 아님. 첫 FREE 위치가 늦거나 대상 공간이 다르면 고정 원호가 칸에 맞지 않을 수 있음.

표지판 수용 시점: 신호등 미션을 끝냈거나(포기 포함) S4 진입 뒤 `after_traffic_distance`(4.0 m, 도면 추정) 이후. 주차 지점 재출발(`start_stage:=parking`)과 주차 단독 시험은 바로 수용. 표지판 최소 면적 0.002는 약 1.8 m 밖에서 확정되어 신호등보다 먼저 주차로 넘어갈 수 있기 때문.

## 흰 선 정지 기준

하단 50% ROI, 좌우 2% 가장자리 제외. HSV 흰색 후보에서 길고 수직 방향 변화가 있는 직선 검출. 가로선만으로는 측면 직선으로 인정하지 않음. 완전 격자·두 선 쌍·BEV·CameraInfo 보정 불필요.

- FREE 발생 영상부터 흰 직선 관측 시작
- 실제 관측한 직선이 새 영상에서 없어지면 즉시 정지. white_loss_frames=1
- 처음부터 흰 선이 없으면 최대 1.5초 탐색 후 WHITE_LINE_NEVER_SEEN으로 정지
- 오래된 영상·카메라 미수신은 선 소실 완료가 아니라 센서 대기 정지
- 밝기 변화·한 프레임 오검출·물체 가림도 선 소실로 보일 수 있으므로 실제 완료 위치 보장 없음

## 남겨둔 정지 조건

- 실제 LiDAR 점이 현재·바로 다음 차체 경로 안이면 정지, 해소되면 재진행 가능
- 카메라·scan·odom 미갱신 또는 계산 지연이면 정지
- 검색 최대 45초·5m, 우회전 최대 8초·0.85m
- 명령 중 odom 속도 무이동 4초면 정지
- /parking/cancel Bool true면 취소·정지

설정은 ericar_driver/ericar_driver/params.py의 parking. turn_steering=-0.35는 우회전, main 조향 영점 보정(-0.015)이 최종 출력에 더해짐. 모터 연결·조종기 제어 모드는 정상 main 주행과 동일하게 준비.

## 리모 설치

레포 `CLAUDE.md` "작업 흐름" 그대로 (차는 받기만 함). 기존 센서·주행을 먼저 종료:

```bash
cd ~/ericar_dev_ws/src/ericar && git fetch && git switch <브랜치> && git pull
cd ~/ericar_dev_ws && colcon build --packages-select ericar_driver && source install/setup.bash
```

## 실행

첫 번째 SSH 창 — 센서·읽기 전용 화면:

```bash
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash
source ~/ericar_dev_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
ros2 launch ericar_driver bringup.launch.py driver:=false debug:=true record:=false
```

컴퓨터 새 터미널에서 ssh -N -L 8080:127.0.0.1:8080 wego@172.20.10.3 실행 후 http://localhost:8080. 왼쪽 노란 선은 정지 판단에 사용하는 흰 직선, 가운데는 같은 ROI 마스크, 오른쪽은 같은 기준의 실시간 LiDAR FREE 판정. 하단 WHITE present/seen/loss frames 확인.

두 번째 SSH 창 — 실행 직후 저속 주행 가능:

```bash
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash
source ~/ericar_dev_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=1
ros2 topic info /limo/ack_cmd
# 발행자 0일 때만 아래 실행
ros2 launch ericar_driver parking_2.launch.py model:=/home/wego/models/best.pt
```

parking_2 launch는 parking 단독 미션·서행 시작, 주행 노드 1개. 센서창 driver=false 유지. 이전 parking_demo/다른 main 노드 동시 실행 금지. 실제 움직임에 대비한 감독·물리 정지 준비 필요. 실행 모터 명령은 /answer/detections의 runtime.published_speed, control.steering 확인.

## 검증

합성 raycast FREE 1회→음수 조향·양수 속도 같은 주기 발행, UNKNOWN 전환에도 턴 유지, 실제 흰 선 영상 소실 시 정지, 미관측·stale 영상 구분, 물체·시간·거리 제한·취소 검사. fake ROS를 통해 main 모터 토픽에 우회전·정지 명령 확인. 실차 성공 미검증. main의 golden 기준은 변경하지 않고 회귀 결과를 검증 보고서에 기록.

### 최신 main 병합 (2026-11-13)

main `3282045`의 갈림길·3차선·신호등·라바콘 개선 포함. 디버그 화면은 모든 구간에서 main의 6패널 구성 그대로(가운데는 항상 BEV). 주차 표시는 주행 노드 텔레메트리 값 사용: 라이다 칸에 물체 군집·빈칸 판정(FREE 초록, 확인 부족 주황, 점유 빨강, 선택 칸 노랑), 회전 중 카메라에 제어기가 쓴 흰 직선(노랑), 하단 WHITE present/seen/loss. 주행 노드 없이 센서만 켠 화면에서는 같은 함수로 빈칸 판정을 직접 계산(`PARK live`). 제어기는 FREE를 한 번 잡으면 회전 유지.
