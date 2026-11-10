# ERICAR 팀 작업 안내 (Claude Code 공용)

AutoRace 2026 본선용 LIMO 주행 코드. 팀원 모두 이 레포에서 Claude Code를 켜면 이 문서를 자동으로 읽음.
최종 갱신: 2026-11-08 (혁준)

## 일하는 방식

- 한국어로 답변. 개념은 "왜 그런지"까지 짧게 설명
- 오류는 원인 → 확인 명령 → 해결 순서. 추측이면 추측이라고 밝힘
- 레포 md·코드 주석은 짧은 명사형 종결(~함, ~로 결정, ~필요). "~한다" 금지
- 다른 사람 담당 파일·구역은 먼저 알린 뒤 수정 (담당표 `ericar_driver/README.md`, `params.py` 구역 주석)
- 커밋·푸시는 요청받았을 때만. `main` 직접 커밋, `push --force`, `reset --hard` 금지
- 추측으로 고치지 말고 녹화 재생·시뮬로 먼저 확인 후 실차 (`docs/lane.md` "검증 순서")

## 작업 흐름

1. 내 컴퓨터: `git switch main && git pull` → `git switch -c <브랜치>` → 수정·커밋·푸시 → PR
2. 차: 받기만 함 (수정·커밋 금지). `cd ~/ericar_dev_ws/src/ericar && git fetch && git switch <브랜치> && git pull`
3. 차 빌드·실행: `cd ~/ericar_dev_ws && colcon build --packages-select ericar_driver && source install/setup.bash`
   → `ros2 launch ericar_driver bringup.launch.py yolo:=false record:=true`
4. 회귀 시험·녹화 재생은 **차(Jetson)에서** 실행. 기준값을 차에서 생성 → 다른 컴퓨터는 계산 끝자리 차이로 첫 줄부터 실패 가능

## 차량 (LIMO Pro, Jetson Orin Nano, Ubuntu 22.04, ROS 2 Humble)

- 접속: 휴대폰 핫스팟에 차·노트북 연결 → `ssh wego@<차 IP>` (비밀번호는 사람이 입력)
  - IPv4를 못 받으면 링크 로컬 IPv6: `ping6 ff02::1%<인터페이스>`로 탐색 후 `ssh wego@<fe80::...>%<인터페이스>`
- 실행은 `bringup.launch.py` 한 줄 (바퀴 `limo_base_ackerman`·라이다·카메라 노출 300·주행 노드·디버그 화면·녹화). 인자 표 `ericar_driver/README.md`
- 디버그 화면: 노트북에서 `ssh -L 8080:localhost:8080 wego@<차>` 후 `http://localhost:8080`. 주행 노드 재시작 후 새로고침
- 명령 `/limo/ack_cmd` (`AckermannDrive`, 조향 + = 왼쪽, 약 ±0.42 rad). 조종기 SWB 위(자동)일 때만 움직임, 아래면 정지
- wego `teleop_launch.py` 사용 금지 (차동 모드라 아커만 odom 미계산, 바퀴 드라이버 중복)
- 카메라 노출 수동 300 (자동 노출은 노란 선 색이 날아감). bringup이 자동 설정
- 배터리 11 V 미만이면 충전 후 시험 (`/limo_status`의 `battery_voltage`)
- 녹화는 차 `~/bags/`. 와이파이로 큰 bag 복사 금지 (망 포화) → 분석은 차에서
- ssh로 백그라운드 실행한 노드는 SIGINT 무시 → 종료는 SIGTERM. `pkill -f` 패턴이 ssh 명령 자체와 겹치면 셸이 끊김 → PID로 종료

## 대회 규정 핵심 (`Autorace2026_규정집` 초안 10/8)

- 시계 방향 1바퀴, 좌측 상단 출발. 2회 주행 중 좋은 점수, 제한 10분, 동점이면 랩타임
- 기본 속도 0.3~0.4 m/s 권고 (`params.py` `base_speed` 0.4), 서행 구간 최대 0.2 m/s
- 차로 이탈·시설물 충돌 1회 −1점, 재시작 없음. 미션 진입·재출발 15초 초과 시 0점
- 판정 기준은 **앞바퀴**. 정지선 ±5 cm, 보행자 3초 이상 대기
- 트랙 순서: 서행(방지턱) → 1번 코너 → 서행 유도선(지그재그) → 보행자 → 3차선 장애물 → 갈림길 → 라바콘 → 신호등 → T 주차
- 도면 실측: 선 중심 간격 0.52 m (규정 표기 550은 바깥 폭), 코너 안쪽 R590·바깥 R660 비동심 → 코너 차로 폭 약 0.77 m. 좌표표 `docs/code_check.md`

## 현재 상태 (11/8)

- 차선(노란색): 1번 코너 0.3·0.4 m/s 실차 통과. 설계·이유·주의점 `docs/lane.md`
- **흰색(정지선·횡단보도·마름모·"천천히") 검출 없음** → 정지선 의존 미션(보행자·신호등·주차 정렬) 실차 미동작. 소민 담당, 작업 안내 `docs/lane.md` "흰색 작업"
- 회귀 시나리오(`test/scenario.py`)는 정지선을 차선색(노랑)으로 그림 → 흰색 검출로 바꾸면 시나리오 수정·기준값 재생성 필요 (혁준)
- 팀 파일 담당: `ericar_driver/README.md` 표. 일정·공지는 노션

## 자주 쓰는 확인

```bash
python3 ericar_driver/test/test_regression.py        # 회귀 시험 (차에서 약 2분)
python3 ericar_driver/tools/bag_timeline.py ~/bags/<이름>   # 주행 구간 조향·오차 시계열
```
