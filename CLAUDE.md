# ERICAR 팀 작업 안내 (Claude Code 공용)

AutoRace 2026 본선용 LIMO 주행 코드. 팀원 모두 이 레포에서 Claude Code를 켜면 이 문서를 자동으로 읽음.
최종 갱신: 2026-11-17 (혁준)

**메인 계획: `docs/plan.md`** (11/10 채택). 본선까지 모든 작업의 기준. 주행 결과와 남은 문제는 이슈 #68(11/17)·#61(11/15~16)

## 일하는 방식

- 세션 시작 시 `docs/plan.md` 0장(사용 규칙·작업 묶음)과 자기 담당 묶음의 장을 먼저 읽고 그 방식대로 작업
- 계획과 다르게 가야 하면 임의로 바꾸지 않고 팀장(혁준) 확인 후 `docs/plan.md` "변경 기록"에 한 줄, 해당 장 수정
- 한국어로 답변. 개념은 "왜 그런지"까지 짧게 설명
- 오류는 원인 → 확인 명령 → 해결 순서. 추측이면 추측이라고 밝힘
- 레포 md·코드 주석은 짧은 명사형 종결(~함, ~로 결정, ~필요). "~한다" 금지
- 다른 사람 담당 파일·구역은 먼저 알린 뒤 수정 (담당표 `ericar_driver/README.md`, `params.py` 구역 주석)
- 커밋·푸시는 요청받았을 때만. `main` 직접 커밋, `push --force`, `reset --hard` 금지
- 추측으로 고치지 말고 녹화 재생·시뮬로 먼저 확인 후 실차 (`docs/lane.md` "검증 순서")

## 작업 흐름

1. 내 컴퓨터: `git switch main && git pull` → `git switch -c <브랜치>` → 수정·커밋·푸시 → PR
2. 차: 받기만 함 (수정·커밋 금지). 작업 공간 두 개
   - `~/ericar_main_ws`: `main` 확정본 (본선용). `cd ~/ericar_main_ws/src/ericar && git pull`
   - `~/ericar_dev_ws`: 브랜치 시험용. `cd ~/ericar_dev_ws/src/ericar && git fetch && git switch <브랜치> && git pull`
3. 차 빌드·실행 (쓰는 작업 공간에서): `colcon build --base-paths src/ericar --packages-select ericar_driver && source install/setup.bash`
   → `ros2 launch ericar_driver bringup.launch.py record:=true`
   - 한 터미널에는 작업 공간 하나만 `source` (둘 다 하면 어느 쪽 코드가 도는지 헷갈림)
   - `params.py`를 바꾸면 다시 빌드해야 반영됨
4. 회귀 기준값(`test/golden_regression.json`)은 맥 생성본 (PR #67). 다른 컴퓨터는 계산 끝자리 차이로 다를 수 있음 → 다르면 주행 명령(`drive`)이 같은지부터 확인

## 차량 (LIMO Pro, Jetson Orin Nano, Ubuntu 22.04, ROS 2 Humble)

- 접속: 휴대폰 핫스팟에 차·노트북 연결 → `ssh wego@<차 IP>` (비밀번호는 사람이 입력)
  - IPv4를 못 받으면 링크 로컬 IPv6: `ping6 ff02::1%<인터페이스>`로 탐색 후 `ssh wego@<fe80::...>%<인터페이스>`
- 실행은 `bringup.launch.py` 한 줄 (바퀴 `limo_base_ackerman`·라이다·카메라 노출은 설정값·주행 노드·디버그 화면·녹화). 인자 표 `ericar_driver/README.md`
- 디버그 화면: 노트북에서 `ssh -L 8080:localhost:8080 wego@<차>` 후 `http://localhost:8080`. 주행 노드 재시작 후 새로고침
- 명령 `/limo/ack_cmd` (`AckermannDrive`, 조향 + = 왼쪽, 약 ±0.42 rad). 조종기 SWB 위(자동)일 때만 움직임, 아래면 정지
- wego `teleop_launch.py` 사용 금지 (차동 모드라 아커만 odom 미계산, 바퀴 드라이버 중복)
- 카메라 노출 수동 600 (자동 노출은 노란 선 색이 날아감). bringup이 자동 설정, 600에서 카메라 약 16.6 Hz(300은 약 30 Hz). 현장 밝기에 따라 `exposure:=`로 조정
- YOLO는 켜고 GPU 추론이 기본 (`detector.device` 0). 모델은 차 `~/models/best.pt`
- 차 터미널 환경 선택은 **3 (dl_env)**: GPU용 torch·ultralytics가 `dl_env`에만 있음. 환경이 안 맞으면 로그에 `YOLO failed`, 표지판 미션만 꺼짐. ssh 한 줄 실행은 `~/.bashrc` 미적용 → `ROS_LOCALHOST_ONLY=1`·`PYTHONPATH`(dl_env site-packages) 직접 설정
- 배터리 11 V 미만이면 충전 후 시험 (`/limo_status`의 `battery_voltage`)
- 녹화는 차 `~/bags/`. 와이파이로 큰 bag 복사 금지 (망 포화) → 분석은 차에서
- ssh로 백그라운드 실행한 노드는 SIGINT 무시 → 종료는 SIGTERM. `pkill -f` 패턴이 ssh 명령 자체와 겹치면 셸이 끊김 → PID로 종료

## 대회 규정 핵심 (`Autorace2026_규정집_v3` 11/12 최종본)

- 시계 방향 1바퀴, 좌측 상단 출발. 2회 주행 중 좋은 점수, 제한 10분, 동점이면 랩타임
- 기본 속도 0.3~0.4 m/s 권고 (`params.py` `base_speed` 0.4), 서행 구간 최대 0.2 m/s
- 차로 이탈·시설물 충돌 1회 −1점, 재시작 없음. 미션 진입·재출발 15초 초과 시 0점
- 판정 기준(진입·통과·정지선 도달·시간)은 **뒷바퀴** (v3, 이전 앞바퀴). 단 **정지 거리(정지선 ±5 cm) 판정은 앞바퀴** (주최 답변 11/15). 보행자 3초 이상 대기
- v3 추가: 아커만 필수 / 서행 유도선·3차선·라바콘은 체크포인트 균등 배점(15/10/5, 10/6/3, 15/10/5) / 서행 유도선도 0.2 m/s 이하, 지그재그 따라 조향하지 않고 직진 / 라바콘 전도·표식 이탈 개당 −1점, 구간 내 차선 밟음 감점 없음 / 3차선 LIMO 위치는 팀·시도마다 변경 가능 / 방지턱 통과 뒤 일반 속도 복귀 가능
- 현장: 가벽 높이 200 mm(라이다 z 0.07 m라 스캔에 잡힘, 도로와 이격 거리 미공지), 바닥 포맥스 + 빛반사 필름(현수막 아님)
- 트랙 순서: 서행(방지턱) → 1번 코너 → 서행 유도선(지그재그) → 보행자 → 3차선 장애물 → 갈림길 → 라바콘 → 신호등 → T 주차
- 도면 실측: 선 중심 간격 0.52 m (v3 차선 폭 500 mm, 이전 550은 오기), 코너 안쪽 R590·바깥 R660 비동심 → 코너 차로 폭 약 0.77 m. 좌표표 `docs/code_check.md`

## 현재 상태 (11/17, main `2365b70`)

상세·근거는 이슈 #68(3차선·갈림길 16회 주행)·#61(통합 주행 9회), 설계는 `docs/plan.md`

| 미션 | 상태 | 근거 |
|---|---|---|
| 서행 | 동작 | 0.15 m/s, 흰 선 기준 진입. 연습 트랙에 방지턱이 없어 방지턱은 미검증 |
| 서행 유도선 | 동작 | 0.18 m/s 통과 |
| 보행자 정지 | 동작 | 정지·약 4 s 대기. 줄자 실측은 미확인 (차 내부 추정값만) |
| 3차선 | 동작 (조건부) | 11/17 6회 연속 무개입. 차로를 바꿔야 하는 장애물 줄 간격 1.5 m 미만은 한계 (#64) |
| 갈림길 | 동작 | A·B 각 3회 진입, 방향 확정은 기둥 약 1.2~2.6 m 전 |
| 라바콘 | 동작 | 11/16 완주 주행에서 통과. **PR #67 조합으로는 실차 미확인** |
| 신호등 | 동작 | 11/16 정지·녹색 출발 확인. PR #67 조합으로는 실차 미확인, 줄자 실측 미확인 |
| 주차 | 동작 | 11/16 `parked` 확인 (정윤). PR #67 조합으로는 실차 미확인 |

- 완주(출발 ~ 주차, 수동 개입 없음): 11/16 `main_1116_0151`, 11/16 밤 `imu_1116_2348` 등. 같은 코드로도 실패한 주행이 있어 여유는 크지 않음
- 차선(노란색)은 구조 변경 없이 동결. 설계·이유·주의점 `docs/lane.md`
- 구간 관리자(IMU 누적 회전으로 S0~S4 판단)·멈춤 방지 적용 → 표지판 하나를 놓쳐도 다음 구간 미션은 동작
- 3차선 판단은 규칙을 더 쌓지 않기로 함 (11/17). 실차 근거가 없는 기능 7개는 구현만 하고 기본 끔, 목록은 `ericar_driver/README.md` "3차선·갈림길 설정". 구조 재설계는 본선 뒤
- 알려진 시험 실패 2건 (기능 문제 아님, 기준 정리 필요): `test_sections.py` 주차 표지판 누락 1건, `test_lidar_missions.py` 임의 3차선
- 팀 파일 담당: `ericar_driver/README.md` 표. 일정·공지는 노션

### 현장에서 맞출 값

연습 트랙에서 잰 값이라 대회장에서 다시 확인 필요

- 3차선 구간 길이에 묶인 값: `sections.lane_change_max_distance` 6.0 (넘으면 차선 변경 꺼짐), `missions.fork_enter_distance` 5.4, `lane_change.request_entrance_distance` 5.2. 3차선 진입 ~ 갈림길 기둥 거리로 맞춤
- 서행 `slow_zone.entry_distance` 1.5·`max_distance` 4.0, 라바콘 `cone_max_distance` 2.5
- 카메라 노출, 정지 위치(줄자 실측), 가벽이 라이다에 잡히는 정도
- 노드를 켤 때 차를 구간 방향과 나란히 놓기 (3차선 진입 정렬·방향 보정이 시작 자세 기준)

## 자주 쓰는 확인

```bash
python3 ericar_driver/test/test_regression.py        # 회귀 시험 (맥 약 15초, 차 약 2분)
python3 ericar_driver/test/three_lane.py             # 3차선 합성 시험 21건
python3 ericar_driver/tools/bag_timeline.py ~/bags/<이름>   # 주행 구간 조향·오차 시계열
ros2 topic echo --once /limo_status | grep battery   # 배터리 (차)
```
