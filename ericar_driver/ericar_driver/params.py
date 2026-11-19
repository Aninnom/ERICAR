"""주행 파라미터. 담당별 구역만 수정. 거리 m, 속도 m/s, 조향 rad(+좌회전)"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)

ANSWER_CONFIG = {
    'rate_hz': 20.0, 'debug_rate_hz': 5.0, 'sensor_timeout': 0.7, 'odometry_timeout': 0.7,
    # 11/18 유경: base_speed 0.4 -> 0.6 (랩타임). 감속 계열(slow_speed·minimum_curve_speed·zigzag·slow_zone)은 그대로
    # 규정 권고는 0.3~0.4라 육안 판정 미션(서행)에서 기본 속도가 빨라 보일 수 있음 -> 팀장 확인 필요
    'base_speed': 0.6, 'slow_speed': 0.2, 'minimum_curve_speed': 0.15,
    # 코너 속도 상한. 도면 코너 시뮬: 0.4 진입은 바깥 접촉
    # 11/18 유경: 0.2 -> 0.25 (0.4 접촉과 사이, 차로 폭 0.77 m에 여유 남김). 0.3 이상은 실차 확인 전까지 금지
    # 11/19 유경: 0.25 -> 0.4 (랩타임, 사용자 지시). 위 "0.3 이상 금지"를 넘는 값이라 실차 확인 전까지는 시험용.
    #   같이 내린 corner_hold.steering 0.38 -> 0.35가 바깥으로 벌어지는 쪽이라 접촉이 더 쉬워질 수 있음 →
    #   첫 주행은 1·2번 코너에서 바깥 선 여유부터 보고, 접촉이 나면 0.3으로 되돌릴 것
    #   (1118 녹화 4개 실측 코너 속도는 0.19~0.22라 0.4는 두 배. 실차 근거 없음)
    # 현장 조정 순서 (11/19). 아래 두 값은 서로 반대 방향이라 한 번에 하나씩만 움직일 것:
    #   corner_speed         : 0.4(지금, 바깥선 위험) -> 0.3(바깥 접촉 나면 먼저 여기로) -> 0.35(둘 다 여유 있으면 튜닝)
    #   corner_hold.steering : 0.35(지금) -> 0.33(속도를 내려도 안쪽 선 밟으면) -> 0.38(바깥으로 벌어지면 되돌림)
    # 첫 주행은 1·2번 코너에서 바깥 선 여유부터 확인 (조향을 줄인 것도, 속도를 올린 것도 바깥으로 미는 쪽)
    'corner_speed': 0.4,
    # 선 곡률(1/m) 이 구간에서 base_speed → corner_speed 보간 (lane_driving)
    # 11/18 유경: [2.0, 3.5] -> [1.6, 3.0]. base_speed가 0.6이면 같은 곡률에서 보간 중 속도가 전보다 높고
    # 줄여야 하는 폭도 커져 코너 진입이 늦음 -> 감속 시작을 약간 앞으로 (곡률 1.6 ≈ 코너 약 0.7 m 전)
    'corner_slow_curvature': [1.6, 3.0],
    # 우회전 코너 유지: 코너 안에서는 카메라 사각지대(앞바퀴 앞 약 0.4 m 이내)로 선을 놓치고 남은 한 선 좌우 오판 (11/9 white_1109_lap)
    # → 코너 감지 후 오른쪽 조향 시작하면 고정 조향으로 odom 회전각 angle(°)까지 유지. 조향 0.31 = atan(축간거리 0.2 / 코너 중심 반지름 0.62)
    # 끝: 회전각 도달 또는 최대 거리 (두 선 조기 종료는 11/9 제거, lane_driving.corner_hold 참고)
    # 조향 출력 변화율 제한(rad/s)·한 프레임 급변 무시 (lane_driving). 11/9 lap4 직선 프레임당 변화 p95 0.015 rad
    'steering_rate': 1.5, 'steering_jump': 0.08, 'jump_frames': 3,
    # 서행 유도선(지그재그, 규정 미션 6 "서행 통과", 서행 구간 최대 0.2 m/s): 양쪽 선이 톱니 모양인 카메라 결과가
    # confirm_frames 연속이면 speed로 제한, 마지막으로 본 뒤 hold_distance(m) 더 갈 때까지 유지
    # (톱니가 화면에서 사라질 때 구간 끝은 뒷축 앞 약 1.0 m → 앞바퀴 기준 0.8 m 남음. 11/10 skip_1110_lap1 109.5~110.5 s)
    'zigzag': {'speed': 0.18, 'confirm_frames': 4, 'hold_distance': 1.0},
    # 차선을 lane.lost_grace 넘게 놓친 뒤: 정지 대신 speed로 가며 탐색, 조향은 straighten_distance(m) 동안 0으로
    # distance(m) 가거나 seconds(초) 지나도 못 찾으면 정지 (lane_driving.decide). 값은 실차 확인 전
    'lost_search': {'speed': 0.15, 'distance': 2.0, 'seconds': 20.0, 'straighten_distance': 0.5},
    'corner_hold': {'enabled': True, 'arm_seconds': 3.0, 'start_steering': 0.15,
                    # 시작 시 곡률 sharp_curvature 이상이면 steering, 아니면 gentle_steering (11/9 lap3, 1번 코너 0.31이면 바깥 이탈)
                    # 11/19 유경: steering 0.38 -> 0.35 (안쪽 선 밟음). 회전 반경 = 축거 0.2 / tan(조향)이라
                    #   0.38은 0.50 m, 0.35는 0.56 m. 코너 차로 중앙 반경은 (안쪽 590 + 바깥 660)/2 = 0.625 m.
                    #   gentle 0.31(= 0.625 m)은 기하학적으로 맞는 값이라 그대로 둠
                    #   조정 순서는 위 corner_speed 주석의 "현장 조정 순서" 참고 (0.35 -> 0.33 -> 0.38)
                    'steering': 0.35, 'gentle_steering': 0.31, 'sharp_curvature': 2.0,
                    # 유지 중 조향 = 시작 값 + error_gain × (시작 오차 − 현재 오차), steering_min~max_steering 제한
                    # 11/19 유경: steering_min 0.2 -> 0.14. 1118 녹화 19·20번에서 차가 코너 안쪽으로 0.09~0.15 m
                    #   치우친 순간 조향이 정확히 0.200에 멈춰 있었음 = 하한에 막혀 보정이 더 펴 주지 못함.
                    #   0.14는 반경 1.4 m라 코너 안에서도 계속 돌기는 함 (0.1 아래로 내리면 유지가 사실상 풀림)
                    'error_gain': 0.4, 'steering_min': 0.14,
                    # 시작 순간 가까운 선이 왼쪽으로 skip_near_curvature(1/m) 넘게 휘면 이번 코너는 유지 없이 pursuit (비스듬한 진입)
                    'skip_near_curvature': 0.1, 'skip_window': 0.6,
                    'angle': 65.0, 'max_distance': 1.3,
                    'start_far_angle': 20.0,    # 선 먼 끝이 오른쪽으로 이 각도(°) 넘을 때만 시작 (갈림길 S자 꺾임 제외)
                    # 유지 중·끝난 뒤 after_seconds 동안 기억 없는 한 선은 가까운 끝이 오른쪽 corner_left_limit(m) 안이면 왼쪽 선
                    'after_seconds': 1.5, 'corner_left_limit': 0.1},

    # 재출발 (노트북 없이, 노드 켜 둔 채): 조종기 SWB 수동(아래) → 차를 미션 시작 지점에 놓음 → SWB 자동(위)
    # 수동이 min_manual(s) 넘게 이어진 뒤 자동으로 돌아오면 드라이버를 새로 만들고(미션 상태 전부 초기화),
    # 놓인 방향으로 구간을 다시 정해 그 구간의 start_stage로 시작한 것과 같게 함 (driver.restart, section.relocate)
    # 노드를 켠 뒤 첫 자동 전환은 출발로 봄: start_stage 그대로
    # /limo_status control_mode: manual_mode(0) = 수동, 1 = 자동 (11/8 lane_1108_1618: 0인 동안 명령 0.2에도 정지, 1부터 주행)
    # 상태가 status_timeout(s) 넘게 안 오면 자동으로 봄 (예전 동작). 약 2 Hz 토픽. 끄기: enabled False
    'restart': {'enabled': True, 'min_manual': 1.0, 'manual_mode': 0, 'status_timeout': 3.0},
    # 구간 관리자 (section.py): 누적 회전이 다음 직선 방향(−90°씩)까지 margin(°) 안으로 들어오면 구간 +1
    # margin 25 → −65°·−155°·−245°·−335°에서 전환. 갈림길 꺾임 45°·차선 변경 흔들림은 못 미침
    # odom_yaw_scale: IMU가 끊겼을 때 odom 회전에 곱하는 값 (실측 = 0.93 × odom, 11/9). imu_timeout(s) 넘게 안 오면 odom 사용
    # enabled False면 구간과 무관하게 모든 미션 허용 (단독 시험용)
    # hold_distance: 라바콘 통과 중에는 구간 전환 보류 (콘 배치가 우회전을 강제하면 S3 도중 −335°를 넘음, 11/13).
    # 기준을 넘은 채 이 거리(m)를 가도 라바콘이 안 끝나면 전환 (안전장치)
    # off: 구간과 무관하게 꺼 둘 미션. 라바콘(cones)은 11/11 segA_1111_lap1(트랙 옆 라바콘·의자 다리를 통로로 보고
    # 진입 → 4번 코너 이탈)로 꺼 뒀다가 11/12 다시 켬 (유경): 단독 시험 corn_test_1112_193257 통과 (모드 3.1 m, 충돌 없음).
    # 꺼져 있으면 S3에서 콘 사이를 차선 주행·코너 유지(고정 조향)로 지나감 (11/12 cone_lane_1). 전환 실차 확인 필요
    'sections': {'enabled': True, 'margin': 25.0, 'odom_yaw_scale': 0.93, 'imu_timeout': 0.3,
                 'hold_distance': 3.0, 'off': [], 'restart_tolerance': 40.0,
                 # 11/18 혁준 (track_test_1118_01, 실차 전): 재출발 때 이웃 구간으로 바꾸는 건 방향 오차가 이 값(°) 안일 때만. 40이면 예전과 같음
                 'restart_switch_tolerance': 20.0,
                 # 자동 상태로 차를 들어 돌린 회전은 구간 전환에서 뺌 (section.update). 회전 속도가 속도 × carry_curvature(1/m, 최대 조향
                 # 곡률 약 2.23) × carry_rate_gain + carry_rate_offset(°/s)을 넘을 때. 0.4 m/s 주행 최대 약 51°/s, 기준 약 94°/s. 끄기: gain None
                 'carry_rate_gain': 1.5, 'carry_curvature': 2.3, 'carry_rate_offset': 15.0,
                 # S2에서 오른쪽으로 이만큼(°) 돈 뒤에는 붉은색이 보이면 라바콘 진입 허용 (S3 전환은 65°). None이면 끔 (11/14)
                 'cone_early_turn': 20.0,
                 # S2에서 라바콘 모드가 실제로 시작되면 회전 기준(65°)을 기다리지 않고 S3로 전환 (11/16). 끄기 False
                 'cone_advance': True,
                 # S2에서 오른쪽으로 이만큼(°) 돈 뒤(= 3번 코너)와 코너 유지 중에는 새 차선 변경 시작 안 함 (11/14 lap4)
                 'lane_change_max_turn': 15.0,
                 # S2에서 차가 직선 방향보다 왼쪽으로 이만큼(°) 넘게 틀어져 있으면 코너 유지 시작 안 함 (갈림길 뒤 S자, 11/14 lap5)
                 'hold_min_turned': 15.0,
                 # 코너 유지는 시작한 구간 직선 방향에서 오른쪽으로 이 각도(°, IMU)까지만. 넘으면 끝내고 새로 시작 안 함
                 'hold_max_turned': 70.0,
                 # S2에서 이 거리(m) 넘게 가면 3차선 구간 끝으로 보고 차선 변경 판단을 끔 (분기 입구를 지나면 그 전에 꺼짐).
                 # 11/15 녹화: 교통섬 앞머리 약 5.0 m(3차선 끝), 3번 코너 약 7.2 m, 코너 앞 저속 접근이 켜진 곳 8.0~8.3 m → 6.0
                 # 합성 시험 장면은 3차선이 더 길어 None으로 끔 (three_lane.py, test_sections.py)
                 'lane_change_max_distance': 6.0},
    # 서행·방지턱 (slow_zone.py, 규정 미션 5: 기본의 1/2·최대 0.2 m/s, 육안 판정). 담당 혁준. 값은 실차 확인 전
    # speed: 서행 속도. 상한 0.2와 차이를 분명히
    # entry_distance: 표지판을 못 봐도 S0 주행 거리(m)가 이 값이면 감속 (None이면 표지판만). 현장에서 표지판 위치로 재측정
    # exit_distance: 방지턱 감지 실패 시 해제할 S0 거리(m). 현장 실측 = 방지턱 끝 + 뒷바퀴 여유. None이면 미사용
    # max_distance: 감속 시작 뒤 이 거리(m)면 무조건 해제 (예전 14초 × 0.2 m/s = 2.8 m)
    # tilt_threshold(°)·tilt_decay(s): 자이로 x·y 누설 적분 기울기가 문턱 넘으면 방지턱. arm_distance(m): 감속 직후 숙임 무시
    # release_distance: 첫 감지 뒤 해제까지 거리(m). 뒷바퀴 기준 = 축거 0.2 + 턱 길이 + 여유
    # hold_distance: 첫 감지 뒤 조향 고정 거리(m, 앞·뒷바퀴 통과). stuck_*: 턱에서 멈추면 boost_speeds를 단계로 올림, boost_distance 구르면 복귀
    # entry_distance 11/18 경기장: 1.5 → 1.2. 방지턱 삼각 표지판이 pedestrian으로 읽혀 표지판 확정이 없고 거리 조건으로만 시작
    #   → 0.39 m/s에서 흰 선(출발 뒤 1.64~1.71 m) 6~16 cm 전에야 감속 시작 (17·20번). 1.2면 선 약 0.45 m 전
    # entry_at_line: 표지판 확정만으로 시작하지 않고 흰 가로선이 앞축 entry_line_distance(m) 안에 올 때 시작 (11/16, 선은 출발 뒤 약 1.47 m)
    # 11/18 혁준 (사용자 지시): speed 0.15 → 0.18. 04번 서행 구간 2.33 m에 15.4 s, 규정 상한 0.2까지 여유 0.02
    'slow_zone': {'speed': 0.18, 'entry_at_line': True, 'entry_line_distance': 0.1, 'entry_distance': 1.2, 'exit_distance': None, 'max_distance': 4.0,
                  'tilt_threshold': 2.0, 'tilt_decay': 0.5, 'arm_distance': 0.15, 'release_distance': 0.5,
                  'hold_distance': 0.35, 'stuck_speed': 0.03, 'stuck_seconds': 0.7,
                  'boost_speeds': [0.2, 0.3], 'boost_distance': 0.1},
    # 멈춤 방지 (driver.stall_guard): 의도하지 않은 정지가 seconds 넘으면 강제 전환 (규정: 15초 넘게 서면 그 미션 0점)
    # 차선 변경 대기 → lane_change_off_distance(m) 동안 차선 변경 끄고 차선 주행
    # 전방 비상 정지 → 라이다 점에서 creep_margin(m) 떨어진 조향이 creep_horizon(m) 앞까지 있으면 creep_speed로 비켜 감
    # 값은 실차 확인 전
    'stall': {'seconds': 8.0, 'reset_distance': 0.3, 'lane_change_off_distance': 1.5,
              'creep_speed': 0.08, 'creep_horizon': 0.3, 'creep_margin': 0.02},

    'detector': {
        # device 0 = Jetson GPU. 11/14 실측: CPU 추론은 주행 노드 CPU 390 %·영상 지연 0.16 s, GPU는 65 %·0.02 s
        # GPU 없는 PC에서 모델을 직접 돌릴 때만 'cpu'로 바꿈
        # image_size 320 → 640, gamma 0.6 (11/18 경기장 어두운 조명, 17·19·20번 사진 재추론): 빨강 신호등 1 → 13프레임(17), 0 → 9(20),
        #   보행자 첫 검출 61.6 → 57.3 s, 갈림길 화살표 오독 10 → 5. 차 GPU 추론 시간은 320·640 모두 약 33 ms (실측)
        #   gamma: YOLO 입력에만 적용하는 밝기 보정(1.0 = 없음, 작을수록 밝게). 320에서 감마만 쓰면 parking·pedestrian이 줄어듦 → 640과 함께
        #   차선·램프 색 판정은 원본 영상 그대로. 되돌리기: image_size 320, gamma 1.0
        'enabled': True, 'confidence': 0.45, 'image_size': 640, 'gamma': 0.6, 'device': 0, 'rate_hz': 5.0,
        # traffic_hold_image_size (11/19 혁준, 녹화 재추론만·실차 전): 신호등 앞 정지 중(traffic_hold)에만 이 크기로 추론
        #   640은 가까운 초록 신뢰도가 낮음 (track_testv2_1118_10 정지 뒤 초록 인정 18/39 → 320은 38/39, 02·04·01번은 같음)
        #   빨강 대기 194프레임 중 320의 초록 오인정 0. 접근·보행자·주차는 640 그대로 (traffic_lamp.image_size). 끄기: None
        'traffic_hold_image_size': 320,
        'timeout': 1.5, 'minimum_area': 0.0006, 'confirm_frames': 2,
        # 작은 화살표부터 준비. 면적 0.15%·서로 다른 3프레임 확인 (실영상 조정 필요)
        # 빨강·노랑은 1프레임에 확정 → 녹색과 동시 검출(광고판 등)되면 바로 녹색 허가 차단 (PR #42 본문 내용, 11/14 반영)
        'confirm_frames_by_class': {'left': 3, 'right': 3, 'traffic_red': 1, 'traffic_yellow': 1},
        # arrow_vote (11/18 소민, 녹화 재현만·실차 전): 화살표는 위 연속 3에 더해 '최근 window 프레임 중 count 이상 +
        # 반대 opposite_max 이하 + 이번 프레임도 같은 화살표'여도 확정 (driver.arrow_vote). 끄기: enabled False
        # 11/18 녹화 4개: 19는 1.8 s 앞당겨짐, 05·17·20은 그대로, 틀린 확정 0. 5중 3은 20을 right로 잘못 확정해서 안 씀
        'arrow_vote': {'enabled': True, 'window': 6, 'count': 4, 'opposite_max': 1},
        'minimum_area_by_class': {
            # 신호등 0.025 → 0.003: 서행 시작 흰 선(정지선 약 1 m 앞)부터 빨강을 읽도록 (소민 11/12)
            'parking': 0.002, 'pedestrian': 0.004, 'traffic_red': 0.003, 'traffic_yellow': 0.003,
            # 11/17 혁준: left·right 0.0015 → 0.003. 멀리서(면적 0.3% 미만) 방향을 틀리게 읽음: 녹화 22개 화살표 562건에서
            # 틀림 0.2% 미만 9%·0.2~0.3% 5%·0.3~1% 3~4%·1% 이상 0~1% (전부 left를 right로). entry_1117_0342 74.3 s:
            # 기둥 3.5 m 전 면적 0.16%에서 right 4연속 → 확정 → 직진 코스에서 오른쪽 차선 변경. 0.3%면 기둥 약 1.6~2.6 m 전 확정
            'traffic_green': 0.003, 'left': 0.003, 'right': 0.003, 'slow': 0.004,
        },
        'cpu_threads': 2, 'edge_margin': 0.01,
        # 신호등 박스 안 켜진 램프 위치로 색 재확인 (traffic_lamp.py, 11/14). 밝기·채도 하한, 칸 화소 비율 하한, 둘째 칸 대비 배수
        # lamp_min_area: 이보다 작은 박스는 판별 안 함 (11/14 lap5: 먼 배경의 빨간 상자 0.003을 녹색으로 판정)
        'lamp_check': True, 'lamp_min_area': 0.005, 'lamp_value_min': 180, 'lamp_saturation_min': 60, 'lamp_min_fraction': 0.002, 'lamp_ratio': 3.0,
        # 낮은 신뢰도 신호등 박스 (11/18 혁준, 사용자 지시, track_test_1118_02~05, 소민에게 공유 필요): 정지선 앞 초록 신뢰도가
        # confidence 근처라 절반쯤 탈락 → 04번 첫 초록 뒤 20.6 s 출발(15초 규정 초과). 신뢰도 lamp_low_confidence 이상·면적
        # lamp_low_area 이상인 신호등 박스는 램프 위치 판정이 확실하면 그 색으로 인정 (traffic_lamp.box_name). 끄기: lamp_low_confidence None
        'lamp_low_confidence': 0.05, 'lamp_low_area': 0.02,
        'allow_truncated_classes': ['traffic_red', 'traffic_yellow', 'traffic_green'],
        'shutdown_timeout': 5.0,
        # 시작 직후 첫 검출을 기다리는 최대 시간(s). 그 뒤로는 검출이 끊겨도(timeout 초과·예외) 표지판 미션만 끄고 주행 계속
        'startup_wait': 20.0,
        'failure_limit': 5,   # YOLO 예외가 이 횟수 연속이면 고장으로 보고 시작 대기도 건너뜀 (추론은 계속 재시도)
    },
    'lane': {
        # 차선 = 노란색, 정지선·횡단보도·노면 표시 = 흰색 (11/8 현수막 확인)
        # 수동 노출 300 기준 노란 선 H 19~39, S 중앙 138, V 중앙 68 → 어두워도 색상으로 구분. 트랙 바닥 H 110대
        # 11/17 광주 현장(포맥스·반사 필름, 노출 600): 3차선·갈림길 구역 맨바닥이 H 18~40·S 40~60·V 40~70으로 노란 범위에 듦
        # (check_1118_01: 노란 색상 화소의 87~93 %가 바닥). 선은 S 90 이상·V 130 이상 → 하한 S 40 → 70, V 50 → 70
        'yellow_min': [18, 70, 70], 'yellow_max': [40, 255, 255],
        # 방지턱 노란 줄(폭 약 0.15 m) 제외: 버드아이뷰에서 가로·세로 모두 이 값(m)보다 두꺼운 노란 덩어리는 차선 후보에서 뺌
        # (check_1118_01 44~53 s: 줄을 차선으로 잡아 오차 ±0.3~0.44 m). 차선 폭 약 0.05 m. None이면 끔
        'yellow_max_thickness': 0.10, 'yellow_thick_margin': 0.035,
        # 정지선·횡단보도 = 흰색. 정지선 검출에만 사용 (차선은 노랑만). 예전 범위로 시작, 노출 300 기준 실측 필요
        # 11/9 통합 시험: V 165 → 110. 3차선 점선 V 135~150 (녹화 white_1109_lap 138~152 s), 정지선 247 s도 V 165에서 누락
        # 부작용: 코너 바깥 현수막 흰 테두리를 정지선으로 잡음 (83·184 s). 소민 확인 필요
        'white_min': [0, 0, 70], 'white_max': [179, 85, 255],
        # 3차선 흰 점선: 보이면 항상 차선 후보로 추가 (lane_vision.white_dashes)
        # 흰 조각 중 길이(m) 이상, 폭(m) 이하, 진행 방향 기준 기울기(도) 이하만 사용. 점선·횡단보도 실측 전 임시값
        'white_dash_min_length': 0.08, 'white_dash_max_width': 0.05, 'white_dash_max_tilt': 40,
        'white_dash_max_length': 0.5,  # 이보다 긴 흰 획은 점선 아님 (테두리 실선 제외). 점선 조각 길이 실측 후 조정
        # 노란 선 너머 흰색(트랙 밖 테두리) 제거 기준: 이 길이(m) 이상인 세로 노란 덩어리만 경계로 사용
        'yellow_edge_min_length': 0.2,
        # 점선 구간 차선 시작점: 예상 위치(중앙 ± 차로 폭 절반)에서 이 거리(m) 안의 노란 선 우선, 없으면 가장 가까운 점선
        'start_expect_tolerance': 0.13,
        'white_dash_max_missing': 3,   # 점선 구간 슬라이딩 윈도우 연속 빈 창 허용 (창 1개 약 0.09 m). 점선 간격 실측 후 조정
        'line_yellow_ratio': 0.5,      # 선 픽셀 중 노란 비율이 이 값 초과면 노란 선 (차로 판단용)
        # 버드아이뷰 범위(m): 좌우 폭, 차 앞 가까운 끝·먼 끝, 픽셀 크기
        'view_width': 2.4, 'near': 0.6, 'far': 1.7, 'width_px': 480, 'height_px': 420,
        # 한 선만 보일 때(이력 없음) 좌우 판단 경계 이동량(m), 선 곡률 split_curvature(1/m)에서 최대. label_sides 참고
        'single_line_split': 0.14, 'split_curvature': 1.7,
        # 마지막(4번) 코너 뒤 이중선: 출발 차로 아래 선과 지금 차로 왼쪽 선이 0.21~0.30 m 간격으로 나란함 (11/11 segA_1111_lap5·6)
        # 구간 S4 진입 뒤 double_line_distance(m)까지, 코너 유지 중이 아닐 때만 적용 (lane_driving.beside_start_lane)
        # 시작점: 왼쪽 칸에 칸 최대의 double_line_ratio 이상인 봉우리가 둘 이상이고 차 쪽 둘의 간격이
        #         double_line_gap_min~double_line_gap(m)이면 차 쪽 선에서 시작 (lane_vision.find_starts)
        # 좌우 판단: 노란 두 선 간격 < double_line_gap이면 이중선 → 차 쪽 선을 왼쪽 선으로 (label_sides). 실제 차로 폭 최소 0.45
        'double_line_distance': 3.0, 'double_line_ratio': 0.4, 'double_line_gap_min': 0.12, 'double_line_gap': 0.38,
        'memory_frames': 10,  # 한 선 이름 이어받기: 좌/우 선별로 마지막으로 본 뒤 이 프레임 수까지 기억 (약 0.35 s)
        'lane_width': 0.52,          # 선 중심 간격(m). 도면 직선 0.51~0.52, 11/8 카메라 보정 0.52
        # 코너 차로 폭(m)과 그 폭을 쓰는 선 곡률 범위(1/m, 시작~끝 보간). 도면: 안쪽 R0.59 → 1.7, 바깥 R0.66 → 1.5, 바깥 R1.4 완화 → 0.7
        'corner_lane_width': 0.77, 'corner_curvature': [0.8, 1.4],
        'near_curvature_distance': 1.0,   # 가까운 선 휨 방향 판단 거리(m). BEV가 약 0.6 m부터라 1 m 안쪽만 의미 (11/10)
        # pure pursuit 목표점 거리(m, 뒷축 기준, 중앙선 위 점). 도면 1번 코너 폐루프 시뮬: 0.9면 탈출 때 바깥 선 접촉,
        # 0.6~0.65 바깥 여유 최대 (5~7 cm). 직선 노이즈 고려 0.65
        'pursuit_distance': 0.65,
        # 중앙선이 모두 pursuit_distance 원 밖이고 선 기울기가 pursuit_ahead_tilt(°) 넘으면 선 따라 pursuit_ahead m 앞 점 목표
        'pursuit_ahead': 0.2, 'pursuit_ahead_tilt': 40,
        'refit_tolerance': 0.04,     # 2차식 맞춤 후 이 거리(m) 밖 픽셀 빼고 재맞춤
        # 지그재그(서행 유도선) 판별 (lane_vision.zigzag_line): 행 묶음 크기, 최소 보이는 길이(m),
        # 잔차 부호 판단 최소 크기(m), 부호 바뀜 최소 횟수, 잔차 크기 범위(m). 11/10 skip_1110_lap1 기준
        'zigzag_bin_rows': 10, 'zigzag_min_span': 0.8, 'zigzag_deadband': 0.008,
        'zigzag_min_crossings': 5, 'zigzag_residual': [0.012, 0.045],
        'windows': 12, 'margin': 0.17, 'minimum_pixels': 25,   # 슬라이딩 윈도우 개수·반폭(m)·최소 픽셀
        'lookahead': 0.9,           # 차로 중앙·방향 기준 거리(m). 목표점 없을 때 조향, 정지선 탐색·차선 변경·주차용
        'lost_grace': 1.2,           # 차선 놓친 뒤 몇 초까지 계속 갈지
        'smoothing': 0.55,           # lookahead 중앙 흔들림 완화. 1에 가까울수록 이번 프레임을 더 믿음
        'stop_min_width': 0.32, 'stop_max_thickness': 0.13,   # 정지선 최소 길이·최대 두께(m)
        # 정지선 탐색 전 흰 마스크를 차선 방향으로 펴는 세로 밀기 기울기(행/px) 하한·상한. stop_level_max 0이면 끔 (11/16, lane_vision.stop_level)
        'stop_level_min': 0.03, 'stop_level_max': 0.6,
        # 11/18 경기장 (redecide_1118_17·19·20 재생): 바닥판 이음매가 흰 선을 끊어 신호등 앞 정지선 0프레임
        # stop_seam_close(m): 정지선 탐색 마스크만 가로로 닫아 틈 메움. 0.035(7 px)에서 S4 횡단보도 선 0 → 6~12프레임,
        #   0.055(11 px)는 S0·S1의 ◇·천천히 글씨를 횡단보도 선으로 잡음 → 올리지 말 것. 0이면 끔
        # stop_white_min: 정지선·횡단보도 전용 흰색 하한. 경기장 S4 흰 페인트 V 100~135 → 110에서는 블록 판정 탈락. None이면 white_min
        'stop_seam_close': 0.035, 'stop_white_min': [0, 0, 85],
        # stop_white_ratio (11/18 오후): 정지선 흰색 하한 = 도로 위 노란 선 밝기(V 가운데값) × 이 값, stop_white_min V ~ white_min V 사이로 제한.
        #   어두운 경기장(노란 선 V 90~105) → 85~95, 밝은 연습장·경기장 S1(V 165 이상) → 110(예전 값). None이면 stop_white_min 고정
        'stop_white_ratio': 0.9, 'stop_white_min_pixels': 300,
        'abnormal_fit_residual': 0.035, 'solid_row_fraction': 0.85,
    },
    'pid': {
        'kp': 1.3, 'ki': 0.03, 'kd': 0.11, 'integral_limit': 0.25, 'derivative_alpha': 0.25,
        'feedback_gain': 0.35,  # 목표점 없을 때만 사용 (lane_driving). 곡률 앞먹임은 11/8 과대 추정으로 제거
    },
    'lidar': {
        'emergency_distance': 0.18, 'front_half_angle': 0.3, 'cluster_gap': 0.12,
        # 라바콘 후보 = 폭 small_object_min~max_width(m)인 좁은 군집, 라이다 앞 small_object_range 안
        'small_object_min_width': 0.015, 'small_object_max_width': 0.12, 'small_object_range': 1.8,
        'planning_point_range': 3.0, 'cluster_min_points': 2, 'wide_object_half_width': 0.9,
        'small_object_half_width': 0.75,
        # ---- 라바콘 (cones.py). 거리는 뒷바퀴 축 기준 m, 각도 rad ----
        # 11/12: 콘 두 줄 가정 제거 → 좌·우 콘 쌍(문) 사이 각도로 조향. 쌍 개수·간격 모름
        # 문 = 간격 cone_gate_min~max인 콘 2개. [실차] 실제 배치 보고 조정 (참고 코드 0.6~0.95, 차폭 0.225)
        'cone_gate_min': 0.35, 'cone_gate_max': 1.0,
        'cone_gate_width': 0.6,      # 콘 하나만 보일 때 쓰는 문 폭 기본값 (문을 보면 측정값으로 교체)
        'cone_group_max_width': 0.6, # 이 폭까지는 붙어 놓인 콘 덩어리로 보고 나눠서 사용. 넘으면 벽·큰 물체로 무시
        # 외곽 가벽 (11/13 도면: 맵 가장자리, 높이 200 mm, 좌측 직선 차로 중앙에서 약 0.72 m. 라이다 z 0.07 m라 스캔에 잡힘)
        # 콘 그림자·점 누락으로 끊긴 벽 조각이 붙은 콘으로 분류되어 벽·콘 사이로 통과 (11/13 합성 확인, 유경)
        # 벽 = 한 직선 위 점들이 cluster_gap 이내로 이어진 길이의 합이 cone_wall_min_length(m) 이상 (cones.wall_lines).
        # 그 직선에서 cone_wall_margin(m) 안의 점은 벽으로 보고 콘 후보에서 먼저 제외
        # cone_wall_margin 0.07: 0.05는 거리 잡음 2 cm에서 벽 점 일부가 밖에 남아 기둥이 됨 (합성 2/57 실패 → 0)
        # 주의: 벽에서 이 거리 안에 놓인 콘도 같이 빠짐 (콘 중심이 벽에서 약 0.12 m 이상이면 남음)
        # cone_wall_min_length 1.0: 붙은 콘 0.5 m + 같은 줄 콘 3개(이어진 길이 합 약 0.74 m)를 벽으로 보지 않는 값
        # cone_wall_beyond: 직선 너머 2·cone_wall_margin 밖에 점이 이 개수보다 많으면 벽이 아님 (벽 앞에 선 촘촘한 콘 줄).
        # 벽은 끊김 없이 사방을 두른 판이라 그 너머는 안 보임 (11/13 유경). 잡음으로 튄 점 2개까지 허용.
        # 세는 범위는 직선이 실제로 뻗은 구간 안으로 한정 (벽 끝 너머로 비켜 보이는 물건은 제외, 11/13 연습장 사진)
        # cone_wall_side: 'left' = 차 왼쪽 직선만 벽 (대회 = 시계 방향이라 외곽벽은 왼쪽), 'both' = 양쪽, 'off' = 벽 제외 안 함.
        # 연습장은 오른쪽에도 벽이 있어 'both'가 필요할 수 있음 → corn_test.py --wall-side로 바꿔 가며 확인 (11/13 유경)
        'cone_wall_min_length': 1.0, 'cone_wall_margin': 0.07, 'cone_wall_beyond': 2, 'cone_wall_side': 'left',
        'cone_view_distance': 1.5,   # 라이다에서 이 거리 안의 콘만 사용
        # 진입 판단: 문 중점이 라이다에서 cone_enter_distance(아래) 안, 문 중점 방위·문 방향이 차 정면에서 cone_start_angle 이내,
        # cone_confirm_count 주기(20 Hz) 연속. 코너 도는 중(문이 비스듬함)에는 켜지 않음
        # cone_start_range: 통과 중 문 없이 콘 하나만 보일 때 그 콘을 쓰는 최대 거리 (11/12부터 진입에는 안 씀)
        'cone_start_range': 1.2, 'cone_start_angle': 0.6, 'cone_confirm_count': 3,
        # 문 안쪽 통로(차 ~ 문 뒤 cone_wide_depth, 기둥에서 cone_wide_margin 안쪽)에 넓은 군집이 있으면 진입 안 함
        'cone_wide_depth': 0.6, 'cone_wide_margin': 0.08,
        # 통과 중: 진행 방향에서 cone_gate_angle 이내를 마주 보는 문만 사용 (같은 쪽 콘 2개 = 90°라 제외),
        # 문 중점 방위는 진행 방향에서 cone_gate_bearing 이내 (옆으로 크게 벗어난 물체 쌍 제외)
        'cone_gate_angle': 0.95, 'cone_gate_bearing': 0.7,
        'cone_path_gates': 4,        # 가까운 문부터 몇 개까지 이어서 볼지
        'cone_gate_step': 0.15,      # 이어지는 문 사이 최소 앞 거리 (붙은 콘이 만드는 겹친 문 건너뜀)
        'cone_lookahead': 0.5,       # 문 중점을 이은 경로에서 몇 m 앞 지점으로 조향할지
        'cone_aim_alpha': 0.5,       # 진행 방향·문 폭 갱신 비율 (1 = 이번 측정만)
        'cone_check_distance': 0.35, 'cone_margin': 0.03,   # 충돌 검사 원호 길이, 차체-콘 최소 여유
        'avoidance_speed': 0.13,     # 라바콘 통과 속도
        'cone_creep_speed': 0.05,    # 안전한 원호가 없을 때 속도 (정지 = 미션 실패라 멈추지 않음)
        # 콘이 안 보인 뒤 이 거리(m) 더 가면 차선 주행으로 복귀 (다음 문이 보이면 다시 켜짐). 도면상 라바콘 끝 선 → 코너 시작 약 0.3 m
        'small_object_memory_distance': 0.3,
        # 즉시 복귀 (11/14 유경): 지날 문 없음 + 지금 화면에 콘 색 없음 + 차선 유효 + 진행 방향 앞 이 거리(m) 안에 콘 없음
        # → 위 기억 거리를 기다리지 않고 바로 차선 주행으로 넘김. 기억으로 끌면 틀어진 방향으로 직진해 차로를 벗어남
        # 0.4: 마지막 콘을 지나치는 중에는 넘기지 않도록. 0이면 콘이 바로 앞에 있어도 넘김
        'cone_exit_clear': 0.4,
        # 라바콘 직후 차선 한 선 이름 힌트 (11/14 유경). lane_vision.single_name의 corner_left_limit 자리에 넣음
        # 왜: 탈출 직후 차가 바깥(왼쪽) 선 가까이·비스듬히 있어 왼쪽 선이 화면 중앙 오른쪽에 찍힘 → 오른쪽 선으로 오인
        #     → 차로 중앙을 그 선의 왼쪽으로 잡아 더 바깥으로 나감. 한 번 틀리면 기억으로 이어져 고착
        # 0.30: 합성 복귀 뒤 차로 중앙 이탈이 최대 0.23 m라 그보다 크게. 도면상 라바콘 끝은 4번 코너 직전이라
        #       안쪽(오른쪽) 선은 코너 사각으로 빠져 한 선만 보이면 바깥 선일 가능성이 큼 (11/14 유경 도면 확인)
        'cone_lane_left_limit': 0.30, 'cone_lane_left_distance': 1.5,
        # 라바콘 모드가 꺼진 뒤 이 거리(m) 동안 속도 상한 (11/14 lap4: 4번 코너에서 속도 출렁임)
        'cone_exit_slow_distance': 2.0, 'cone_exit_slow_speed': 0.22,
        'cone_max_distance': 2.5,    # 안전장치: 첫 진입 시 문까지 거리 + 이 값 넘게 주행하면 끝 (다시 안 켜짐). 라바콘 구간 약 2.1 m + 여유
        # 카메라 붉은색 (11/12 유경): 주최 측 안내 = 콘은 빨강에 가까운 붉은색. 11/12 cone_lane_3 진입 못 하고 차선 복귀·충돌
        # 붉은색이 보이면 진입·통과 조건 완화: 문 간격 최대 cone_red_gate_max, 좌우 cone_red_half_width, 거리 cone_red_start_range
        # 통과 중 붉은색이 보이면 콘이 안 보여도 모드 유지 (cone_max_distance 안전장치는 그대로)
        # 판정: 화면 아래 cone_red_rows 비율 영역을 1/4로 줄여 HSV, 색상 0~h_low 또는 h_high~180, 채도·밝기 하한, 화소 비율 cone_red_ratio 이상
        # 11/12 실차 red_check (노출 300, 콘 약 1 m): 콘 H 0~9·170~179, S 약 190, V 약 130 → 색 범위 맞음.
        # 노란 선(H 20~39, V 40 전후)·흰 선·검은 바닥은 판정 0. 카메라가 낮아 콘 몸통은 대부분 화면 가운데보다 위
        # (아래 50%만 볼 때 0.040, 위쪽만 0.153) → 판정 영역 아래 75%로 넓힘. 먼 콘(약 2 m)은 아래 50%에 안 들어옴
        'cone_red_h_low': 10, 'cone_red_h_high': 165, 'cone_red_s_min': 100, 'cone_red_v_min': 10,   # 11/18 20시 유경 speed c68f28b: 30 → 10 (사용자 지시, 시험 없이 반영)
        # cone_red_v_min 60 → 35 (11/18 경기장 redecide_1118_17·19·20, 유경에게 공유 필요): 어두운 조명에서 콘 몸통 밝기 V 중앙 45
        #   → 콘이 화면을 채워도 붉은 비율 0.00~0.14, 진입 기준 미달로 3회 모두 0.37~0.40 m/s로 콘 0.11 m 앞까지 감.
        #   35면 비율 0.2~0.28(다른 구간 0.00~0.01), 라이다 포함 재생에서 3번 코너가 끝나는 시점(콘 0.41~0.50 m 앞)에 켜짐
        #   → 30으로 확정 (유경 I_See_Parking acba2fd, 11/18): 진입 시각은 35와 같고 접근 신호가 더 큼(19번 최대 0.31 vs 0.26),
        #   라바콘 구간 밖 최대는 0.056~0.060으로 비슷. 조명이 바뀌면 tools/red_check.py로 재측정 (유경)
        # 11/12 유경: 판정 영역 화면 전체로 (콘 바로 앞 위치는 사진 확인 못 함). 마지막으로 보인 뒤 cone_red_memory(m)까지 유지
        'cone_red_rows': 1.0, 'cone_red_ratio': 0.03, 'cone_red_memory': 0.5,
        # 11/13 실차 사진 18장(red_1113_000127, 코너 앞 → 콘 통로): 관중석 붉은 의자만 보여도 전체 0.010~0.015
        # → 기준 0.002에서 0.03으로 올림 (콘 1개가 화면 끝에 보일 때 0.07).
        # 진입은 화면 전체 붉은 비율 cone_red_enter_ratio 이상일 때만 (11/13 유경 결정). 같은 사진: 의자 0.01, 콘 1개 0.07,
        # 코너 중 콘 줄 0.17~0.27, 통로 정면 0.30, 통로 안 0.39~0.45. 0.27(회전 중 한쪽 줄)은 진입함
        'cone_red_enter_ratio': 0.15,   # 11/18: 0.2 → 0.15. V 35에서도 진입 시점 비율이 0.22~0.27이라 여유 확보 (콘 1개 0.07보다는 위)
        # 붉은색이 보이는데 비율이 위 기준 미달일 때 라이다 문만으로 진입할지 (11/18 유경). False면 예전처럼 막음
        # 왜 True: run_1118_1931은 v_min 60이라 비율이 0.087까지만 올라 123.2~128.5 s 내내 "red지만 corridor 아님"이었고,
        # 그동안 라이다 진입까지 막혀 콘 0.24 m 앞에서야 켜짐 → 추돌·3 s 정지. 조금 보이는 상태가 아예 안 보이는 상태보다
        # 위험해지는 구조. 라이다 조건(문 쌍·정면 방위·0.3 m·넓은 군집 없음)은 그대로라 한쪽 줄·표지판 기둥에는 안 켜짐
        'cone_red_lidar_entry': True,
        'cone_red_side_width': 0.2, 'cone_red_side_top': 0.4,   # 양쪽 끝 띠 비율: 확인용 출력만 (판단에 안 씀)
        'cone_red_gate_max': 1.6, 'cone_red_half_width': 0.9,
        # 진입 시점: 라바콘 모드로 바꾸는 라이다 앞 방향 거리(m). 붉은색이 보이면 콘 후보 하나, 아니면 문 중점 기준.
        # 11/12 실차(유경): 1.2 m에서 켜면 코너 유지(고정 조향) 중에 바로 꺼져 차선 이탈 → 콘 바로 앞까지 차선 주행·코너 유지.
        # 같은 날 문 중점 직선거리 0.3 m는 안 켜짐 (차로 중앙과 문 중점이 옆으로 어긋남) → 붉은색 + 콘 앞 방향 거리로 변경
        'cone_enter_distance': 0.3,
        # 라바콘 접근 서행 (11/18): S3에 들어와 라바콘이 한 번도 안 켜진 동안 구간 거리 cone_approach_distance(m)까지 cone_approach_speed.
        #   색 판정이 또 빗나가도 0.39 m/s로 콘에 닿지 않게 하는 예비 장치 (라이다 단독 진입은 콘 0.3 m 앞에서야 켜짐). None이면 끔
        'cone_approach_speed': 0.2, 'cone_approach_distance': 2.0,
        # 11/15 인계 조건 (cones.start_gate): 코너 유지 중에는 차 진행 폭(좌우 cone_hold_half_width m) 안 콘이 바로 앞일 때만,
        # 유지가 아니면 문을 정면으로 볼 때(문 중점 앞 cone_face_distance·옆 cone_face_lateral 안, 방향 cone_face_angle rad 안) 미리 인계
        'cone_hold_half_width': 0.2, 'cone_face_distance': 1.0, 'cone_face_lateral': 0.2, 'cone_face_angle': 0.35,
        # 11/18 유경 완화 (코너 유지 중 진입 지연): 유지 중에도 붉은색이 화면에 차고(corridor) 문을 정면으로 보면
        # cone_hold_face_distance 안에서 미리 인계. 폭(cone_face_lateral)·방위(cone_face_angle) 조건은 그대로라
        # 콘 한 줄·비스듬한 문에는 안 켜짐. 0이나 None이면 예전처럼 유지 중 미리 인계 없음
        # 근거 run_1118_1931 재생: 정면 문이 124.4 s에 0.51 m 앞으로 들어왔는데 유지 중이라 못 받음.
        # 그 구간은 driver 접근 서행(state=='lane')도 안 걸려 corner_speed로 콘에 다가감.
        # 이 녹화에서 진입이 124.84 → 124.75 s로 0.1 s 빨라짐 (그 앞 0.6 s는 문이 정면 방위 밖이라 완화와 무관)
        # 값 0.6: 유지가 아닐 때의 1.0보다 짧게 (아직 도는 중이라 먼 문은 코너 바깥 줄일 수 있음)
        'cone_hold_face_distance': 0.6,
    },
    'missions': {
        'fork_prepare_speed': 0.18,  # S2 방향 확정 전 상한. 늦은 진입 완화
        'fork_exit_speed': 0.3,      # 방향 확정 뒤 목적 차로에서 분기 통과 중 상한 (11/16, 실차 전)
        # 표지판 확정 전 중앙 차로 정렬(fork.center_request). 11/16 끔: 회피 직후 장애물 1.7 m 앞 가운데 차로로 복귀하다 정지 (main_1116_1614 192.3 s)
        'fork_center_align': False,
        'sign_cooldown': 12.0, 'fork_bias': 0.55,
        # 11/15 주최 답변: 정지 거리(정지선 ±5 cm) 판정은 앞바퀴 → 목표 = 앞축이 선 가까운 모서리 위.
        # main_1115_1904: stop_offset 0.088 + stop_tolerance 0.025 → 앞축이 선 0.111 m 앞 정지(보행자·신호등 모두) = 범위 밖
        # 실제 정지 지점 = stop_offset + stop_tolerance 근처. stop_min_speed: 마지막 접근 최저 속도(m/s, 너무 느리면 안 움직임)
        # pedestrian_wait: 같은 녹화 실제 정지 3.2 s (규정 3초 이상, 여유 부족) → 4.0
        'pedestrian_wait': 4.0,
        'stop_offset': 0.0, 'stop_tolerance': 0.01, 'stop_min_speed': 0.025, 'stop_approach_speed': 0.13,
        # 정지 접근 속도 2단 (11/16, 실차 전): 선까지 stop_slow_distance(m) 넘게 남으면 stop_far_speed, 신호등 정지선을 아직 못 찾았고 빨강이 작으면 traffic_search_speed
        'stop_far_speed': 0.25, 'stop_slow_distance': 0.7, 'traffic_search_speed': 0.25,
        'traffic_require_stop': True, 'traffic_min_stop': 0.3, 'stop_gain': 0.9, 'stopped_speed': 0.012,
        'approach_timeout': 15.0, 'stop_line_memory': 8.0,
        'line_match_tolerance': 0.1,
        # 미션 순서는 구간 관리자가 결정 (위 sections, section.py SECTION_MISSIONS). 예전 sequence·enforce_order 제거
        # 신호등 감시 (소민 11/12, 실차 전): S4에서 화면 traffic_watch_area 이상 신호등이 보이면 traffic_watch_hold(s) 동안
        # traffic_watch_speed로 감속. 정지선 없이 빨강 박스가 traffic_stop_area 이상이면 그 자리 예비 정지
        'traffic_watch_area': 0.0008, 'traffic_watch_hold': 1.5, 'traffic_watch_speed': 0.18, 'traffic_stop_area': 0.04,
        # 신호등은 횡단보도 바로 앞 선(너머에 블록 3개 이상)에만 정지. 앞축 기준 0.088 m = 앞 범퍼(앞축 +0.058) 3 cm 앞
        # traffic_stop_offset 11/15: −0.045 → −0.133 (stop_offset과 같은 폭 이동, 연습장에서 먼 측정이 짧게 나오던 것 보정)
        # 11/18 경기장 redecide_1118_19: 먼 측정(0.9 m)도 정확 → 보정 −0.133이 그대로 초과가 되어 앞축이 선 0.12~0.14 m 지나 정지,
        #   신호등이 화면 오른쪽 끝에 걸려 녹색 미검출 9.4 s 대기. → 0.0 (모자라게 서는 쪽이 신호등이 화면에 남음). 실차 재확인 필요
        'traffic_crosswalk_only': True, 'traffic_stop_offset': 0.0,
        # 신호등 구간 서행 (11/18 경기장): 신호등 앞에 ◇·천천히·원형 서행 표지판이 있음. 예전에는 S4 0.75 m 뒤 0.38 m/s로 가속
        #   → 빨강을 면적 0.158(코앞)에서야 확정·정지선 측정 전에 통과 (17·20번). S4에서 신호등을 끝내기 전이면
        #   구간 거리 traffic_zone_distance(m)까지 traffic_zone_speed로 제한 (차선 주행·정지 접근 모두). None이면 끔
        #   거리 상한: 신호등을 끝내 못 봐도 횡단보도(정지선 약 1.9 m, 뒷선 약 2.4 m)를 지나면 풀림
        'traffic_zone_speed': 0.18, 'traffic_zone_distance': 3.0,
        # 신호등 정지선 거리 조건 (11/18 오후, redecide_1118_26·28): S4 순서는 흰 선 → ◇ → 천천히 → 정지선 → 횡단보도.
        #   첫 선(S4 약 0.80~0.84 m)이나 '천천히' 글씨(약 1.35~1.4 m)가 횡단보도 선으로 잘못 판정돼도 서지 않게,
        #   S4 구간 거리 기준 위치가 traffic_line_min_distance(m)보다 앞인 선에는 신호등 정지를 하지 않음. 정지선은 약 1.70~1.80 m (앞축)
        #   한 바퀴 주행에만 적용 (신호등·주차 지점 재출발은 구간 거리가 트랙 기준이 아님). None이면 끔
        'traffic_line_min_distance': 1.55,
        # 11/14: 보행자 stop_offset 0.015 → 0.088 (lap3 보행자 선 0.04 m 앞을 선 밟음으로 봄). 11/15 앞바퀴 기준 확정으로 0.0
        # 신호등 박스 선별 (driver.traffic_filter): 따라가던 박스와 IoU traffic_box_iou 이상 또는 면적 traffic_main_ratio배 이상만 사용
        'traffic_main_ratio': 0.5, 'traffic_box_iou': 0.2,
        'slow_min_speed': 0.05,   # 이 속도(m/s) 이상으로 움직일 때만 서행 표지판 수용 (출발 전 소진 방지)
        'traffic_max_wait': 60.0,   # 신호등 정지 후 녹색을 못 본 채 이 시간(s) 넘으면 포기하고 진행. 신호 주기 확인 후 조정 (소민)
        # 목표 끝 차로 도착 후 출구 추종 거리(m). 정렬·회피 중에는 차감 없음
        # 11/15: 3.0 → 2.5. 표지판 통과(구간 거리 4.2~4.8 m) + 2.5 m = 교통섬 꼬리(약 6.7 m)는 지나고 3번 코너(약 7.2 m) 전
        # (fix_1115_1949 B 길, main_1115_1904 A 길). 이 거리 동안만 바깥선 하나 추종(fork_outer_only), 뒤에는 일반 차선 주행
        'fork_distance': 2.5,
        # 분기 입구 1순위 = 표지판 통과: 화살표를 면적 fork_sign_pass_area 이상으로 본 뒤 fork_sign_pass_distance(m)·
        # fork_sign_pass_seconds(s) 넘게 안 보이면 입구 (두 녹화에서 사라지기 직전 면적 0.009~0.08)
        'fork_sign_pass_area': 0.006, 'fork_sign_pass_distance': 0.15, 'fork_sign_pass_seconds': 0.5,
        # 안전장치 (유경 11/16 run_1116_2113, 실차 전): 목적 차로 도착 뒤 입구 판정이 안 나도 S2 주행 거리가 이 값(m)이면 입구로 처리
        # (그 녹화: follow가 끝까지 유지돼 fork_remaining 2.5 → 라바콘 모드 차단, 콘 앞 정지). 11/16 녹화의 입구 5.2~5.6 m
        # None이면 끔. 합성 시험 장면은 3차선이 더 길어 끔 (three_lane.py, lane_change_max_distance와 같은 이유)
        'fork_enter_distance': 5.4,
        # S2 주행 거리(m). 앞쪽 장애물 구간에서는 화살표 검출만 누적하고 후반부터 분기 방향 확정
        'fork_decision_distance': 3.5,
        # 반대편 끝 차로에 있을 때는 표지판이 이 거리(m) 넘게 안 보여야 그 길로 진행 (가운데 차로는 기존 점선 판정)
        'fork_wrong_lane_distance': 0.4,
        # 분기 입구 뒤 이 거리(m)까지만 바깥선 하나 추종 (교통섬 앞머리의 비스듬한 선 구간). 뒤에는 두 선 모두 사용
        'fork_outer_distance': 0.8,
        # 화살표 박스 선별 (driver.arrow_filter): 가로:세로 범위, S2 직선 방향에서 허용 틀어짐(°), 영상 가로/세로 비
        'fork_sign_aspect': (0.6, 1.6), 'fork_sign_max_turn': 20.0, 'fork_sign_image_aspect': 4/3,
        # fork_sign_bearing (11/17): 틀어진 자세(fork_sign_max_turn 초과)여도 박스의 도로 기준 방위가 섬 표지판 쪽이면 인정
        # (driver.arrow_on_island). 차가 섬 옆 차로면 섬 쪽으로 −5~wide(°), 가운데 근처면 ±narrow(°). 끄기: False
        'fork_sign_bearing': True, 'fork_sign_fov': 71.0, 'fork_sign_bearing_wide': 40.0, 'fork_sign_bearing_narrow': 15.0,
        # 방향 확정 뒤 반대 화살표가 이 프레임 수만큼 연속이면 목적지 교체 (입구 전까지. 검출 약 3.5 Hz → 약 1.4 s)
        'fork_relock_frames': 5,
        # 분기 입구 = 흰 점선 끝 (소민 11/12, 실차 전). 점선 사이 빈칸 최대 길이(m): 점선 끝이 화면 먼 끝(far)보다
        # 이만큼 넘게 가까우면 '늦음' → 새 차선 변경 금지. 점선이 끝나고 entrance_confirm(m) 더 가면 입구 도착
        # 11/12: 0.3 → 0.45 (빈칸 + 화면 끝 잘린 조각 최소 길이 0.08 여유. 실제 점선 빈칸 실측 후 조정)
        'fork_dash_gap': 0.45, 'fork_entrance_confirm': 0.3,
        'pedestrian_require_line': True,
        # 11/17 현장: 보행자 표지판이 횡단보도 뒷선 쪽에 있음 → 앞선 0.24 m 전에야 면적 1.2 % → 앞선 통과 뒤 확정 → 뒷선 정지
        # 확정 면적 0.012 → 0.004(앞선 약 0.7 m 전), 접근 시작은 횡단보도 앞 선(line_kind crosswalk)에서만.
        # 표지판이 pedestrian_near_area 이상으로 크게 보이면 줄무늬 판정 없이도 접근 (예전 동작)
        'pedestrian_crosswalk_only': True, 'pedestrian_near_area': 0.012,
        'trigger_line_max_distance': 1.5, 'trigger_line_freshness': 0.4,
    },
    'parking': {
        'enabled': True, 'test_wait_for_sign': False, 'test_drive_until_sign': False,
        # parking_real2: 카메라로 도색 칸을 만들고 LiDAR FREE가 확인되면 기존 조향 목표로 고정.
        # 두 설정이 모두 아래 값일 때만 실제 selected에 연결된다.
        'grid_enabled': True, 'grid_observe_only': False, 'grid_enable_actuation': True,
        'grid_slot_width': .50, 'grid_slot_depth': .59, 'grid_size_tolerance': .12,
        'grid_roi_top': .42, 'grid_hough_votes': 18, 'grid_min_length_fraction': .08,
        'grid_stable_frames': 3, 'grid_track_distance': .15, 'grid_track_alignment': .90,
        'grid_occupancy_padding': .03, 'grid_free_clearance': .02,
        'grid_yellow_min': [18,40,50], 'grid_yellow_max': [40,255,255],
        # 흰색 채도 상한은 85 유지 (11/18 혁준 측정). 45로 낮추면 노출 300 녹화(06·07·08·12)는 좋아지지만 노출 600 녹화(03·04)에서는
        # 기준점을 놓치고 더 뒤 지점에 틀이 놓임 → 노출에 따라 달라지는 값이라 손대지 않고, 아래 template_anchor_after_start로 거름
        'grid_white_min': [0,0,95], 'grid_white_max': [179,85,255],
        'grid_right_midpoint_max': .15, 'grid_history_frames': 6,
        'grid_maximum_extrapolation': .14,
        'grid_entrance_right_min': -.85, 'grid_entrance_right_max': -.05,
        # 11/18 혁준 (parking_grid_relaxed.py): 칸 사이 선이 입구 쪽 grid_relaxed_visible_min(m)만 보여도 칸으로 추정. 끄기: grid_relaxed False
        # grid_relaxed_single: 가로선이 하나만 보여도 그 옆에 칸을 가정. 11/18 녹화 재생(redecide_1118_03·04)에서 주차 줄 시작 전
        #   바닥을 빈 칸으로 고름 → 끔. 아래 template_enabled가 켜져 있으면 도색 칸 경로(_observe_grid) 자체가 안 돎
        'grid_relaxed': True, 'grid_relaxed_visible_min': .15, 'grid_relaxed_single': False,
        # 11/18 실차: 가로 흰 선으로 구간을 arm하고 황색→흰색 세로선 전환점에서 3칸 생성.
        # 이 모드에서는 장애물 두 개 사이 gap이 목표를 선택하지 않는다.
        'template_enabled': True, 'template_slot_count': 3,
        'template_start_frames': 2, 'template_anchor_frames': 2,
        'template_anchor_distance': .18, 'template_anchor_alignment': .94,
        'template_roi_top': .40, 'template_hough_votes': 18,
        'template_min_length_fraction': .07, 'template_max_line_gap_px': 18,
        'template_min_ground_length': .18, 'template_start_min_length': .35,
        'template_ground_far': 2.5, 'template_orientation_ratio': 1.8,
        # 기준선은 차로 오른쪽 선(옆 약 -0.26 m)만 인정 (11/18 혁준, redecide_1118_06: -1.1이던 때 1행 뒷선(옆 -0.95)에 틀이 놓여
        # 칸이 한 줄 바깥에 그려짐. 성공한 03·04·05는 모두 옆 -0.25~-0.26의 COLOR_TRANSITION)
        'template_right_min': -.55, 'template_right_max': -.05,
        # 회전 시작까지 이 거리(m) 안에 든 칸만 선택 (approach_distance_limit 1.2보다 작게). 멀리서 고르면 접근 한계에 걸려 실패
        # (06: 1.26 m 전 선택 → 0.07 m 남기고 APPROACH_DISTANCE_LIMIT). 그 전까지는 parking_scan 속도로 계속 전진
        'template_select_max_remaining': 1.0,
        # 기준점은 시작선(차로 안 가로 흰 선)보다 이 거리(m) 넘게 앞쪽일 때만 인정. 시작선 추적은 template_start_track_distance 안의 관측만 반영
        'template_anchor_after_start': .10, 'template_start_track_distance': .25, 'template_start_lane_half': .45,
        # 기준 선분 방향이 차 진행 방향에서 이만큼(기울기) 넘게 틀어져 있으면 제외. 2 m 밖 짧은 선분은 방향이 20° 넘게 틀려 칸이 차로 쪽으로 그려짐
        'template_anchor_max_slope': .12,
        # 줄 시작을 시작선 + template_start_to_row(m)로 잡음 (parking_start_anchor.py, 11/18 혁준). 끄면 색 전환 기준점 방식
        # 0.45 = 연습 매트 값: 0.43으로 재현한 여섯 주행(03·04·06·07·08·15)에서 라이다상 표지판 받침보다 평균 0.02~0.03 m 짧아 올림
        # **경기장에서 시작선 ~ 주차 줄 시작(표지판) 거리를 줄자로 확인**
        # 시작선이 template_start_fix_distance(m) 안에 들어와 template_start_frames_min번 넘게 보였을 때 틀 확정
        # 0.65: 카메라 바닥 거리 추정이 1 m 밖에서는 10~20% 짧게 나옴(라이다 대조, 1.1 m에서 틀이 0.12~0.24 m 앞당겨짐) → 가까울 때만
        'template_anchor_from_start': True, 'template_start_to_row': .45,
        # 첫 칸(0번)을 고르면 조향 목표를 다음 칸 쪽으로 이만큼(m) 옮김 (parking._observe_template, 11/18 혁준, 정윤에게 공유 필요)
        #   표지판 받침이 첫 칸 시작 모서리에 있어 회전 안쪽 옆면이 스침. 22번 재생: +0.05 여유 0.02(막힘), +0.08 여유 0.05(회전 시작), +0.11 여유 0.27
        #   0.08이면 차 옆면 ~ 0·1번 칸 경계선 약 0.06 m (칸 폭 0.50, 차 폭 0.225). 더 올리면 선을 밟음. 0이면 끔. 실차 미확인
        # 11/18 17시: 정윤 757b563 병합으로 turn_radius 0.615 → 0.55(회전 시작이 6.5 cm 늦어짐) → 0.08 → 0.02로 줄임
        # (첫 칸 회전 시작 지점은 redecide_1118_30 성공 때와 0.5 cm 차이로 유지)
        'template_first_slot_shift': .02,
        'template_start_fix_distance': .65, 'template_start_frames_min': 3,
        'template_row_lateral': -.26, 'template_row_lateral_window': .12, 'template_row_line_far': 1.6,
        'template_row_line_band': .08, 'template_row_fit_slope': False, 'template_row_fit_length': .30,
        'template_transition_tolerance': .24,
        # 11/18: 반사된 노란 차선(채도 48~)을 흰색에서 제외. 전환점은 차로 가로 시작선 뒤
        # start_to_lot_min~max(m)에서만 인정 (도면 0.50, 사진 0.47). 시작선은 |y|≤start_max_lateral
        'template_white_exclude_min': [10,30,60], 'template_white_exclude_max': [45,255,255],
        'template_start_max_lateral': .35, 'template_start_to_lot_min': .25, 'template_start_to_lot_max': .90,
        # 전환점 좌우 위치(오른쪽 차로 경계)·차로 대비 각도(rad) 범위, 시작선 추적 관측 허용 차이(m)
        'template_anchor_lateral': [-.50,-.12], 'template_anchor_max_angle': .26, 'template_start_track_gate': .30,
        # 선택 후 2프레임 연속 점유되면 다른 칸으로 전환. 회전 직전에는 차량 폭을 포함한 원호 검사.
        'template_selected_occupied_frames': 2,
        # 카메라 템플릿을 LiDAR 군집에 맞춰 작은 평행이동만 허용. 칸 크기·간격·방향은 변경하지 않는다.
        # 11/18 redecide_1118_08: 진행 방향 오차 약 0.21 m로 상한 0.15에 걸림. 큰 오차는 가까운 카메라 재보정이 담당.
        # 상한은 칸 간격 절반(0.257) 미만 필수: 0.30이면 옆면 하나만 보일 때 한 칸 밀린 해가 같은 점수 (재생 확인)
        # 깊이 방향은 물체 차로 쪽 끝이 입구선 밖일 때만 이동(한쪽 벌점). 옆면 u_min = -LIMO 폭/2 가정
        'template_lidar_offset': [0.,0.], 'template_fusion_max_offset': .20,
        # 11/18 track_test_1118_04·05: 둘째 줄 물체를 담으려 칸을 0.14 m 깊게 밀어 가운데 칸 OCCUPIED → 칸 안쪽 이동은
        # max_deeper_offset 0.05까지만 (차로 쪽은 입구 밖으로 나온 물체용으로 normal_offset 0.15 유지).
        # 차 쪽 면이 칸 입구에서 max_face_depth(m)보다 깊은 군집(둘째 줄)은 정렬 점수에서 제외. 빈 칸 판정은 앞쪽 occupancy_depth(m)만
        'template_fusion_max_normal_offset': .15, 'template_fusion_max_deeper_offset': .05,
        'template_fusion_max_face_depth': .40,
        'template_occupancy_depth': .50, 'template_fusion_obstacle_half_width': .1125,
        'template_fusion_face_weight': 4.,
        'template_fusion_step': .03, 'template_fusion_smoothing': .35,
        'template_fusion_roi_padding': .25, 'template_fusion_cluster_distance': .12,
        'template_fusion_cluster_min_points': 3, 'template_fusion_offset_penalty': 2.,
        'template_fusion_obstacle_front_depth': .08,
        # 도면 3칸 x 6110~7650 = 1.54 m → 칸 중심 간격 0.513 (폭 0.50 + 선 두께)
        'template_slot_pitch': .513,
        # 래치 뒤 가까운(0.70~0.95 m) 황색→흰색 전환점으로 진행 방향만 재보정. 1회 반영 비율 gain. 11/18 재생: 0.9 m 밖은 LiDAR 대비 0.1~0.3 m 짧음
        'template_refine_enabled': True, 'template_refine_range': [.70,.95],
        'template_refine_gain': .5, 'template_refine_max_jump': .35, 'template_refine_max_normal': .08,
        'template_refine_consistency': .06,
        'turn_path_check_distance': .90, 'turn_path_margin': .03,
        # 회전 직전 원호가 막히면 정지 후 새 스캔 block_scans회·block_seconds초 동안 계속 막힐 때만 칸 포기
        'turn_path_block_scans': 4, 'turn_path_block_seconds': 1.0, 'turn_path_self_margin': .01,
        # 11/18 redecide_1118_33: 표지판 인식 뒤 시작선을 보자마자 0.12 m/s 서행 → 회전까지 2.4 m·22 s.
        # 시작선 뒤에도 차선 주행 유지(YOLO 계속), 상한 template_cruise_max_speed. 서행 전환은
        # 표지판 화면 소실 / 칸 회전점이 감속 거리 안 / 시작선 뒤 armed_distance(m) 중 먼저. 서행 속도 search_speed
        # 0.30 근거: 표지판은 첫 칸 회전점 약 0.4 m 앞에서 화면을 벗어남, 0.4 m/s 감속 거리 0.49 m·0.30은 0.35 m
        'template_cruise_max_speed': .30, 'template_slow_on_sign_passed': True,
        'template_cruise_armed_distance': 2.5, 'search_speed': .18,
        # 신호등을 끝내지 않았으면 S4 진입 뒤 이 거리(m)부터 주차 표지판 수용. 도면 추정(횡단보도 끝 약 3.5 m, 주차 시작선 약 5.1 m) → 실차 조정
        'after_traffic_distance': 4.0,
        # 표지판 없이 주차 단계 시작 (11/18 경기장 redecide_1118_18·19·20, 정윤에게 공유 필요): P 표지판을 right로 읽고 parking은
        #   화면 끝 잘린 박스로만 나와 확정이 안 됨 → 18·19번은 주차 단계 미시작. 표지판이 첫 칸 모서리라 표지판으로 시작하면
        #   시작선이 이미 뒤(20번 WAITING_FOR_START_LINE). 시작선 기준 틀은 주차 단계가 시작선 전에 켜져 있어야 함.
        #   한 바퀴 주행: S4 구간 거리 auto_start_distance(m)부터 (횡단보도 뒷선 약 2.4 m, 시작선 약 4.2 m, 앞축 기준)
        #   주차 지점 재출발(start_stage parking): 출발 뒤 auto_start_placed(m). 신호등 지점 재출발: 신호등을 끝낸 뒤 auto_start_after_traffic(m)
        #   표지판으로 시작하는 기존 조건은 그대로 둠. auto_start False면 예전과 같음
        'auto_start': True, 'auto_start_distance': 3.0, 'auto_start_placed': 0.2, 'auto_start_after_traffic': 0.9,
        'command_deadline': .65, 'scan_speed': .12, 'turn_speed': .12, 'turn_steering': -.35,
        # 최초 표지판부터 탐색. 목표 감속 거리에서 서행, 표지판 통과만으로 감속 제외
        'cruise_search_enabled': True, 'cruise_max_speed': .4, 'trial_cruise_max_speed': .2, 'cruise_timeout': 30., 'cruise_distance_limit': 8.,
        'slow_on_sign_passed': False, 'no_target_cruise_seconds': 1.0, 'no_target_cruise_distance': .35,
        'sign_near_area': .012, 'sign_exit_center': .90, 'sign_exit_min_area': .006,
        'sign_gone_frames': 3, 'sign_gone_seconds': .35,
        'cruise_reaction_time': .25, 'cruise_braking_acceleration': .30, 'cruise_slow_buffer': .10,
        'turn_entry_max_speed': .135, 'turn_entry_speed_tolerance': .015,
        'turn_entry_settle_seconds': .25, 'turn_entry_settle_distance': .05,
        # 원호 유지 후 정렬로 전환. 선이 순간적으로 맞아 보여도 회전각·입구 진입 확인 필요
        'arc_min_angle': 1.134, 'arc_align_heading': .35, 'arc_align_lateral': .18,
        'arc_entry_tolerance': .05, 'arc_align_frames': 3,
        'arc_release_angle': 1.484, 'arc_release_heading': .15,

        # 양수 offset: 더 일찍 회전. 음수: 더 늦게 회전. 시작 오차·초과 허용은 m
        'approach_start_offset': 0., 'approach_start_tolerance': .02, 'approach_passed_tolerance': .06,
        'approach_timeout': 15., 'approach_distance_limit': 1.2,
        # 주차 전용 PD. 도색 칸 폭 사용, 장애물 사이 폭과 분리. I항 0으로 초기 적용
        # 11/14: 끔. lap4에서 경계선 연결이 한 번도 안 됨(NO_ASSOCIATED_PARKING_BOUNDARY), 연결 끊기면 정지 대기 경로 있음 → 실차 확인 뒤 켬
        'pd': {'enabled': False, 'kp': 1.5, 'ki': 0., 'kd': .08,
               'integral_limit': .10, 'derivative_alpha': .25, 'heading_weight': .35,
               'reference_distance': .55, 'activate_heading': .70, 'full_heading': .20,
               'slot_width': .50, 'width_tolerance': .12, 'center_association': .30,
               'single_association': .12, 'direction_tolerance': .25, 'parallel_tolerance': .15,
               'merge_distance': .035, 'minimum_ground_length': .10,
               'ground_near': .30, 'ground_far': 2., 'maximum_extrapolation': .20,
               'maximum_lateral': 1., 'feedback_timeout': .30, 'steering_rate': .8,
               'arc_kp': 2., 'arc_observe_heading': 1.15, 'arc_correction_limit': .12, 'arc_min_steering': .23, 'align_min_steering': .20},

        # 11/14: 회전 90° = 반경 0.615 × π/2 = 0.97 m·8 s → 상한 0.85 m·8 s로는 79°에서 실패 종료 (lap4). 1.2 m·13 s로
        'search_timeout': 45., 'search_distance_limit': 5., 'turn_timeout': 13., 'turn_distance_limit': 1.2,
        # 실측 회전 반경(m, 조향 −0.35, lap4 IMU 69°·0.756 m). 회전 시작 시점(turn lead) 계산에 사용. None이면 축거/tan(조향)
        # 11/18 redecide_1118_23·34·35 IMU 실측 0.539~0.574 (조향 명령 -0.365). 0.615면 회전을 약 6 cm 일찍 시작해
        # 칸 안쪽(이전 칸 장애물·표지판 받침 쪽)으로 치우침 → 가운데 칸 회전 경로 여유 0.017 m로 포기 (33)
        'turn_radius': .55,
        # 방향 정면 정지는 보조 종료. 흰 선 소실 정지는 방향·입구 조건 없이 즉시 우선 적용
        'turn_done_heading': .05,
        'sensor_timeout': .7, 'motion_timeout': 4., 'body_margin': .015,
        'slot_depth': .59, 'view_near': .30,
        'gap_view_far': 3., 'gap_view_right': 1.5, 'gap_cluster_distance': .12,
        'gap_cluster_min_points': 3, 'gap_object_min_width': .08, 'gap_object_max_width': 1.,
        'gap_edge_padding': .025, 'gap_front_padding': 0., 'gap_row_tolerance': .18,
        'gap_extra_width': .04, 'gap_max_width': .85,
        'occupancy_edge_margin': .015, 'occupancy_min_hits': 3, 'free_ray_fraction': .65,
        'white_roi_top': .50, 'white_roi_side_margin': .02, 'white_hough_votes': 20,
        'white_min_length_fraction': .12, 'white_min_vertical_fraction': .12,
        # 장애물 모서리를 흰 선으로 오인해 38도에서 멈춘 11/18 bag 대응.
        'white_loss_frames': 1, 'white_loss_min_angle': 1.134, 'white_acquire_timeout': 1.5,
        # 11/18 redecide_1118_35: 최소 회전각 전에 사라진 선으로는 정지하지 않음 (셋째 칸 65°에서 덜 들어간 채 정지)
        'white_loss_requires_seen_after_arm': True,
        # 템플릿 모드 흰 선 소실 정지 최소 회전각 80° (track_test_1118_02: 71°에서 정지, 방향 정지는 84~88°)
        'template_white_loss_min_angle': 1.396,
    },
    # 카메라 화면 한 장으로 갈 수 있는 거리(m) 상한: 속도 <= max_image_travel / 화면 나이 (loop.py)
    # 11/18 유경: 0.04 -> 0.06. 화면 나이가 약 0.1 s라 0.04에서는 base_speed를 올려도 0.4 m/s에서 잘림
    # 부작용: 한 장이 오래된 동안 최대 6 cm까지 눈 감고 가므로 차선 반응이 그만큼 늦음 (차선 담당 공유 필요)
    'max_image_travel': 0.06, 'curve_speed_gain': 0.6,
    # 3차선 점선 추종 (3lane_chu_js 시제품, 11/19 추추, dash_lane.py). 합성 시험만, 실차 전. 끄기: enabled False → 기존 lane_change 그대로
    # 차로 사이 흰 점선(±0.26)에 차 중심을 두고 달림. 장애물은 라이다로 비는 옆 위치를 골라 몇 cm만 비켜 감
    'dash_lane': {
        # 11/19 09:22 끔 (사용자 지시, 시험 주행용): breakfastv3_1119_02에서 넘겨줄 때 차로 번호가 한 칸 틀려 두 칸 가로지르기가 바깥선 너머로 감
        'enabled': False,
        'default_side': 1,           # 표지판 전에 탈 점선: +1 왼쪽(1|2 사이, A 쪽), −1 오른쪽(0|1 사이, B 쪽)
        'speed': 0.30,               # 점선 위 유지 속도 (m/s). 기존 S2 상한 fork_prepare_speed 0.18 대신 적용
        'shift_speed': 0.22,         # 목표 옆 위치까지 settle(m) 넘게 남았을 때
        # 비는 옆 위치가 없을 때: 앞 장애물까지 stop_distance 넘으면 approach_speed로 천천히, 안이면 그 주기는 기존 방식에 맡김(handoff)
        # stop_distance는 기존 lane_change.stop_distance(0.6)와 같게: 더 다가간 뒤 넘기면 기존 후진 재시도가 못 들어감 (합성 시험 backup_tight)
        'approach_speed': 0.15, 'stop_distance': 0.6, 'handoff': True,
        'handoff_early': True,       # 비는 위치가 없으면 거리와 상관없이 바로 기존 방식에 맡김 (False면 stop_distance까지 천천히 다가간 뒤)
        'settle': 0.06,
        'lookahead': 0.45,           # pure pursuit 목표점 앞 거리 (m, 뒷축 기준)
        'slope': 0.35,               # 경로 모델 옆 이동 기울기 (앞 1 m당 옆 m, 약 19°). 0.45는 차가 24° 넘게 틀어짐 (합성 시험)
        'start_near': 0.8,           # 가장 가까운 점선 끝이 이 앞 거리(m) 안이어야 시작 (3차로 구간 진입 확인)
        # 진입 노란 선 보호 (11/19 사용자: 진입 때 너무 일찍 조향해 노란 선 밟음·넘음): 첫 점선(구간 시작)을 차체 뒤끝까지 지나기 전에는
        # 들어온 1차로 노란 선을 가상 벽으로 경로 검사 → 옆 이동이 그만큼 늦게 시작. zone_refresh: 구간 시작이 이 앞 거리(m) 넘게 남았을 때만 화면으로 다시 맞춤
        'entry_wall': True, 'zone_refresh': 0.7,
        'corner_turn': 30.0,         # S2에서 오른쪽으로 이 각도(°) 넘게 돌면 3번 코너로 보고 끔 (기존 corner3 15°는 회피 중 틀어짐에 걸림)
        # 경로 검사 차체 여유 (m) 3단계: soft → margin → tight(approach_speed로 천천히). 앞 단계에 비는 위치가 있으면 그 단계에서 고름
        # 점선 위 + 양옆 차로 중앙 장애물(폭 0.22)이면 이론 여유 0.0375 → 위치 추정 오차 몇 mm면 0.035로는 못 지나감 (합성 시험 venue)
        'soft_margin': 0.07, 'margin': 0.025, 'tight_margin': 0.012,
        'delay': 0.15,               # 경로 모델: 옆 이동이 시작되기까지 앞 거리 (m, 조향 지연·방향 전환)
        'y_max': 0.52,               # 차 중심 옆 위치 한계 (m, 가운데 차로 중앙 기준) = 바깥 차로 중앙
        'step': 0.005, 'hold_weight': 0.3,  # 후보 간격 (0.01은 폭 5 mm 틈을 못 찾음), 지금 목표 유지 가중치 (좌우로 튀지 않게)
        'window': 1.8,               # 장애물로 볼 앞 거리 (m). 1.4는 바깥 차로 장애물을 늦게 봐 옆에 끼임 (합성 시험 normal)
        'wall_y': 0.72,              # 이보다 바깥(옆 위치) 점은 벽·트랙 밖으로 보고 제외 (노란 바깥선 ±0.78)
        'fix_gain': 0.5, 'fix_window': 0.15,   # 점선 보정 비율, 추정과 이만큼 넘게 다르면 보정 안 함
        'dash_memory': 2.0,          # 점선 보정 없이 이 거리(m) 넘게 가면 기존 방식으로 넘김
        'lane_tolerance': 0.12,      # 차로 중앙에서 이 거리 안이면 '그 차로에 있음' (갈림길 입구 판단용)
        'exit_slope': 0.6,           # 입구 앞 반 칸·반대쪽 이동 기울기 (0.5는 extend_right 늦은 표지판에서 섬 기둥 반대쪽, 0.7은 reverse_right 기둥 접촉)
        'exit_distance': 1.2,        # 분기 입구까지 이 거리(m) 안이면 목적 바깥 차로 중앙으로 반 칸 이동
        # 표지판 확정 전 입구 center_distance(m) 안이면 가운데 차로 중앙으로 (어느 쪽 표지판이든 0.52 m만 옮기면 됨, 11/19 사용자 제안)
        'center_before_sign': True, 'center_distance': 2.5,
        'entrance_distance': None,   # S2 시작 ~ 분기 입구 거리(m). None이면 lane_change.request_entrance_distance (현장 실측값)
        'exit_by_distance': True,    # 입구 거리를 S2 주행 거리(lane_change.request_entrance_distance)로도 판단 (합성 시험은 끔: 장면 길이가 다름)
        'safety_horizon': 0.3, 'safety_margin': 0.02,
    },
    # 차선 번호: 0=오른쪽, 1=가운데, 2=왼쪽 (진행 방향 기준)
    'lane_change': {
        # 11/15 소민: trigger_distance 1.15 → 1.4 (가까워진 뒤 시작하면 조향 공간 없음 no_safe_steering, 다음 장애물 늦게 봄)
        'lane_count': 3, 'initial_lane': 1, 'lane_width': 0.52, 'trigger_distance': 1.4,
        # 보는 거리: 이동 시작 lookahead(표지판 이동 request_lookahead) → 도착할수록 lookahead_end까지 늘림 (11/15 소민)
        'lookahead_end': 0.70,
        'lookahead': 0.40, 'request_lookahead': 0.40, 'speed': 0.18, 'margin': 0.035, 'safety_horizon': 0.3,
        # 11/14: 0.025·0.035 → 0.05·0.15. lap3에서 세 번 모두 조건 미달로 max_distance(3 m, 16 s)까지 끌림
        # (목표 도달 뒤 방향 추정 오차 약 5°, 횡 오차 0.04). 남은 오차는 차선 주행이 처리
        'finish_error': 0.05, 'finish_heading': 0.15,
        # 좌·우 선택은 최근 choice_window(s) 동안 옆 차로 빈 거리의 최솟값으로 (11/14 lap3: 의자 다리 한 스캔 누락 → 1.42 → 3.0 m로 튀어 막힌 쪽 선택)
        'choice_window': 0.5,
        'max_distance': 3.0,         # 한 차로 이동 최대 주행 거리(m). 넘으면 차선 주행으로 복귀
        # 좌·우 선택 (라이다): 옆 차로가 min_free_distance 이상, 지금 차로보다 free_gain 이상 더 비어 있어야 이동.
        # 둘 다 되면 더 멀리 비어 있는 쪽. 점이 없으면 free_range로 봄 (= lidar planning_point_range)
        # 11/15 소민: 1.0/0.3 → 0.8/0.1 (지그재그 장애물에서 양쪽 다 탈락해 정지). 0.8보다 낮추면 옮긴 직후 앞 장애물에 닿을 위험
        'min_free_distance': 0.8, 'free_gain': 0.1, 'free_range': 3.0,
        # 옆 차로가 차 바로 옆 물체(빈 거리 < beside_free m) 때문에만 막혔으면 앞 장애물에 stop_distance_beside(m, 뒷축)까지 접근
        # 일반 회피에서 실선을 막힘으로 볼지 (False: 3차선 차로 사이는 점선, 점선 오판 방지)
        'beside_free': 0.3, 'stop_distance_beside': 0.6, 'solid_blocks_avoid': False,
        # 짧은 S자 (11/16 소민): 이동 시작 때 앞 장애물이 short_front(m, 뒷축)보다 가까우면 원호–직선–원호를 short_arc_angle(°)로
        # 크게 틀어 짧게 옮김 (보통 arc_angle). 속도 short_speed. 끄기: short_front 0
        # short_arc_margin: 짧은 S자 복귀 여유(m, 보통 회피는 arc_margin). 큰 각도일수록 실차가 더 밀려 일찍 되돌림
        # short_flip_seconds(s) 동안 short_flip_speed: 시작 꺾기·복귀 반대 꺾기 순간 감속 (바퀴가 넘어가는 동안 덜 감)
        # 11/17 혁준: short_arc_margin 0.06 → 0.10 → 0.08. 실차 도착 오차: 0.06은 0.10~0.13 m 지나침(2회, back_1117_0249·
        # rev_1117_0212), 0.10은 0.05~0.13 m 못 미침(4회, bend·tight_0405·hard_0409: 못 미친 채 시작한 두 칸 요청 이동이
        # 오른쪽 노란 선을 넘음). 보통 이동(arc_margin 0.08)은 ±0.04 m 안(8회) → 같은 0.08로 맞춤
        'short_front': 1.0, 'short_arc_angle': 45.0, 'short_speed': 0.10, 'short_arc_margin': 0.08,
        'short_flip_seconds': 0.3, 'short_flip_speed': 0.05,
        # 11/16 혁준(실차 전): 표지판 요청 이동은 옆 차로 점을 뒷축 앞 request_side_ahead(m)부터만 막힘으로 보고, 짧은 S자로 옮김
        # 끄기: request_side_ahead None, short_on_request False
        # short_on_request: 11/16 sominb_1116_1753에서 한 칸 1.00 m(보통 1.06 m)·0.10 m/s라 10.7 s → 거리 단축 거의 없어 끔
        'request_side_ahead': 0.45, 'short_on_request': False,   # 11/17: 0.2 → 0.45 (출발 전 경로 확인이 실제 여유를 봄)
        # 표지판 요청 이동 경로 계획 (lane_change.plan_request, 11/16 혁준, 실차 전). 끄기: request_plan False
        # request_angles(°): 후보 원호 각도. request_plan_margin: 경로와 라이다 점 사이 최소 여유(m)
        # request_plan_stretch: 실차 이동이 계산보다 긴 비율 (한 칸 계산 1.01 m, 실측 1.06~1.14 m)
        # request_plan_tail: 도착 뒤 목적 차로 앞이 비어 있어야 하는 거리(m)
        # request_entrance_distance: 갈림길 입구(점선 끝)의 S2 주행 거리(m). 11/16 녹화에서 5.4~5.6 m부터 차선 변경 꺼짐 → 여유 두고 5.2
        # 각도 비교(합성, sessions/tools/lc_random_eval.py, 통과 가능한 임의 배치 7대 × 50): 한 칸씩 39 / 36° 41 / 45° 43(충돌·이탈 0) / 54° 43(충돌 1)
        # / 30~54° 중 선택 42(충돌 3: 가파르면 접촉, 완만하면 두 칸에서 바깥선 넘음) / 40·45·50° 43(이탈 1) → 45° 하나로
        'request_plan': True, 'request_angles': [45.0],
        'request_plan_margin': 0.04, 'request_plan_stretch': 1.1, 'request_plan_tail': 0.3,
        'request_entrance_distance': 5.2,
        # 출발 지연 (유경 11/16, 합성 시험만·실차 전. lane_change.plan_request): 두 칸 경로가 지금 막혀 있어도
        # request_delay_max(m)까지 request_delay_step(m)씩 더 직진한 뒤 꺾는 경로가 열리면 한 칸으로 내려가지 않고 기다림
        # 끄기: request_delay_max 0 (11/16 팀장 동작 그대로)
        'request_delay_max': 0.6, 'request_delay_step': 0.1,
        # 라이다 기준점 (11/16 혁준, 유경 feat/fork-double-move의 기준점을 옮김, 합성 시험만·실차 전). 끄기: anchor_enabled False
        # 가운데 차로 앞 통로에서 크기 anchor_width(m) 이하인 물체 = 섬 위 표지판 기둥. anchor_memory(m)까지는 못 봐도 거리 유지
        # 쓰는 곳: ① 요청 이동이 끝나야 하는 남은 거리(driver.request_limit, 기둥 거리 + anchor_limit_offset)
        #         ② 기둥이 지금 차로 바로 앞이면(가운데 차로 확실) 요청 방향은 옆 선이 노랑으로 읽혀도 넘음
        #         ③ 늦은 지점에서 목적 쪽이 끝내 막혀 있고 기둥이 anchor_escape(m) 안이면 반대쪽으로라도 피함 (기둥 앞 정지 방지)
        # anchor_limit_offset: 이동이 기둥 옆을 이만큼(m) 지나서 끝나도 됨 (끝 구간은 이미 목적 차로 쪽, 기둥과의 간격은 경로 확인이 봄)
        'anchor_enabled': True, 'anchor_width': 0.15, 'anchor_memory': 0.5, 'anchor_limit_offset': 0.4, 'anchor_escape': 1.0,
        # request_late: 늦은 지점(점선 끝이 보임)에서도 경로가 확인되면 목적 쪽 이동 허용. 끄기 False (예전: 새 이동 금지)
        'request_late': True,
        # request_extend: 진행 중인 한 칸 이동과 같은 방향으로 목적 차로가 두 칸이면(이동 시작 뒤 표지판 확정) 경로 확인 뒤 두 칸으로 넓힘 (11/17)
        'request_extend': True,
        # request_reverse: 진행 중인 이동의 도착 차로가 목적 차로 반대쪽이면(이동 시작 뒤 표지판 확정) 경로가 비는 순간 목적 차로 쪽으로 되돌림 (11/17)
        #   request_reverse_tail: 되돌아간 차로가 도착 뒤 비어 있어야 하는 거리(m). 방금 피한 장애물 차로로 돌아가 회피를 반복하지 않게
        #   trigger_distance(1.4, 뒷축) − 뒷축~차체 앞(0.26) 정도
        # request_blocked_escape: 요청 방향은 열렸는데 경로만 막히고 기둥이 anchor_escape 안이면 일반 회피로 넘김 (11/17)
        # 11/17 05:50 정리: 실차 근거가 없는 것은 끔 (오늘 추가한 규칙끼리 충돌 사례 5건, 사용자 결정). 코드는 유지, 현장에서 하나씩 비교
        #   끔: request_blocked_escape·safe_flip_block·back_end·lane_memory·backup
        #   켬: request_extend(실차 2회 성공)·short_arc_margin 0.10·entry_align·heading_bias_min_distance·fork_sign_bearing·화살표 최소 면적
        # 11/18 오후 request_reverse 켬 (사용자 지시, redecide_1118_20): 왼쪽 끝 차로에서 오른쪽으로 피하기 시작한 직후(옆 이동 0.05 m)
        #   left 확정 → 이동을 끝까지 마친 뒤(S2 4.97 m)에야 돌아오려다 late_request_blocked → B로 진입.
        #   녹화 재생: 켜면 확정 1.2 s 뒤(옆 이동 0.13 m, 경로 여유 0.04) 목적 차로로 되돌림. 합성 27건 켬·끔 모두 27/27. 실차 미확인
        # 11/18 저녁 다시 끔 (사용자 지시, redecide_1118_33): 반대쪽 45°를 본 자세에서 되돌리면 방향만 펴고 끝남(arc_steering 복귀 판정이
        #   차 방향 부호를 안 봄) → 차로 번호만 바뀐 채 가운데 차로에 남아 기둥을 반대쪽으로 피함, right인데 A 진입.
        #   같은 상황(redecide_1118_19·20)은 소민 줄이기(request_shrink, 아래)로 대신함. 줄이기는 되돌리기와 같이 켜면 되돌리기로 넘어감
        'request_reverse': False, 'request_reverse_tail': 1.1, 'request_blocked_escape': False,
        # safe_flip_block: 이동 시작 직후(safe_flip_travel m 안) 안전 조향이 목표 반대쪽이고 차이가 safe_flip_max(rad) 넘으면 따르지 않고 정지 (11/17)
        'safe_flip_block': False, 'safe_flip_max': 0.4, 'safe_flip_travel': 0.15, 'safe_flip_offset': 0.08,
        'request_extend_slack': 0.5,   # 넓히기: 기둥을 못 본 채 어림한 한계는 이만큼(m) 넘어도 허용
        # back_end: 복귀 원호 중 횡이동이 목표에서 back_end_error(m) 안까지 왔는데 안전 조향이 원하는 조향보다 이동 방향(바깥)으로
        #   back_end_deviation(rad) 넘게 벗어나면 이동 종료 (그대로 따르면 바깥으로 밀려남, 11/17)
        # lane_memory: 이동·후진이 끝난 뒤 lane_memory_seconds(s) 안에 차선이 안 잡히면 '차로 중앙·IMU 방향'으로 보고 판단 유지 (11/17)
        'back_end': False, 'back_end_error': 0.02, 'back_end_deviation': 0.15,
        'lane_memory': False, 'lane_memory_seconds': 3.0,
        # 후진 재시도 (lane_change.BackUp, 11/17): 앞 장애물 backup_front(m, 뒷축) 안에서 backup_wait(s) 넘게 서 있으면
        # 지나온 경로를 backup_distance(m) 되짚어 후진한 뒤 다시 판단. 한 자리 backup_max번. 실차 값 확인 전. 끄기: backup False
        # 11/17 04:50 끔: 실차 2회 발동, 2회 모두 구조 실패 (back_1117_0249 차로 밖에서 발동, arrow_1117_0322 틀린 차로 번호 위에서
        # 발동해 바깥쪽 이동으로 이어짐). 규정도 회색(부칙 가·나). 코드는 남겨 둠
        'backup': False, 'backup_wait': 1.0, 'backup_front': 0.9, 'backup_distance': 0.3, 'backup_speed': 0.10,
        'backup_max': 2, 'backup_seconds': 6.0, 'backup_side': 0.15, 'backup_reset_distance': 1.0,
        # backup_align: 차로 안에 선 경우 도로 방향으로 정렬하며 후진 (끄면 항상 지나온 경로 되짚기). backup_known_offset: 차선 변경으로
        #   들어온 차로에서 '뒤가 빈 것을 아는 거리' = 그 이동 거리 − 이 값 (출발 때 옆 차로를 뒷축 앞 0.45 m부터 확인 + 차 뒤끝 0.07)
        'backup_align': True, 'backup_align_gain': 2.5, 'backup_known_offset': 0.52, 'backup_mid_travel': 0.15,
        # 방향 기준 (11/16 혁준, anchor_1116_2313 대응, 합성 시험만·실차 전)
        # imu_heading: 도로 방향 기준 차 방향을 구간 관리자의 IMU 누적 회전으로 (끄면 예전처럼 카메라·이어받기)
        # heading_disagree(rad): 카메라 차선 방향이 IMU 값과 이만큼 넘게 다르면 IMU 값 사용
        # start_align(rad): 이보다 더 틀어져 있으면 새 차선 변경 시작 안 함 (차선 주행으로 먼저 정렬)
        # camera_guard: 카메라 종료·보정은 방향이 camera_trust_heading 안이고 카메라와 camera_agree_heading 안에서 맞을 때만
        'imu_heading': True, 'heading_disagree': 0.2, 'start_align': 0.21,
        # 누적 회전 치우침 보정 (11/17 혁준, early_1117_0015 대응, driver.observe_heading_bias). 끄기: heading_bias False
        # 직진(카메라 방향 heading_bias_camera° 안, 조향 heading_bias_steering rad 안, 두 선 보임) 중 (누적 회전 − 카메라 방향)을
        # heading_bias_seconds(s) 시정수로 평균. 차이가 heading_bias_max(°) 넘으면 표본에서 뺌
        # heading_bias_min_distance: 구간 전환 뒤 이 거리(m) 안에서는 치우침을 배우지 않음 (코너 출구, 11/17)
        # entry_align: 3차선 진입 뒤 entry_align_distance(m) 안에서 구간 방향보다 왼쪽으로 entry_align_angle(°) 넘게 틀어져 있으면
        #   오른쪽 조향으로 코너를 마저 돎 (driver.entry_align, 11/17)
        'heading_bias_min_distance': 1.0,
        'entry_align': True, 'entry_align_distance': 1.5, 'entry_align_angle': 12.0, 'entry_align_gain': 1.5,
        'heading_bias': True, 'heading_bias_seconds': 2.0, 'heading_bias_camera': 12.0,
        'heading_bias_steering': 0.08, 'heading_bias_max': 25.0,
        'camera_guard': True, 'camera_trust_heading': 0.26, 'camera_agree_heading': 0.175,
        # 일반 회피 일찍 시작 (11/16 혁준, imu_1116_2348 대응, 합성 시험만·실차 전). 끄기: avoid_side_ahead None, avoid_plan False
        # avoid_side_ahead(m): 옆 차로 점은 뒷축 앞 이 거리보다 먼 것만 막힘 (예전: 뒷축보다 앞이면 전부)
        # avoid_plan: 출발 전에 실제 쓸 각도로 경로를 그려 라이다 점과 여유 avoid_plan_margin(m) 미만이면 아직 출발 안 함
        # avoid_plan_tail(m): 도착 뒤 목적 차로 앞이 비어 있어야 하는 거리
        # 0.45: 45° 원호로 꺾으면 앞 오른쪽 모서리가 옆 차로 장애물 선에 닿기까지 뒷축이 약 0.45 m 감. 합성(지그재그, 앞 장애물까지
        # 간격 0.58 m): 예전 기준·0.2는 앞 0.45 m에서 시작해 충돌·정지, 0.45는 앞 0.71 m에서 시작해 통과
        'avoid_side_ahead': 0.45, 'avoid_plan': True, 'avoid_plan_margin': 0.04, 'avoid_plan_tail': 0.2,
        # 3차선 진입 흰 선 전 차선 변경 금지 (11/18 혁준, driver.entry_gate_open). 끄기: entry_gate False
        # entry_line_start: 앞축이 흰 선 이 거리(m) 전에 오면 허용 (차선 변경은 시작 뒤 약 0.4 m를 가야 차로 가장자리를 넘음)
        # entry_min_distance: 흰 선을 못 봤을 때 S2 주행 거리 기준(m). entry_max_distance: 이 거리부터는 무조건 허용
        'entry_gate': True, 'entry_line_start': 0.30, 'entry_line_range': 1.3,
        'entry_min_distance': 0.25, 'entry_max_distance': 1.0,
        # 이동 중 재판단 (11/18 혁준, lane_change.redecide, 합성 시험만·실차 전). 끄기: redecide False
        # 일반 회피로 한 칸 옮기는 중 도착할 차로 앞 빈 거리가 redecide_front(m) 미만이면 다음 차로까지 그대로 가로지름
        # 출발 판단에서도 옆 차로 한 칸이 안 되면 두 칸 직선 가로지르기(lane_change.cross_two, 각도 redecide_angles °)를 봄
        # 11/18 통합 시험 브랜치(test/3lane-parking-1118): 실차 시험용으로 켬. main 반영 때는 실차 결과 보고 결정
        # 11/19 다시 켬 (사용자 지시): 끈 상태로는 점선 추종이 넘겨준 뒤 기존 방식이 경기장 배치(합성 venue 6건)에서 75 s 멈춤,
        #   켜면 5/6 통과. 소민 test/dashed-line-avoidance(점선 쪽 당김·갈림길 우회)도 켬 전제
        'redecide': True, 'redecide_front': 1.0, 'redecide_angles': [36.0, 45.0],
        # redecide_tail (11/18, track_test_1118_01~05): cross_two arrival must leave this much free distance ahead (m).
        # 0.2 -> 0.6: all five runs arrived 0.37 m behind an obstacle, then a 0.10 m/s short S (11~16 s).
        # synthetic 100 layouts: 0.2/0.6/1.0 identical (67/100). real car unverified. revert: remove key (falls back to avoid_plan_tail)
        'redecide_tail': 0.6,
        # 두 칸 → 한 칸 줄이기 (11/18 소민, lane_change.try_shrink, 녹화 재현·합성 시험만·실차 전). 끄기: request_shrink False
        # 두 칸 이동 중 표지판이 확정됐는데 첫째(가운데) 차로가 목적 차로에 더 가까우면, 복귀 원호를 일찍 시작해 첫째 차로에서 끝냄
        # 조건: 복귀 원호 전 + 첫째 차로 중앙까지 남은 횡거리 > 되돌리는 원호 횡거리 + shrink_slack(m) + 첫째 차로 쪽 경로가 빔
        # redecide_1118_20: 확정 때 횡이동 0.05 m (45°면 남은 횡거리 약 0.26 m까지 줄일 수 있음) → 줄였으면 19처럼 2 → 1 → 2
        'request_shrink': True, 'shrink_slack': 0.0,
        # redecide_fork_guard (11/18 소민, run_1118_1627 대응): 표지판 확정 뒤 재판단이 목적 차로에서 더 멀어지는 쪽으로 두 칸 넓히는 것 막음
        'redecide_fork_guard': True,
        # 가운데로 한 칸 피할 때 넘어온 흰 점선 쪽으로 목표를 당기는 거리(m). 끄기: 0 (11/19 소민 test/dashed-line-avoidance)
        'intermediate_dash_bias': 0.05,
        # 흔들림 방지 (11/16 소민): 회피 판단 중 앞이 front_clear_hold(s) 동안 계속 비어야 판단을 놓음.
        # no_side_wait: 갈림길 확정 전 양쪽 '차로 없음'이어도 차선 주행으로 넘기지 않고 천천히 다가가며 다시 판단
        'front_clear_hold': 0.5, 'no_side_wait': True,
        # 갈 차로가 없을 때: 앞 장애물까지 stop_distance(m, 뒷축 기준) 넘게 남았으면 approach_speed로 지금 차로 계속 주행,
        # 그 안이면 정지. 0.6 = 한 차로 비켜 가는 데 필요한 최소 앞 거리 근사 (R 0.45, 횡 0.25 m). 실차 조정 필요
        'stop_distance': 0.6, 'approach_speed': 0.15,
        # 선 색으로 현재 차로 확인: 노랑/흰 → 2, 흰/흰 → 1, 흰/노랑 → 0. 새 카메라 프레임 이 수만큼 연속 같으면 index 보정
        'index_confirm_frames': 5,
        # 차가 도로 방향에서 이만큼(rad) 넘게 틀어져 있으면 선 색으로 차로를 판정하지 않음 (11/15 main_1115_1904 178~179 s:
        # 0→1 이동 중 왼쪽으로 약 30° 틀어진 채 두 차로 건너 바깥 노란 선 + 흰 점선을 '노랑/흰 = 2번 차로'로 봄)
        'index_heading_max': 0.3,
        # 이동 시작 방향: 직전 이동이 chain_seconds(s) 안에 끝났으면 적분해 온 방향을 이어 씀, 아니면 최근 heading_frames 프레임 중앙값
        'chain_seconds': 2.0, 'heading_frames': 5,
        # 이동 중 조향 (11/15): 'arc' = 원호–직선–원호 (lane_change.arc_steering), 'pursuit' = 예전 목표점 추종(lookahead)
        # arc_angle: 도로 방향에서 틀 최대 각(°). arc_gain: 방향 오차 → 조향 배율. arc_margin: 복귀 시작 여유(m, 조향 지연)
        # arc_finish_heading: 복귀 뒤 이 각(rad) 안이면 종료. lateral_scale: 추측 항법 횡이동 배율 (실제가 더 감 → 1보다 큼)
        # 자전거 모델 계산(추측 항법 0.8배, 조향 지연 0.15 s): pursuit 1.35 m·최대 42°·끝 8.6°·차선 오차 0.21 m
        #   → arc + 1.2배 1.26 m·30°·2.8°·0.04 m. 두 값(arc_angle, lateral_scale)은 실차 조정 필요
        # 11/16 main_1116_fix1: 30°에서 직선 구간 2.5~3.1 s(실제 최대 33~38°)로 김 → 36 (계산상 직선 약 1.8 s, 최대 약 42°). 실차 34~38에서 조정
        # 11/17 bias_1117_0036: 방향 기준이 맞게 된 뒤 lateral_scale 1.2가 과보정(두 칸 추정 1.11 m·실제 약 0.88 m → 0.16 m 모자라 경계선 밟음)
        # → 1.0. arc_margin 0.03 → 0.08: 복귀 원호를 시작한 뒤 조향이 넘어가는 동안 계산(0.09 m)보다 약 0.05 m 더 감 (실차 전)
        'steer_mode': 'arc', 'arc_angle': 36.0, 'arc_gain': 2.5, 'arc_margin': 0.08, 'arc_finish_heading': 0.05,
        'lateral_scale': 1.0,
        # 끝 차로 안쪽 여유 (11/18 혁준, 사용자 지시, track_test_1118_01~05: 두 칸 이동이 바깥 노란 선 쪽 차로에서 끝날 때
        # 복귀 원호 중 앞 왼쪽 모서리가 노란 선까지 계산상 0.13~0.15 m). 끝 차로로 가는 일반 회피 이동은 조향 목표를
        # 차로 중앙에서 점선 쪽으로 이만큼(m) 당김 (lane_change.edge_inset). 표지판 요청·확정 뒤 이동은 제외. 끄기: 0
        # 합성 시험만·실차 전. 도착 뒤에는 차선 주행이 다시 차로 중앙으로 맞춤
        'edge_lane_inset': 0.10,   # 11/18 20시: 0.06 → 0.10 (사용자 지시, 시험 없이 반영)
        # 교통섬 노란 선 구분 (소민 11/15, lane_change.trusted_colors)
        # 표지판 확정 뒤: 출구 쪽 노란 선만 바깥선, 반대쪽 노란 선은 교통섬 (숫자 없음)
        # 표지판 전: 노란 선 가까운 끝이 yellow_near_max(m, 뒷축 앞 거리. BEV 맨 아래 0.6 m)보다 멀거나
        #   차선 방향과 yellow_heading_max(rad) 넘게 비스듬하면 교통섬
        'yellow_near_max': 0.9, 'yellow_heading_max': 0.25,
        # 표지판 확정 뒤 '가운데'(흰/흰) 판정으로 차로 번호를 고치려면 이 프레임 수 연속 필요 (평소 index_confirm_frames)
        'index_confirm_frames_fork': 20,
        # 소민 11/12 (three_lane 합성 시험 기준, 실차 확인 전)
        # 카메라 종료: 한 차로 폭 × progress 넘게 옮겼고 선 색이 목적 차로 + 중앙 오차(m)·방향(rad)이 이 안이면 종료
        # camera_finish_center (11/17): 중앙 판정을 원 차선 오차 대신 방향을 보정한 차로 중앙값으로 (틀어진 채 일찍 끝나는 것 방지)
        #   기본 끔: 실차 cfc_1117_0428 한 번 시험, 그 주행에서는 판정이 갈리는 상황이 안 생겨 효과 확인 못 함 (사용자 결정: 효과 없으면 되돌림)
        #   근거 사례는 m08_1117_0417 31.7 s (오차 −0.03인데 실제 중앙에서 0.10 m 못 미친 채 종료)
        'camera_finish_progress': 0.6, 'camera_finish_error': 0.06, 'camera_finish_heading': 0.12,
        'camera_finish_center': False,
        'overshoot': 0.08,   # 목표 횡이동보다 이만큼(m) 더 가면 바로 종료 (바깥 노란 선 넘음 방지)
        # 11/12: 이동 중 목적 차로 색이 camera_fix_frames번 연속이고 그 중앙이 차 기준 lane_width×range 안이면
        # 추측 항법 위치·방향을 카메라 값으로 교체 (odom·IMU 오차 누적 → 두 번 연속 변경 뒤 바깥선 이탈 대책)
        'camera_fix_frames': 2, 'camera_fix_range': 0.5,
        # 11/13 대회장 벽(노란 바깥선 위) 라이다 점 제거 (lane_change.drop_walls). 바깥선 ±wall_search(m) 안에서
        # 앞뒤 wall_min_length(m) 넘게 이어진 직선(점 wall_min_points개 이상, 기울기 wall_max_slope 이하) = 벽
        # → 그 선에서 wall_band(m) 안쪽 ~ 바깥 점 버림. 벽 없는 곳에서는 직선이 안 잡혀 아무것도 안 버림
        'wall_filter': True, 'wall_search': 0.25, 'wall_min_length': 0.8, 'wall_min_points': 12,
        'wall_max_slope': 0.35, 'wall_band': 0.05,
        # 11/14: 맞춘 선이 바깥선보다 이만큼(m) 넘게 안쪽이면 벽 아님 (바깥선 쪽으로 붙은 장애물 차 줄 오인 방지)
        'wall_inside': 0.08,
        # 옆 차로 '차 옆·바로 뒤' 검사 범위: 차체 뒤끝(뒷축 뒤 7.2 cm) + 이만큼(m). 예전 차 길이(33 cm)
        # 11/16 소민: 0.05 → -0.072 = 뒷축보다 앞에 있는 점만 '옆 막힘' (좌회전 때 뒤 오버행은 반대쪽으로 빠짐).
        # 지나친 장애물 옆에서 바로 돌아오도록 (run_1115_2152: 옆 장애물이 차 뒤끝까지 빠지길 기다리다 앞 장애물에 너무 붙음)
        'side_behind_margin': -0.072,
    },
}

# 주차 A/B 시험 (11/19 추추). 실행: driver.launch.py parking_profile:=a|b (기본 a)
#   a: 지금 값 = 83b0474(3283a8d 되돌림) 기준. 시작선→줄 0.45, LiDAR 진행 방향 보정 ±0.20
#   b: 3283a8d의 두 숫자만 다시 켬 (0.54, ±0.05). 목표 고정·첫 칸 밀기 유지 로직은 되돌림으로 빠져 b에도 없음
#   b는 회전을 약 2~7 cm 일찍 시작. 같은 칸에서 번갈아 달리고 /debug의 parking.profile·reason을 비교
PARKING_PROFILES = {
    'a': {},
    'b': {'template_start_to_row': .54, 'template_fusion_max_offset': .05},
}


def apply_parking_profile(parking, name):
    """parking 설정 dict에 A/B 값을 덮어씀. 알 수 없는 이름이면 시작 시 종료"""
    name = (name or 'a').lower()
    if name not in PARKING_PROFILES:
        raise SystemExit(f'parking_profile "{name}"는 {list(PARKING_PROFILES)} 중 하나여야 합니다')
    parking.update(PARKING_PROFILES[name])
    parking['profile'] = name
    return parking


YOLO_CLASSES = ['traffic_red', 'traffic_yellow', 'traffic_green', 'left', 'right', 'pedestrian', 'slow', 'parking']

