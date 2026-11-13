"""주행 파라미터. 담당별 구역만 수정. 거리 m, 속도 m/s, 조향 rad(+좌회전)"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)

ANSWER_CONFIG = {
    'rate_hz': 20.0, 'debug_rate_hz': 5.0, 'sensor_timeout': 0.7, 'odometry_timeout': 0.7,
    'base_speed': 0.4, 'slow_speed': 0.2, 'minimum_curve_speed': 0.15,
    'corner_speed': 0.2,   # 코너 속도 상한. 도면 코너 시뮬: 0.4 진입은 바깥 접촉
    'corner_slow_curvature': [2.0, 3.5],   # 선 곡률(1/m) 이 구간에서 base_speed → corner_speed 보간 (lane_driving)
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
                    'steering': 0.38, 'gentle_steering': 0.31, 'sharp_curvature': 2.0,
                    # 유지 중 조향 = 시작 값 + error_gain × (시작 오차 − 현재 오차), steering_min~max_steering 제한
                    'error_gain': 0.4, 'steering_min': 0.2,
                    # 시작 순간 가까운 선이 왼쪽으로 skip_near_curvature(1/m) 넘게 휘면 이번 코너는 유지 없이 pursuit (비스듬한 진입)
                    'skip_near_curvature': 0.1, 'skip_window': 0.6,
                    'angle': 65.0, 'max_distance': 1.3,
                    'start_far_angle': 20.0,    # 선 먼 끝이 오른쪽으로 이 각도(°) 넘을 때만 시작 (갈림길 S자 꺾임 제외)
                    # 유지 중·끝난 뒤 after_seconds 동안 기억 없는 한 선은 가까운 끝이 오른쪽 corner_left_limit(m) 안이면 왼쪽 선
                    'after_seconds': 1.5, 'corner_left_limit': 0.1},

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
                 'hold_distance': 3.0, 'off': []},
    # 멈춤 방지 (driver.stall_guard): 의도하지 않은 정지가 seconds 넘으면 강제 전환 (규정: 15초 넘게 서면 그 미션 0점)
    # 차선 변경 대기 → lane_change_off_distance(m) 동안 차선 변경 끄고 차선 주행
    # 전방 비상 정지 → 라이다 점에서 creep_margin(m) 떨어진 조향이 creep_horizon(m) 앞까지 있으면 creep_speed로 비켜 감
    # 값은 실차 확인 전
    'stall': {'seconds': 8.0, 'reset_distance': 0.3, 'lane_change_off_distance': 1.5,
              'creep_speed': 0.08, 'creep_horizon': 0.3, 'creep_margin': 0.02},

    'detector': {
        'enabled': True, 'confidence': 0.45, 'image_size': 320, 'device': 'cpu', 'rate_hz': 5.0,
        'timeout': 1.5, 'minimum_area': 0.0006, 'confirm_frames': 2,
        # 작은 화살표부터 준비. 면적 0.15%·서로 다른 3프레임 확인 (실영상 조정 필요)
        'confirm_frames_by_class': {'left': 3, 'right': 3},
        'minimum_area_by_class': {
            'parking': 0.002, 'pedestrian': 0.012, 'traffic_red': 0.025, 'traffic_yellow': 0.025,
            'traffic_green': 0.025, 'left': 0.0015, 'right': 0.0015, 'slow': 0.004,
        },
        'cpu_threads': 2, 'edge_margin': 0.01,
        'allow_truncated_classes': ['traffic_red', 'traffic_yellow', 'traffic_green'],
        'shutdown_timeout': 5.0,
        # 시작 직후 첫 검출을 기다리는 최대 시간(s). 그 뒤로는 검출이 끊겨도(timeout 초과·예외) 표지판 미션만 끄고 주행 계속
        'startup_wait': 20.0,
        'failure_limit': 5,   # YOLO 예외가 이 횟수 연속이면 고장으로 보고 시작 대기도 건너뜀 (추론은 계속 재시도)
    },
    'lane': {
        # 차선 = 노란색, 정지선·횡단보도·노면 표시 = 흰색 (11/8 현수막 확인)
        # 수동 노출 300 기준 노란 선 H 19~39, S 중앙 138, V 중앙 68 → 어두워도 색상으로 구분. 트랙 바닥 H 110대
        'yellow_min': [18, 40, 50], 'yellow_max': [40, 255, 255],
        # 정지선·횡단보도 = 흰색. 정지선 검출에만 사용 (차선은 노랑만). 예전 범위로 시작, 노출 300 기준 실측 필요
        # 11/9 통합 시험: V 165 → 110. 3차선 점선 V 135~150 (녹화 white_1109_lap 138~152 s), 정지선 247 s도 V 165에서 누락
        # 부작용: 코너 바깥 현수막 흰 테두리를 정지선으로 잡음 (83·184 s). 소민 확인 필요
        'white_min': [0, 0, 110], 'white_max': [179, 85, 255],
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
        'cone_max_distance': 2.5,    # 안전장치: 첫 진입 시 문까지 거리 + 이 값 넘게 주행하면 끝 (다시 안 켜짐). 라바콘 구간 약 2.1 m + 여유
        # 카메라 붉은색 (11/12 유경): 주최 측 안내 = 콘은 빨강에 가까운 붉은색. 11/12 cone_lane_3 진입 못 하고 차선 복귀·충돌
        # 붉은색이 보이면 진입·통과 조건 완화: 문 간격 최대 cone_red_gate_max, 좌우 cone_red_half_width, 거리 cone_red_start_range
        # 통과 중 붉은색이 보이면 콘이 안 보여도 모드 유지 (cone_max_distance 안전장치는 그대로)
        # 판정: 화면 아래 cone_red_rows 비율 영역을 1/4로 줄여 HSV, 색상 0~h_low 또는 h_high~180, 채도·밝기 하한, 화소 비율 cone_red_ratio 이상
        # 11/12 실차 red_check (노출 300, 콘 약 1 m): 콘 H 0~9·170~179, S 약 190, V 약 130 → 색 범위 맞음.
        # 노란 선(H 20~39, V 40 전후)·흰 선·검은 바닥은 판정 0. 카메라가 낮아 콘 몸통은 대부분 화면 가운데보다 위
        # (아래 50%만 볼 때 0.040, 위쪽만 0.153) → 판정 영역 아래 75%로 넓힘. 먼 콘(약 2 m)은 아래 50%에 안 들어옴
        'cone_red_h_low': 10, 'cone_red_h_high': 165, 'cone_red_s_min': 100, 'cone_red_v_min': 60,
        # 11/12 유경: 판정 영역 화면 전체로 (콘 바로 앞 위치는 사진 확인 못 함). 마지막으로 보인 뒤 cone_red_memory(m)까지 유지
        'cone_red_rows': 1.0, 'cone_red_ratio': 0.03, 'cone_red_memory': 0.5,
        # 11/13 실차 사진 18장(red_1113_000127, 코너 앞 → 콘 통로): 관중석 붉은 의자만 보여도 전체 0.010~0.015
        # → 기준 0.002에서 0.03으로 올림 (콘 1개가 화면 끝에 보일 때 0.07).
        # 진입은 화면 전체 붉은 비율 cone_red_enter_ratio 이상일 때만 (11/13 유경 결정). 같은 사진: 의자 0.01, 콘 1개 0.07,
        # 코너 중 콘 줄 0.17~0.27, 통로 정면 0.30, 통로 안 0.39~0.45. 0.27(회전 중 한쪽 줄)은 진입함
        'cone_red_enter_ratio': 0.2,
        'cone_red_side_width': 0.2, 'cone_red_side_top': 0.4,   # 양쪽 끝 띠 비율: 확인용 출력만 (판단에 안 씀)
        'cone_red_gate_max': 1.6, 'cone_red_half_width': 0.9,
        # 진입 시점: 라바콘 모드로 바꾸는 라이다 앞 방향 거리(m). 붉은색이 보이면 콘 후보 하나, 아니면 문 중점 기준.
        # 11/12 실차(유경): 1.2 m에서 켜면 코너 유지(고정 조향) 중에 바로 꺼져 차선 이탈 → 콘 바로 앞까지 차선 주행·코너 유지.
        # 같은 날 문 중점 직선거리 0.3 m는 안 켜짐 (차로 중앙과 문 중점이 옆으로 어긋남) → 붉은색 + 콘 앞 방향 거리로 변경
        'cone_enter_distance': 0.3,
    },
    'missions': {
        'fork_prepare_speed': 0.18,  # S2 방향 확정 전·분기 유지 중 상한. 늦은 진입 완화
        'sign_cooldown': 12.0, 'slow_seconds': 14.0, 'fork_bias': 0.55, 'pedestrian_wait': 3.2,
        'stop_offset': 0.015, 'stop_tolerance': 0.025, 'stop_approach_speed': 0.13,
        'traffic_require_stop': True, 'traffic_min_stop': 0.3, 'stop_gain': 0.9, 'stopped_speed': 0.012,
        'approach_timeout': 15.0, 'stop_line_memory': 8.0,
        'line_match_tolerance': 0.1,
        # 미션 순서는 구간 관리자가 결정 (위 sections, section.py SECTION_MISSIONS). 예전 sequence·enforce_order 제거
        'traffic_max_wait': 60.0,   # 신호등 정지 후 녹색을 못 본 채 이 시간(s) 넘으면 포기하고 진행. 신호 주기 확인 후 조정 (소민)
        # 목표 끝 차로 도착 후 출구 추종 거리(m). 정렬·회피 중에는 차감 없음
        'fork_distance': 3.0,
        # 분기 입구 = 흰 점선 끝 (소민 11/12, 실차 전). 점선 사이 빈칸 최대 길이(m): 점선 끝이 화면 먼 끝(far)보다
        # 이만큼 넘게 가까우면 '늦음' → 새 차선 변경 금지. 점선이 끝나고 entrance_confirm(m) 더 가면 입구 도착
        # 11/12: 0.3 → 0.45 (빈칸 + 화면 끝 잘린 조각 최소 길이 0.08 여유. 실제 점선 빈칸 실측 후 조정)
        'fork_dash_gap': 0.45, 'fork_entrance_confirm': 0.3,
        'pedestrian_require_line': True,
        'trigger_line_max_distance': 1.5, 'trigger_line_freshness': 0.4,
    },
    'parking': {
        'enabled': True, 'test_wait_for_sign': False, 'test_drive_until_sign': False,
        # 신호등을 끝내지 않았으면 S4 진입 뒤 이 거리(m)부터 주차 표지판 수용. 도면 추정(횡단보도 끝 약 3.5 m, 주차 시작선 약 5.1 m) → 실차 조정
        'after_traffic_distance': 4.0,
        'command_deadline': .65, 'scan_speed': .12, 'turn_speed': .12, 'turn_steering': -.35,
        'search_timeout': 45., 'search_distance_limit': 5., 'turn_timeout': 8., 'turn_distance_limit': .85,
        'sensor_timeout': .7, 'motion_timeout': 4., 'body_margin': .015,
        'slot_depth': .59, 'view_near': .30,
        'gap_view_far': 3., 'gap_view_right': 1.5, 'gap_cluster_distance': .12,
        'gap_cluster_min_points': 3, 'gap_object_min_width': .08, 'gap_object_max_width': 1.,
        'gap_edge_padding': .025, 'gap_front_padding': 0., 'gap_row_tolerance': .18,
        'gap_extra_width': .04, 'gap_max_width': .85,
        'occupancy_edge_margin': .015, 'occupancy_min_hits': 3, 'free_ray_fraction': .65,
        'white_roi_top': .50, 'white_roi_side_margin': .02, 'white_hough_votes': 20,
        'white_min_length_fraction': .12, 'white_min_vertical_fraction': .12,
        'white_loss_frames': 1, 'white_acquire_timeout': 1.5,
    },
    'max_image_travel': 0.04, 'curve_speed_gain': 0.6,
    # 차선 번호: 0=오른쪽, 1=가운데, 2=왼쪽 (진행 방향 기준)
    'lane_change': {
        'lane_count': 3, 'initial_lane': 1, 'lane_width': 0.52, 'trigger_distance': 1.15,
        'lookahead': 0.55, 'speed': 0.18, 'margin': 0.035, 'safety_horizon': 0.3,
        'finish_error': 0.025, 'finish_heading': 0.035,
        'max_distance': 3.0,         # 한 차로 이동 최대 주행 거리(m). 넘으면 차선 주행으로 복귀
        # 좌·우 선택 (라이다): 옆 차로가 min_free_distance 이상, 지금 차로보다 free_gain 이상 더 비어 있어야 이동.
        # 둘 다 되면 더 멀리 비어 있는 쪽. 점이 없으면 free_range로 봄 (= lidar planning_point_range)
        'min_free_distance': 1.0, 'free_gain': 0.3, 'free_range': 3.0,
        # 갈 차로가 없을 때: 앞 장애물까지 stop_distance(m, 뒷축 기준) 넘게 남았으면 approach_speed로 지금 차로 계속 주행,
        # 그 안이면 정지. 0.6 = 한 차로 비켜 가는 데 필요한 최소 앞 거리 근사 (R 0.45, 횡 0.25 m). 실차 조정 필요
        'stop_distance': 0.6, 'approach_speed': 0.15,
        # 선 색으로 현재 차로 확인: 노랑/흰 → 2, 흰/흰 → 1, 흰/노랑 → 0. 새 카메라 프레임 이 수만큼 연속 같으면 index 보정
        'index_confirm_frames': 5,
        # 소민 11/12 (three_lane 합성 시험 기준, 실차 확인 전)
        # 카메라 종료: 한 차로 폭 × progress 넘게 옮겼고 선 색이 목적 차로 + 중앙 오차(m)·방향(rad)이 이 안이면 종료
        'camera_finish_progress': 0.6, 'camera_finish_error': 0.06, 'camera_finish_heading': 0.12,
        'overshoot': 0.08,   # 목표 횡이동보다 이만큼(m) 더 가면 바로 종료 (바깥 노란 선 넘음 방지)
        # 11/12: 이동 중 목적 차로 색이 camera_fix_frames번 연속이고 그 중앙이 차 기준 lane_width×range 안이면
        # 추측 항법 위치·방향을 카메라 값으로 교체 (odom·IMU 오차 누적 → 두 번 연속 변경 뒤 바깥선 이탈 대책)
        'camera_fix_frames': 2, 'camera_fix_range': 0.5,
        # 11/13 대회장 벽(노란 바깥선 위) 라이다 점 제거 (lane_change.drop_walls). 바깥선 ±wall_search(m) 안에서
        # 앞뒤 wall_min_length(m) 넘게 이어진 직선(점 wall_min_points개 이상, 기울기 wall_max_slope 이하) = 벽
        # → 그 선에서 wall_band(m) 안쪽 ~ 바깥 점 버림. 벽 없는 곳에서는 직선이 안 잡혀 아무것도 안 버림
        'wall_filter': True, 'wall_search': 0.25, 'wall_min_length': 0.8, 'wall_min_points': 12,
        'wall_max_slope': 0.35, 'wall_band': 0.05,
        # 옆 차로 '차 옆·바로 뒤' 검사 범위: 차체 뒤끝(뒷축 뒤 7.2 cm) + 이만큼(m). 예전 차 길이(33 cm)
        'side_behind_margin': 0.05,
    },
}

YOLO_CLASSES = ['traffic_red', 'traffic_yellow', 'traffic_green', 'left', 'right', 'pedestrian', 'slow', 'parking']

