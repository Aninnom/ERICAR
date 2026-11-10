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
    'corner_hold': {'enabled': True, 'arm_seconds': 3.0, 'start_steering': 0.15,
                    # 시작 시 곡률 sharp_curvature 이상이면 steering, 아니면 gentle_steering (11/9 lap3, 1번 코너 0.31이면 바깥 이탈)
                    'steering': 0.38, 'gentle_steering': 0.31, 'sharp_curvature': 2.0,
                    # 유지 중 조향 = 시작 값 + error_gain × (시작 오차 − 현재 오차), steering_min~max_steering 제한
                    'error_gain': 0.4, 'steering_min': 0.2,
                    'angle': 65.0, 'max_distance': 1.3,
                    'start_far_angle': 20.0,    # 선 먼 끝이 오른쪽으로 이 각도(°) 넘을 때만 시작 (갈림길 S자 꺾임 제외)
                    # 유지 중·끝난 뒤 after_seconds 동안 기억 없는 한 선은 가까운 끝이 오른쪽 corner_left_limit(m) 안이면 왼쪽 선
                    'after_seconds': 1.5, 'corner_left_limit': 0.1},

    'detector': {
        'enabled': True, 'confidence': 0.45, 'image_size': 320, 'device': 'cpu', 'rate_hz': 5.0,
        'timeout': 1.5, 'minimum_area': 0.0006, 'confirm_frames': 2,
        'minimum_area_by_class': {
            'parking': 0.02, 'pedestrian': 0.012, 'traffic_red': 0.025, 'traffic_yellow': 0.025,
            'traffic_green': 0.025, 'left': 0.006, 'right': 0.006, 'slow': 0.004,
        },
        'cpu_threads': 2, 'edge_margin': 0.01,
        'allow_truncated_classes': ['traffic_red', 'traffic_yellow', 'traffic_green'],
        'shutdown_timeout': 5.0,
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
        'memory_frames': 10,  # 한 선 이름 이어받기: 좌/우 선별로 마지막으로 본 뒤 이 프레임 수까지 기억 (약 0.35 s)
        'lane_width': 0.52,          # 선 중심 간격(m). 도면 직선 0.51~0.52, 11/8 카메라 보정 0.52
        # 코너 차로 폭(m)과 그 폭을 쓰는 선 곡률 범위(1/m, 시작~끝 보간). 도면: 안쪽 R0.59 → 1.7, 바깥 R0.66 → 1.5, 바깥 R1.4 완화 → 0.7
        'corner_lane_width': 0.77, 'corner_curvature': [0.8, 1.4],
        # pure pursuit 목표점 거리(m, 뒷축 기준, 중앙선 위 점). 도면 1번 코너 폐루프 시뮬: 0.9면 탈출 때 바깥 선 접촉,
        # 0.6~0.65 바깥 여유 최대 (5~7 cm). 직선 노이즈 고려 0.65
        'pursuit_distance': 0.65,
        # 중앙선이 모두 pursuit_distance 원 밖이고 선 기울기가 pursuit_ahead_tilt(°) 넘으면 선 따라 pursuit_ahead m 앞 점 목표
        'pursuit_ahead': 0.2, 'pursuit_ahead_tilt': 40,
        'refit_tolerance': 0.04,     # 2차식 맞춤 후 이 거리(m) 밖 픽셀 빼고 재맞춤
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
        # ---- 라바콘 (cones.py). 거리는 뒷바퀴 축 기준 m ----
        # 진입 판단: 앞 cone_start_range 안에 콘 cone_start_min_cones개 이상, 좌·우에 각각 콘,
        # 가장 가까운 좌·우 콘 간격 cone_gate_min~max. cone_confirm_count 주기(20 Hz) 연속이면 시작
        # [실차] 간격 범위는 실제 라바콘 배치 보고 조정 (참고 코드 0.6~0.95, 차폭 0.22)
        'cone_start_range': 1.5, 'cone_start_min_cones': 3, 'cone_confirm_count': 3,
        'cone_gate_min': 0.35, 'cone_gate_max': 1.0,
        'cone_gate_width': 0.6,      # 한쪽 줄만 보일 때 쓰는 좌·우 간격 기본값 (진입 시 측정값으로 교체)
        'cone_gate_window': 0.5,     # 같은 줄에서 가장 가까운 콘부터 이 앞 거리 안의 콘 중 가장 안쪽을 통로 경계로
        'cone_view_distance': 1.5,   # 통과 중 보는 콘 앞 거리
        'cone_band_margin': 0.15,    # 통로(중앙 ± cone_gate_max/2) 밖 이 거리 넘는 물체는 무시 (표지판 기둥 등)
        'cone_center_alpha': 0.5,    # 통로 중앙 갱신 비율 (1 = 이번 측정만)
        'cone_lookahead': 0.6,       # 통로 중앙의 몇 m 앞 지점으로 조향할지
        'cone_check_distance': 0.35, 'cone_margin': 0.03,   # 충돌 검사 원호 길이, 차체-콘 최소 여유
        'avoidance_speed': 0.13,     # 라바콘 통과 속도
        'cone_creep_speed': 0.05,    # 안전한 원호가 없을 때 속도 (정지 = 미션 실패라 멈추지 않음)
        # 라바콘 모드 종료: 마지막 라바콘을 라이다가 지난 뒤 이동 거리(m). 도면상 라바콘 끝 선 → 코너 시작 약 0.3 m
        'small_object_memory_distance': 0.3,
        'cone_max_distance': 2.5,    # 안전장치: 진입 시 첫 콘까지 거리 + 이 값 넘게 주행하면 종료. 라바콘 구간 약 2.1 m + 여유
    },
    'missions': {
        'sign_cooldown': 12.0, 'slow_seconds': 14.0, 'fork_bias': 0.55, 'pedestrian_wait': 3.2,
        'stop_offset': 0.015, 'stop_tolerance': 0.025, 'stop_approach_speed': 0.13,
        'traffic_require_stop': True, 'traffic_min_stop': 0.3, 'stop_gain': 0.9, 'stopped_speed': 0.012,
        'approach_timeout': 15.0, 'stop_line_memory': 8.0,
        'line_match_tolerance': 0.1, 'enforce_order': True,
        'sequence': ['slow', 'pedestrian', 'fork', 'traffic', 'parking'],
        # 갈림길 표지판 인식 후 차선 변경 유지·라바콘 모드 금지 거리(m). 표지판 기둥을 라바콘으로 오인하지 않을 만큼만.
        # 도면상 인식 지점 → 라바콘 시작선 약 5.1~6.1 m라 5 m면 라바콘 입구에서 아직 꺼져 있을 수 있음
        'fork_distance': 3.0,
        'fork_wait_for_obstacles': True, 'fork_clear_distance': 0.1, 'pedestrian_require_line': True,
        'trigger_line_max_distance': 1.5, 'trigger_line_freshness': 0.4,
    },
    'parking': {
        'require_camera_during_sequence': False, 'enabled': True, 'align_timeout': 24.0,
        'align_speed': 0.08, 'line_distance': 0.15, 'distance_tolerance': 0.025,
        'lateral_tolerance': 0.035, 'heading_tolerance': 0.09, 'stable_seconds': 0.6,
        'obstacle_wait': 2.0,  # 시간표 중 장애물로 멈춰 기다리는 최대 누적 시간(s). 넘으면 접촉 감점 감수하고 진행
        'sequence': [
            {'seconds': 2.1, 'speed': 0.12, 'steering': 0},
            {'seconds': 6.65, 'speed': -0.1, 'steering': -0.5},
            {'seconds': 3.0, 'speed': -0.1, 'steering': 0},
            {'seconds': 3.2, 'speed': 0, 'steering': 0},
        ],
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
        # 선 색으로 현재 차로 확인: 노랑/흰 → 2, 흰/흰 → 1, 흰/노랑 → 0. 새 카메라 프레임 이 수만큼 연속 같으면 index 보정
        'index_confirm_frames': 5,
    },
}

YOLO_CLASSES = ['traffic_red', 'traffic_yellow', 'traffic_green', 'left', 'right', 'pedestrian', 'slow', 'parking']

