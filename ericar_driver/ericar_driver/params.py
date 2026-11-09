"""주행 파라미터. 담당별 구역만 수정. 거리 m, 속도 m/s, 조향 rad(+좌회전)"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)

ANSWER_CONFIG = {
    'rate_hz': 20.0, 'debug_rate_hz': 5.0, 'sensor_timeout': 0.7, 'odometry_timeout': 0.7,
    'base_speed': 0.4, 'slow_speed': 0.2, 'minimum_curve_speed': 0.15,
    'corner_speed': 0.2,   # 앞에 코너(선 곡률 ≥ lane corner_curvature 시작값) 보이면 상한. 도면 코너 시뮬: 0.4 진입은 바깥 접촉

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
        'white_min': [0, 0, 165], 'white_max': [179, 85, 255],
        # 3차선 흰 점선: 노란 선이 탐색 범위에 하나도 없을 때만 차선 후보로 추가. 진행 방향으로 이 길이(m) 이상 이어진 흰 획만 사용
        'white_lane_min_length': 0.15,
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
        'emergency_distance': 0.18, 'front_half_angle': 0.3,
        'avoidance_speed': 0.13, 'planning_margin': 0.025, 'cluster_gap': 0.12,
        'small_object_min_width': 0.015, 'small_object_max_width': 0.12, 'small_object_range': 1.8,
        'small_object_clearance': 0.025, 'cone_inside_required': True, 'cone_side_deadband': 0.02,
        'cone_inside_margin': 0.18, 'cone_row_spacing': 1.0, 'cone_spacing_tolerance': 0.2, 'cone_turn_distances': [0.1, 0.25, 0.4],
        'cone_planning_distance': 0.8, 'road_heading_limit': 0.75, 'small_object_memory_distance': 0.8,
        'planning_point_range': 3.0, 'cluster_min_points': 2, 'wide_object_half_width': 0.9,
        'small_object_half_width': 0.75, 'planning_steps': 35, 'planning_steering_samples': 33,
    },
    'missions': {
        'sign_cooldown': 12.0, 'slow_seconds': 14.0, 'fork_bias': 0.55, 'pedestrian_wait': 3.2,
        'stop_offset': 0.015, 'stop_tolerance': 0.025, 'stop_approach_speed': 0.13,
        'traffic_require_stop': True, 'traffic_min_stop': 0.3, 'stop_gain': 0.9, 'stopped_speed': 0.012,
        'approach_timeout': 15.0, 'stop_line_memory': 8.0, 'line_missing_timeout': 2.0,
        'line_match_tolerance': 0.1, 'enforce_order': True,
        'sequence': ['slow', 'pedestrian', 'fork', 'traffic', 'parking'], 'fork_distance': 5.0,
        'fork_wait_for_obstacles': True, 'fork_clear_distance': 0.1, 'pedestrian_require_line': True,
        'trigger_line_max_distance': 1.5, 'trigger_line_freshness': 0.4,
    },
    'parking': {
        'require_camera_during_sequence': False, 'enabled': True, 'align_timeout': 24.0,
        'align_speed': 0.08, 'line_distance': 0.15, 'distance_tolerance': 0.025,
        'lateral_tolerance': 0.035, 'heading_tolerance': 0.09, 'stable_seconds': 0.6,
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
        'check_distance': 1.8, 'lookahead': 0.55, 'speed': 0.18, 'margin': 0.035, 'safety_horizon': 0.3,
        'finish_error': 0.025, 'finish_heading': 0.035, 'max_distance': 3.0,
    },
}

YOLO_CLASSES = ['traffic_red', 'traffic_yellow', 'traffic_green', 'left', 'right', 'pedestrian', 'slow', 'parking']

