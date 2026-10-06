"""autorace_lidar 공용 함수 모음... 일단 적당히!

미션별 모듈(cone_corridor, roundabout, tunnel)과 estop_node가 같이 쓴다.
ROS 노드가 아니라 순수 함수만 모아 두었기 때문에 rclpy 없이도 시험할 수 있다.

좌표계 약속 (라이다 기준)
    x: 차량 앞쪽, y: 차량 왼쪽
    각도: x축에서 반시계 방향이 +, 즉 왼쪽이 +각도, 오른쪽이 -각도
"""

import math
from typing import List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# 기본 수학 도우미
# ---------------------------------------------------------------------------

def clamp(value: float, low: float, high: float) -> float:
    """value를 [low, high] 범위로 제한한다."""
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    """각도(rad)를 [-pi, pi] 범위로 맞춘다."""
    return math.atan2(math.sin(angle), math.cos(angle))


# ---------------------------------------------------------------------------
# LaserScan 전처리
# ---------------------------------------------------------------------------

def scan_to_arrays(
    msg,
    angle_offset_deg: float = 0.0,
    max_range: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """LaserScan을 (각도 배열, 거리 배열)로 바꾼다.

    유효하지 않은 값(nan, inf, range_min 미만, range_max 초과)은 버리고
    유효한 점만 돌려준다. max_range를 주면 그보다 먼 점도 버린다.
    msg는 sensor_msgs/LaserScan이며, ranges/angle_min/angle_increment/
    range_min/range_max 속성만 있으면 된다.

    angle_offset_deg는 센서의 0도가 차량 정면이 아닐 때 보정하는 값이다.
    실차에서 정면이 어긋나 있으면 이 값만 바꾸면 된다.
    """
    ranges = np.asarray(msg.ranges, dtype=float)
    angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
    angles = angles + math.radians(angle_offset_deg)
    angles = np.arctan2(np.sin(angles), np.cos(angles))

    upper = msg.range_max
    if max_range is not None:
        upper = min(max_range, msg.range_max)
    valid = (
        np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= upper)
    )
    return angles[valid], ranges[valid]


def polar_to_xy(
    angles: np.ndarray,
    ranges: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """극좌표(각도, 거리)를 (x, y) 좌표로 바꾼다."""
    return ranges * np.cos(angles), ranges * np.sin(angles)


def scan_to_xy(
    msg,
    angle_offset_deg: float = 0.0,
    max_range: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """LaserScan을 유효한 점의 (x, y) 좌표 배열로 바로 바꾼다."""
    angles, ranges = scan_to_arrays(msg, angle_offset_deg, max_range)
    return polar_to_xy(angles, ranges)


# ---------------------------------------------------------------------------
# 영역 선택
# ---------------------------------------------------------------------------

def sector_mask(
    angles: np.ndarray,
    lo_deg: float,
    hi_deg: float,
) -> np.ndarray:
    """lo_deg ~ hi_deg 사이 각도에 있는 점의 마스크를 만든다."""
    lo = math.radians(lo_deg)
    hi = math.radians(hi_deg)
    return (angles >= lo) & (angles <= hi)


def sector_min(
    angles: np.ndarray,
    ranges: np.ndarray,
    lo_deg: float,
    hi_deg: float,
    default: float = float('inf'),
) -> float:
    """lo_deg ~ hi_deg 구간에서 가장 가까운 거리를 구한다.

    구간 안에 점이 하나도 없으면 default를 돌려준다.
    기본값 inf는 그 방향에 장애물이 없다는 뜻이다.
    """
    mask = sector_mask(angles, lo_deg, hi_deg)
    if not np.any(mask):
        return default
    return float(np.min(ranges[mask]))


def box_mask(
    x: np.ndarray,
    y: np.ndarray,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> np.ndarray:
    """차량 기준 직사각형 관심 영역 안에 있는 점의 마스크를 만든다."""
    return (x >= x_min) & (x <= x_max) & (y >= y_min) & (y <= y_max)


# ---------------------------------------------------------------------------
# 군집화 (라바콘, 벽 같은 물체 나누기)
# ---------------------------------------------------------------------------

def cluster_points(
    x: np.ndarray,
    y: np.ndarray,
    gap: float = 0.15,
    min_points: int = 1,
) -> List[np.ndarray]:
    """스캔 순서로 이웃한 점 사이 거리가 gap 이하이면 한 덩어리로 묶는다.

    반환값은 덩어리별 점 좌표(N x 2) 배열의 리스트이다.
    점이 min_points개보다 적은 덩어리는 잡음으로 보고 버린다.
    """
    if x.size == 0:
        return []
    points = np.column_stack((x, y))
    steps = np.hypot(np.diff(x), np.diff(y))
    split_idx = np.where(steps > gap)[0] + 1
    groups = np.split(points, split_idx)
    return [g for g in groups if len(g) >= min_points]


def cluster_centers(clusters: List[np.ndarray]) -> np.ndarray:
    """각 덩어리의 중심점(평균) 좌표를 N x 2 배열로 돌려준다."""
    if not clusters:
        return np.empty((0, 2))
    return np.array([c.mean(axis=0) for c in clusters])


def cluster_extent(cluster: np.ndarray) -> float:
    """덩어리의 크기(바운딩 박스 대각선 길이)를 구한다.

    라바콘처럼 작은 물체와 벽처럼 긴 물체를 구분할 때 쓴다.
    """
    span = cluster.max(axis=0) - cluster.min(axis=0)
    return float(math.hypot(span[0], span[1]))


def split_left_right(
    points: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """점들을 왼쪽(y > 0)과 오른쪽(y <= 0)으로 나눈다."""
    if points.size == 0:
        return np.empty((0, 2)), np.empty((0, 2))
    return points[points[:, 1] > 0.0], points[points[:, 1] <= 0.0]


def fit_line(
    points: np.ndarray,
    min_points: int = 2,
) -> Optional[Tuple[float, float]]:
    """점들에 직선 y = a * x + b를 맞춘다. (a, b)를 돌려준다.

    점이 부족하거나 x 값이 거의 변하지 않으면 None을 돌려준다.
    터널 벽처럼 차량과 거의 나란한 선에 쓰는 용도이다.
    """
    if len(points) < min_points:
        return None
    if float(np.ptp(points[:, 0])) < 1e-6:
        return None
    a, b = np.polyfit(points[:, 0], points[:, 1], 1)
    return float(a), float(b)


# ---------------------------------------------------------------------------
# 값 평활화
# ---------------------------------------------------------------------------

class LowPassFilter:
    """지수 이동 평균 필터. 값이 튀는 것을 줄일 때 쓴다.

    alpha가 클수록 새 값을 빨리 따라가고, 작을수록 부드럽게 변한다.
    """

    def __init__(self, alpha: float = 0.3):
        """alpha(0 ~ 1)를 정하고 필터를 초기화한다."""
        self.alpha = alpha
        self.value: Optional[float] = None

    def update(self, new_value: float) -> float:
        """새 값을 넣고 필터링된 값을 돌려준다."""
        if self.value is None:
            self.value = new_value
        else:
            self.value = (
                self.alpha * new_value + (1.0 - self.alpha) * self.value
            )
        return self.value

    def reset(self) -> None:
        """저장된 값을 지운다. 미션이 바뀔 때 호출한다."""
        self.value = None
