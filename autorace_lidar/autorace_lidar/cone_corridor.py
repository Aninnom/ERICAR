"""미션1: 라바콘 구간 인식.

좌우로 줄지어 선 라바콘 사이의 중앙 경로를 찾아 횡오차와 방향 오차를 계산한다.
조향 명령은 만들지 않고 측정값과 검출 여부만 돌려준다(topic_spec 원칙).

좌표계: x 앞, y 왼쪽(REP-103). 횡오차와 방향 오차는 왼쪽이 +이다.
"""

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from autorace_lidar.lidar_common import (
    LowPassFilter, box_mask, cluster_centers, cluster_extent, cluster_points,
    split_left_right)


@dataclass
class ConeResult:
    """라바콘 구간 인식 결과. detected가 False이면 다른 값은 무시한다."""

    detected: bool = False
    lateral_error: float = 0.0   # 중앙선 목표점의 y (m, 왼쪽 +)
    heading_error: float = 0.0   # 목표점 방향 각도 (rad, 왼쪽 +)
    target_x: float = 0.0        # 목표점 좌표 (디버깅, RViz 표시용)
    target_y: float = 0.0
    num_left: int = 0            # 왼쪽에서 잡힌 콘 개수
    num_right: int = 0           # 오른쪽에서 잡힌 콘 개수
    min_cone_dist: float = float('inf')   # 가장 가까운 콘까지 거리 (m)


class ConeCorridor:
    """좌우 라바콘 줄 사이의 중앙선을 따라가기 위한 오차를 계산한다.

    처리 순서
        1. 차량 앞 관심영역의 라이다 점만 남긴다.
        2. 점을 덩어리로 묶고, 크기가 작은 덩어리만 콘으로 본다.
        3. 콘을 왼쪽/오른쪽으로 나눠 look_ahead 거리에서의 좌우 경계 y를 구한다.
        4. 두 경계의 가운데를 목표점으로 잡아 횡오차와 방향 오차를 낸다.
        5. 한쪽만 보이면 corridor_width를 이용해 반대쪽 경계를 추정한다.
    """

    def __init__(
        self,
        look_ahead: float = 1.0,
        roi_x_max: float = 2.0,
        roi_y_half: float = 1.2,
        min_x: float = 0.10,
        cluster_gap: float = 0.12,
        min_points: int = 2,
        max_cone_extent: float = 0.35,
        corridor_width: float = 0.9,
        alpha: float = 0.5,
    ):
        """파라미터를 정하고 필터를 초기화한다. 단위는 m, 필터 alpha는 0 ~ 1."""
        self.look_ahead = look_ahead
        self.roi_x_max = roi_x_max
        self.roi_y_half = roi_y_half
        self.min_x = min_x
        self.cluster_gap = cluster_gap
        self.min_points = min_points
        self.max_cone_extent = max_cone_extent
        self.corridor_width = corridor_width
        self._lat_filter = LowPassFilter(alpha)
        self._head_filter = LowPassFilter(alpha)

    def reset(self) -> None:
        """필터를 비운다. 미션이 시작되거나 콘이 사라졌을 때 호출한다."""
        self._lat_filter.reset()
        self._head_filter.reset()

    def find_cones(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """라이다 점에서 콘의 중심 좌표(N x 2)를 찾는다."""
        mask = box_mask(
            x, y, self.min_x, self.roi_x_max, -self.roi_y_half, self.roi_y_half)
        clusters = cluster_points(
            x[mask], y[mask], gap=self.cluster_gap, min_points=self.min_points)
        cones = [c for c in clusters if cluster_extent(c) <= self.max_cone_extent]
        return cluster_centers(cones)

    @staticmethod
    def _side_y_at(points: np.ndarray, x_target: float) -> Optional[float]:
        """한쪽 콘 줄이 x_target에서 놓이는 y 위치를 구한다. 콘이 없으면 None."""
        if len(points) == 0:
            return None
        order = np.argsort(points[:, 0])
        # 범위 밖에서는 가장 가까운 콘의 y를 그대로 쓴다(곡선에서도 튀지 않음).
        return float(np.interp(x_target, points[order, 0], points[order, 1]))

    def process(self, x: np.ndarray, y: np.ndarray) -> ConeResult:
        """유효한 라이다 점(x, y)에서 중앙선 오차를 계산한다."""
        cones = self.find_cones(x, y)
        if len(cones) == 0:
            self.reset()
            return ConeResult()

        left, right = split_left_right(cones)
        tx = self.look_ahead
        y_left = self._side_y_at(left, tx)
        y_right = self._side_y_at(right, tx)

        if y_left is not None and y_right is not None:
            ty = 0.5 * (y_left + y_right)
        elif y_left is not None:
            ty = y_left - 0.5 * self.corridor_width
        elif y_right is not None:
            ty = y_right + 0.5 * self.corridor_width
        else:
            self.reset()
            return ConeResult()

        lateral = self._lat_filter.update(ty)
        heading = self._head_filter.update(math.atan2(ty, tx))
        min_dist = float(np.min(np.hypot(cones[:, 0], cones[:, 1])))
        return ConeResult(
            detected=True,
            lateral_error=lateral,
            heading_error=heading,
            target_x=tx,
            target_y=ty,
            num_left=len(left),
            num_right=len(right),
            min_cone_dist=min_dist,
        )
