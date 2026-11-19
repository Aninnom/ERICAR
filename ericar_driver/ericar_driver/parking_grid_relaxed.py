"""도색 칸 추정 완화판 (11/18 혁준, 실차 redecide_1118_03·04 대응). parking_grid.py의 보조 경로

기존 partial_grid_slots_from_ground_segments는 칸 사이 선(가로선) 두 개가 입구부터 칸 안쪽 끝(깊이 0.59 m)까지
이어져 보여야 칸으로 인정함. 실차에서는 가로선이 화면 오른쪽 끝에 잘려 0.18~0.24 m만 보임 (카메라 시야각 71°,
1 m 앞에서 옆 ±0.5 m) → 67프레임 전부 탈락 (redecide_1118_04 151~168 s)
여기서는 칸 깊이·폭이 이미 아는 값이므로:
- 가로선은 입구 쪽 visible_min(m)만 보이면 인정 (입구선보다 차로 쪽에 놓인 선은 제외)
- 가로선 두 개가 칸 폭만큼 떨어져 있으면 그 사이를 칸으로
- (single=True) 가로선이 하나뿐이면 그 선 옆에 칸 폭만큼의 칸을 가정. 입구선이 그 칸 폭 전체에 이어져 있을 때만
  (어느 쪽이 비었는지는 라이다 판정이 가름). redecide_1118_04 재생: 첫 칸(주차 표지판 뒤, 상자 앞)이 이 경로로만 FREE
정윤 코드와 겹치지 않게 별도 파일. parking_grid.parking_grid_candidates에서 기존 두 방식이 칸을 못 만들 때만 호출
"""
import math
import numpy as np

from .parking_grid import _angle_difference, _cluster_lines, _covers


def relaxed_partial_slots(segments, slot_width=.50, slot_depth=.59, size_tolerance=.12, visible_min=.15,
                          edge=.05, angle_tolerance=.18, merge_distance=.045, extrapolation=.14, single=False):
    """입구선 + 일부만 보이는 가로선으로 칸 후보를 만듦. 반환 형식은 partial_grid_slots_from_ground_segments와 같음"""
    segments = np.asarray(segments, float).reshape(-1, 4)
    if not len(segments):
        return []
    starts, ends = segments[:, :2], segments[:, 2:]
    delta = ends-starts
    lengths = np.linalg.norm(delta, axis=1)
    valid = np.isfinite(segments).all(axis=1) & (lengths > .10)
    if np.count_nonzero(valid) < 2:
        return []
    starts, ends, delta, lengths = starts[valid], ends[valid], delta[valid], lengths[valid]
    angles = np.arctan2(delta[:, 1], delta[:, 0])
    axis_angle = math.atan2(float(np.sum(lengths*np.sin(4*angles))), float(np.sum(lengths*np.cos(4*angles))))/4
    axes = (np.array([math.cos(axis_angle), math.sin(axis_angle)]), np.array([-math.sin(axis_angle), math.cos(axis_angle)]))
    families = ([], [])
    for start, end, length, angle in zip(starts, ends, lengths, angles):
        errors = (_angle_difference(angle, axis_angle), _angle_difference(angle, axis_angle+math.pi/2))
        family = 0 if errors[0] <= errors[1] else 1
        if errors[family] > angle_tolerance:
            continue
        direction, perpendicular = axes[family], axes[1-family]
        projection = sorted((float(start@direction), float(end@direction)))
        families[family].append(dict(offset=float(((start+end)/2)@perpendicular), projection_min=projection[0],
                                     projection_max=projection[1], length=float(length)))
    merged = (_cluster_lines(families[0], merge_distance), _cluster_lines(families[1], merge_distance))
    candidates = []
    for side_index in (0, 1):
        # side: 칸 사이 선(가로선)이 뻗는 방향, across: 입구선이 뻗는 방향
        direction, across = axes[side_index], axes[1-side_index]
        for entrance_line in merged[1-side_index]:
            level = entrance_line['offset']                       # 입구선 위치 (direction 좌표)
            # 칸 안쪽 = 입구선 기준 차에서 먼 쪽
            middle = (entrance_line['projection_min']+entrance_line['projection_max'])/2
            point = direction*level+across*middle
            sign = 1.0 if np.linalg.norm(point+direction*slot_depth) >= np.linalg.norm(point-direction*slot_depth) else -1.0
            low, high = sorted((level, level+sign*visible_min))
            attached = sorted(
                line['offset'] for line in merged[side_index]
                if _covers(line, low, high, edge)                  # 입구에서 안쪽으로 visible_min만큼은 보여야 함
                and entrance_line['projection_min']-extrapolation <= line['offset'] <= entrance_line['projection_max']+extrapolation)
            if not attached:
                continue
            spans, paired = [], set()
            for first, second in zip(attached, attached[1:]):
                if abs(second-first-slot_width) <= size_tolerance:
                    spans.append((first, second, .65))
                    paired.update((('right', first), ('left', second)))
            for offset in (attached if single else []):
                # 짝이 없는 쪽에는 칸 폭만큼의 칸을 가정. 입구선(같은 색)이 그 칸 폭 전체에 이어져 있어야 함
                # (주차 줄 밖은 차로 선 색이 달라 입구선이 끊김 → 줄 밖 바닥을 칸으로 가정하지 않게)
                if ('right', offset) not in paired and entrance_line['projection_max'] >= offset+slot_width-.08:
                    spans.append((offset, offset+slot_width, .5))
                if ('left', offset) not in paired and entrance_line['projection_min'] <= offset-slot_width+.08:
                    spans.append((offset-slot_width, offset, .5))
            normal = sign*direction
            for first, second, score in spans:
                width = second-first
                entrance = direction*level+across*(first+second)/2
                corners = np.array([entrance-width/2*across, entrance+width/2*across,
                                    entrance+width/2*across+slot_depth*normal, entrance-width/2*across+slot_depth*normal])
                center = entrance+normal*slot_depth/2
                candidates.append(dict(entrance=entrance.tolist(), tangent=across.tolist(), normal=normal.tolist(),
                                       width=float(width), depth=float(slot_depth), polygon=corners.tolist(),
                                       center=center.tolist(), source='painted_grid_relaxed', requires_measured_rays=True,
                                       geometry_score=score))
    kept = []
    for candidate in sorted(candidates, key=lambda item: -item['geometry_score']):
        center = np.asarray(candidate['center'])
        if any(np.linalg.norm(center-np.asarray(other['center'])) < slot_width*.5 for other in kept):
            continue
        kept.append(candidate)
    return kept
