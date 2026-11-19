"""시작선 기준 줄 시작점 (parking_start_anchor)"""
import numpy as np

from test_parking_turn_done import config
from ericar_driver.parking_start_anchor import anchor_from_start_line


def test_row_start_is_fixed_distance_after_start_line():
    c=config()['answer']['parking']
    anchor=anchor_from_start_line(.60,[[[.4,-.23],[1.0,-.23]],[[1.0,-.25],[1.5,-.25]]],c)
    assert np.allclose(anchor['anchor'],[.60+c['template_start_to_row'],-.24],atol=.011) and np.allclose(anchor['tangent'],[1.,0.])
    assert anchor['confidence']=='START_LINE_OFFSET'


def test_other_lines_and_far_segments_do_not_move_the_row():
    c=config()['answer']['parking']
    lines=[[[.4,-.25],[1.0,-.25]],[[.6,-.50],[1.2,-.50]],[[1.8,-.10],[2.4,-.30]]]   # 차로 선, 칸 안쪽 선, 먼 선분
    assert abs(anchor_from_start_line(.60,lines,c)['anchor'][1]+.25)<1e-6


def test_no_lane_line_uses_default_lateral():
    c=config()['answer']['parking']
    assert np.allclose(anchor_from_start_line(.50,[],c)['anchor'],[.50+c['template_start_to_row'],-.26])
