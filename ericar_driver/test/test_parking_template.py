"""White start/side anchor renders fixed cells; LiDAR only marks occupancy."""
from unittest.mock import patch

import numpy as np

from test_parking_turn_done import config
from ericar_driver.parking import Parking
from ericar_driver.parking_template import slots_from_anchor


def landmarks(start=True,anchor=True):
    anchors=[] if not anchor else [dict(anchor=[.50,-.20],tangent=[1.,0.],normal=[0.,-1.],
                                          confidence='COLOR_TRANSITION',length=.8,pixel=[0,0,1,1],
                                          support_pixel=[0,0,1,1])]
    return dict(start_seen=start,anchors=anchors,start_segments=[],white_longitudinal=[],
                yellow_longitudinal=[],reason='ANCHOR_FOUND' if anchors else 'WAITING_FOR_WHITE_SIDE')


def test_known_anchor_renders_three_metric_cells():
    cells=slots_from_anchor([.50,-.20],[1.,0.],[0.,-1.],.50,.59,3)
    assert [cell['slot_index'] for cell in cells]==[0,1,2]
    assert np.allclose([cell['entrance'] for cell in cells],[[.75,-.20],[1.25,-.20],[1.75,-.20]])
    assert all(cell['width']==.50 and cell['depth']==.59 for cell in cells)


def test_start_then_stable_anchor_selects_first_free_template_cell():
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    image=np.zeros((480,640,3),np.uint8)
    def occupancy(slot,*_):
        return ('OCCUPIED' if slot['slot_index']==0 else 'FREE'),dict(reason='TEST')
    with patch('ericar_driver.parking.parking_template_landmarks',return_value=landmarks()), \
         patch('ericar_driver.parking.classify_camera_slot',side_effect=occupancy):
        assert not p._observe_template(image,1.,{},1.,1.,0.,0.)
        assert not p._observe_template(image,1.1,{},1.1,1.1,0.,0.)
        assert p._observe_template(image,1.2,{},1.2,1.2,0.,0.)
    assert p.template_armed and len(p.template_slots)==3
    assert p.selected['source']=='parking_template' and p.selected['slot_index']==1
    assert p.template_debug['evaluations'][0]['state']=='OCCUPIED'
    assert p.template_debug['evaluations'][1]['control_applied']


def test_template_waits_when_no_side_anchor_exists():
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    image=np.zeros((480,640,3),np.uint8)
    with patch('ericar_driver.parking.parking_template_landmarks',return_value=landmarks(anchor=False)):
        p._observe_template(image,1.,{},1.,1.,0.,0.)
        p._observe_template(image,1.1,{},1.1,1.1,0.,0.)
    assert p.template_armed and not p.template_slots and p.selected is None
    assert p.template_debug['reason']=='WAITING_FOR_SIDE_ANCHOR'


def test_template_mode_never_uses_obstacle_pair_gap_as_target():
    c=config();c['answer']['parking']['template_enabled']=True;p=Parking(c);p.start(0.)
    image=np.zeros((480,640,3),np.uint8)
    scan=dict(valid_fraction=1.,parking_rays=dict(measured=np.ones(8,bool)),parking_points=np.empty((0,2)),points=np.empty((0,2)))
    with patch.object(p,'_observe_template',return_value=False), \
         patch.object(p,'_find_gap',side_effect=AssertionError('gap path must stay disabled')), \
         patch.object(p,'_clear',return_value=True):
        result=p.step(1.,image,1.,scan,1.,None,1.,[],0.,dict(valid=True),search_command=(.12,0.))
    assert result[2] in ('parking_cruise','parking_scan') and p.selected is None
