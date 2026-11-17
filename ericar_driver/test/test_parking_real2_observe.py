"""parking_real2 camera/LiDAR fusion remains observe-only by default."""
from unittest.mock import patch

import numpy as np

from test_parking_turn_done import config
from ericar_driver.parking import Parking


def cell(x=.9):
    return dict(entrance=[x,-.2],tangent=[1.,0.],normal=[0.,-1.],width=.5,depth=.59,
                polygon=[[x-.25,-.2],[x+.25,-.2],[x+.25,-.79],[x-.25,-.79]],
                center=[x,-.495],source='painted_grid',requires_measured_rays=True)


def observe(parking,stamp,status='FREE'):
    details=dict(reason='GRID_SLOTS_FOUND',segments_px=[[1,2,3,4]],segments_ground=[])
    with patch('ericar_driver.parking.parking_grid_candidates',return_value=([cell()],details)), \
         patch('ericar_driver.parking.classify_gap',return_value=(status,dict(reason='TEST'))):
        parking._observe_grid(np.zeros((480,640,3),np.uint8),stamp,{},stamp,stamp,0.,0.)


def test_free_grid_never_replaces_control_target_in_observe_only_mode():
    p=Parking(config());p.start(0.)
    for stamp in (1.,1.1,1.2):observe(p,stamp)
    assert p.selected is None
    assert p.grid_debug['observe_only'] is True
    assert p.grid_debug['enable_actuation'] is False
    assert p.grid_debug['control_applied'] is False
    assert p.grid_debug['evaluations'][0]['state']=='FREE'
    assert p.grid_debug['evaluations'][0]['stable'] is True
    assert p.grid_debug['evaluations'][0]['observations']==3


def test_occupancy_is_reported_without_changing_existing_lidar_target():
    p=Parking(config());p.start(0.);existing=cell(1.4);existing['source']='lidar_obstacle_gap';p.selected=existing
    observe(p,1.,'OCCUPIED')
    assert p.selected is existing
    assert p.grid_debug['evaluations'][0]['state']=='OCCUPIED'
    assert p.grid_debug['evaluations'][0]['control_applied'] is False


def test_duplicate_image_stamp_is_not_counted_as_another_observation():
    p=Parking(config());p.start(0.);observe(p,1.);observe(p,1.)
    assert p.grid_debug['evaluations'][0]['observations']==1


def test_experimental_detector_error_cannot_interrupt_controller():
    p=Parking(config());p.start(0.);existing=cell(1.4);p.selected=existing
    with patch('ericar_driver.parking.parking_grid_candidates',side_effect=RuntimeError('camera test')):
        p._observe_grid(np.zeros((480,640,3),np.uint8),1.,{},1.,1.,0.,0.)
    assert p.selected is existing
    assert p.grid_debug['reason']=='GRID_DETECTOR_ERROR'
    assert p.grid_debug['observe_only'] is True
    assert p.grid_debug['enable_actuation'] is False

