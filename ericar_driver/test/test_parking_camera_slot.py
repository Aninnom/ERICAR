"""Camera cells use raw LiDAR occupancy, not obstacle-cluster formation."""
import math

import numpy as np

from test_parking_turn_done import config
from ericar_driver.lidar import scan_sectors
from ericar_driver.parking_gap import classify_camera_slot


def cell():
    return dict(entrance=[.75,-.20],tangent=[1.,0.],normal=[0.,-1.],
                width=.50,depth=.59,polygon=[[.5,-.2],[1.,-.2],[1.,-.79],[.5,-.79]],
                source='painted_grid')


def no_return_scan(c,value=np.inf):
    angles=np.linspace(-math.radians(110),math.radians(110),881)
    ranges=np.full(len(angles),value,float)
    lidar=dict(c['answer']['lidar']);lidar['zero_is_no_return']=True
    return scan_sectors(ranges,angles[0],angles[1]-angles[0],.01,12.,lidar)


def local_points(c,world):
    return np.asarray(world,float)-[c['robot']['lidar']['x'],c['robot']['lidar'].get('y',0.)]


def test_disconnected_raw_returns_inside_camera_cell_are_occupied():
    c=config();scan=no_return_scan(c)
    # Deliberately separated points: no object cluster is required.
    scan['parking_points']=local_points(c,[[.58,-.35],[.70,-.52],[.91,-.70]])
    state,evidence=classify_camera_slot(cell(),scan,c['robot'],c['answer']['parking'])
    assert state=='OCCUPIED'
    assert evidence['reason']=='RAW_INTERIOR_RETURNS'
    assert evidence['interior_hits']==3


def test_sparse_interior_return_can_never_trigger_free_latch():
    c=config();scan=no_return_scan(c)
    scan['parking_points']=local_points(c,[[.70,-.52]])
    state,evidence=classify_camera_slot(cell(),scan,c['robot'],c['answer']['parking'])
    assert state=='UNKNOWN' and evidence['reason']=='SPARSE_INTERIOR_RETURNS'


def test_inf_and_limo_zero_no_return_are_valid_free_space_evidence():
    c=config()
    for value in (np.inf,0.):
        scan=no_return_scan(c,value)
        assert not np.any(scan['parking_rays']['measured'])
        assert np.all(scan['parking_rays']['valid'])
        state,evidence=classify_camera_slot(cell(),scan,c['robot'],c['answer']['parking'])
        assert state=='FREE',(value,evidence)
        assert evidence['reason']=='VALID_CLEAR_RAYS'


def test_nan_rays_remain_unknown_not_free():
    c=config();scan=no_return_scan(c,np.nan)
    state,evidence=classify_camera_slot(cell(),scan,c['robot'],c['answer']['parking'])
    assert state=='UNKNOWN'
    assert evidence['covered_samples']==0


def test_adjacent_cell_returns_do_not_mark_selected_cell_occupied():
    c=config();scan=no_return_scan(c)
    scan['parking_points']=local_points(c,[[1.08,-.35],[1.16,-.52],[1.25,-.70]])
    state,evidence=classify_camera_slot(cell(),scan,c['robot'],c['answer']['parking'])
    assert state=='FREE'
    assert evidence['interior_hits']==0
