"""Camera cells and LiDAR objects share one bounded, inspectable frame."""
import json
import numpy as np

from test_parking_turn_done import config
from ericar_driver.parking_fusion import fuse_template_slots
from ericar_driver.parking_template import slots_from_anchor


def scene(shift_y=.0):
    c=config();slots=slots_from_anchor([.5,-.2],[1.,0.],[0.,-1.],.5,.59,3)
    points=[]
    for x,y in ((1.25,-.25+shift_y),(1.75,-.25+shift_y)):
        points.extend([[x,y],[x+.01,y+.005],[x+.02,y-.005],[x+.03,y]])
    origin=np.array([c['robot']['lidar']['x'],c['robot']['lidar'].get('y',0.)])
    local=np.asarray(points)-origin
    return c,slots,dict(points=local,parking_points=local)


def test_bounded_alignment_brings_shifted_lidar_clusters_onto_cells():
    c,slots,scan=scene(.15)
    fused,assigned,debug,offset=fuse_template_slots(slots,scan,c['robot'],c['answer']['parking'])
    assert abs(offset[0])<=.15 and .10<=offset[1]<=.15
    assert [len(part) for part in assigned]==[0,4,4]
    assert [cluster['slot_index'] for cluster in debug['clusters']]==[1,2]
    json.dumps(debug)  # telemetry must not contain NumPy scalar objects


def test_boundary_return_is_assigned_to_only_one_cell():
    c,slots,scan=scene(0.)
    # Add a compact cluster exactly around the S1/S2 longitudinal boundary.
    origin=np.array([c['robot']['lidar']['x'],c['robot']['lidar'].get('y',0.)])
    boundary=np.array([[1.,-.4],[1.01,-.4],[1.02,-.4]])-origin
    scan['points']=scan['parking_points']=np.vstack((scan['parking_points'],boundary))
    _,assigned,debug,_=fuse_template_slots(slots,scan,c['robot'],c['answer']['parking'])
    assert sum(len(part) for part in assigned)==len(scan['parking_points'])
    assert all(cluster['slot_index'] is None or 0<=cluster['slot_index']<3 for cluster in debug['clusters'])


def test_empty_scan_keeps_nominal_camera_template():
    c,slots,_=scene();scan=dict(points=np.empty((0,2)),parking_points=np.empty((0,2)))
    fused,assigned,debug,offset=fuse_template_slots(slots,scan,c['robot'],c['answer']['parking'])
    assert np.allclose(offset,[0.,0.]) and np.allclose(fused[0]['polygon'],slots[0]['polygon'])
    assert debug['reason']=='NO_LIDAR_CLUSTER_KEEP_ALIGNMENT' and not any(map(len,assigned))
