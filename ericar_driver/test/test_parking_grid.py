"""Painted 3x2 parking-grid proposal tests; no controller or motor publisher."""
import math
import numpy as np

from ericar_driver.parking_grid import (grid_slots_from_ground_segments,partial_grid_slots_from_ground_segments,
                                        reachable_right_slots)


def grid_segments(angle=0.,missing=()):
    """Three columns by two rows on the vehicle's right, matching known bay size."""
    width,depth=.50,.59
    segments=[]
    # Accessible edge y=-.20; second row is deliberately behind the first.
    for row,y in enumerate((-.20,-.20-depth,-.20-2*depth)):
        if ('row',row) not in missing:segments.append([.60,y,.60+3*width,y])
    for column,x in enumerate((.60,1.10,1.60,2.10)):
        if ('column',column) not in missing:segments.append([x,-.20,x,-.20-2*depth])
    if angle:
        c,s=math.cos(angle),math.sin(angle);rotation=np.array([[c,-s],[s,c]])
        values=[]
        for segment in segments:
            points=np.asarray(segment).reshape(2,2)@rotation.T;values.append(points.reshape(-1).tolist())
        return values
    return segments


def test_reconstructs_all_six_equal_cells_independent_of_grid_angle():
    for angle in (0.,math.radians(8),math.radians(-11)):
        cells=grid_slots_from_ground_segments(grid_segments(angle))
        assert len(cells)==6
        assert all(abs(cell['width']-.50)<1e-6 and abs(cell['depth']-.59)<1e-6 for cell in cells)


def test_only_three_aisle_facing_cells_are_reachable_by_existing_right_turn():
    cells=grid_slots_from_ground_segments(grid_segments())
    reachable=reachable_right_slots(cells)
    assert len(reachable)==3
    assert all(cell['normal'][1]<-.9 for cell in reachable)
    assert all(-.21<cell['entrance'][1]<-.19 for cell in reachable)


def test_missing_paint_does_not_invent_a_complete_cell():
    cells=grid_slots_from_ground_segments(grid_segments(missing=(('column',1),)))
    assert len(cells)<6
    assert all(cell['source']=='painted_grid' for cell in cells)


def test_irregular_cell_size_is_rejected():
    segments=grid_segments()
    # Move the last divider so the final bay is far wider than the known size.
    segments[-1]=[2.35,-.20,2.35,-1.38]
    cells=grid_slots_from_ground_segments(segments)
    assert len(cells)==4


def test_three_visible_sides_reconstruct_known_size_cell():
    segments=[[.60,-.20,.60,-.79],[1.10,-.20,1.10,-.79],[.60,-.20,1.10,-.20]]
    cells=partial_grid_slots_from_ground_segments(segments)
    reachable=reachable_right_slots(cells)
    assert len(reachable)==1
    assert reachable[0]['source']=='painted_grid_partial'
    assert np.allclose(reachable[0]['entrance'],[.85,-.20])
    assert reachable[0]['normal'][1]<-.9


if __name__=='__main__':
    for name,function in list(globals().items()):
        if name.startswith('test_'):function()
    print('parking grid tests OK')
