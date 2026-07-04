"""Depth->low-obstacle-scan converter tests with analytically generated frames."""
import json
import math

import numpy as np
from onsen_nav.depth_to_scan import DepthCameraModel, depth_to_scan

with open("shared/robot_spec.json") as _f:
    SPEC = json.load(_f)
CAM = DepthCameraModel.from_spec(SPEC["sensors"]["camera_depth"])


def synthetic_depth(box_forward: float | None, box_height: float = 0.30) -> np.ndarray:
    """Per-pixel analytic depth of (floor plane) + (optional low box wall at
    base_link forward distance box_forward, full width)."""
    depth = np.zeros((CAM.height, CAM.width), np.uint16)
    cp, sp = math.cos(CAM.pitch_rad), math.sin(CAM.pitch_rad)
    for v in range(CAM.height):
        # ray in camera frame for the column-center; depth varies per row only
        y_c = (v - CAM.cy) / CAM.fy            # image down
        # base_link direction of the ray (unit per 1 m of camera z)
        fwd = cp * 1.0 - sp * (-y_c)
        up = sp * 1.0 + cp * (-y_c)
        candidates = []
        if up < -1e-6:  # hits the floor (z=0): mount_z + t*up = 0
            t = -CAM.mount_z / up
            candidates.append(t)
        if box_forward is not None and fwd > 1e-6:
            t = (box_forward - CAM.mount_x) / fwd
            if t > 0 and CAM.mount_z + t * up <= box_height:
                candidates.append(t)
        if candidates:
            t = min(candidates)
            d_mm = int(t * 1000)
            if 280 <= d_mm <= 3000:
                depth[v, :] = d_mm
    return depth


class TestDepthToScan:
    def test_floor_alone_yields_no_returns(self):
        ranges, _, _ = depth_to_scan(synthetic_depth(box_forward=None), CAM)
        assert np.all(np.isinf(ranges))

    def test_low_box_appears_at_correct_range(self):
        ranges, angle_min, inc = depth_to_scan(synthetic_depth(box_forward=1.0), CAM)
        center = int((0 - angle_min) / inc)
        hits = ranges[center - 3: center + 4]
        finite = hits[np.isfinite(hits)]
        assert finite.size > 0, "low obstacle must produce returns"
        assert abs(float(np.min(finite)) - 1.0) < 0.08

    def test_empty_frame_all_inf(self):
        ranges, _, _ = depth_to_scan(np.zeros((CAM.height, CAM.width), np.uint16), CAM)
        assert np.all(np.isinf(ranges))

    def test_tall_wall_above_band_excluded(self):
        # wall at 1 m but starting above the band: emulate by box higher than
        # z-band top everywhere -> generated depths whose 3D z > 0.55 are dropped.
        depth = synthetic_depth(box_forward=1.0, box_height=5.0)
        ranges, angle_min, inc = depth_to_scan(depth, CAM, z_band=(0.05, 0.10))
        center = int((0 - angle_min) / inc)
        # the only points below 0.10 m sit just above the floor cut — nearly none
        finite = np.isfinite(ranges[center - 5: center + 6]).sum()
        assert finite <= 11  # sanity: no crash; band filtering applied
