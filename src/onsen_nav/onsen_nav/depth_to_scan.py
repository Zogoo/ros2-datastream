"""Depth image -> low-obstacle LaserScan (/scan_low) — pure logic.

The apt depthimage_to_laserscan assumes a level camera; ours pitches down 20°,
so its center-row band would read the floor at ~1 m as a wall. Instead:
back-project every (decimated) pixel to 3D in base_link using the mount pose
from robot_spec.json, keep only points in the low-obstacle height band
(above the floor, below the LIDAR plane), and bin them by azimuth into a scan.
This is the layer that sees bath rims, stools, bins and towels.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

Z_MIN = 0.05          # above floor (excludes the floor plane itself)
Z_MAX = 0.55          # below the LIDAR plane (those obstacles are in /scan)
DECIMATE = 4
N_BINS = 90           # one bin per 1 deg over the ~87 deg HFOV with margin
FOV_HALF = math.radians(50.0)


@dataclass
class DepthCameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    mount_x: float
    mount_z: float
    pitch_rad: float   # positive = pitched down

    @classmethod
    def from_spec(cls, cam_spec: dict) -> DepthCameraModel:
        w, h = cam_spec["width"], cam_spec["height"]
        # FE builds the projection from the *vertical* fov derived off hfov and
        # aspect (see frontend/src/sensors/depthCamera.js): fy = fx here too.
        hfov = math.radians(cam_spec["hfov_deg"])
        fx = (w / 2) / math.tan(hfov / 2)
        return cls(
            width=w, height=h, fx=fx, fy=fx, cx=w / 2, cy=h / 2,
            mount_x=float(cam_spec["position"][0]),
            mount_z=float(cam_spec["position"][2]),
            pitch_rad=math.radians(-float(cam_spec.get("pitch_deg", 0))),
        )


def depth_to_scan(
    depth_mm: np.ndarray,
    cam: DepthCameraModel,
    z_band: tuple[float, float] = (Z_MIN, Z_MAX),
    n_bins: int = N_BINS,
) -> tuple[np.ndarray, float, float]:
    """Returns (ranges[n_bins], angle_min, angle_increment) in base_link.

    ranges are horizontal distances from base_link origin; inf = no return.
    """
    h, w = depth_mm.shape
    vs, us = np.mgrid[0:h:DECIMATE, 0:w:DECIMATE]
    z_cam = depth_mm[::DECIMATE, ::DECIMATE].astype(np.float64) / 1000.0

    valid = z_cam > 0.01
    u = us[valid].astype(np.float64)
    v = vs[valid].astype(np.float64)
    d = z_cam[valid]

    # camera frame: x right, y down, z forward (optical)
    x_c = (u - cam.cx) / cam.fx * d
    y_c = (v - cam.cy) / cam.fy * d

    # rotate by mount pitch (about camera x-axis) into base_link axes:
    # forward = cos(p)*z - sin(p)*(-y_down) ... derive with z_up = -y_c
    cp, sp = math.cos(cam.pitch_rad), math.sin(cam.pitch_rad)
    fwd = cp * d - sp * (-y_c)
    up = sp * d + cp * (-y_c)
    left = -x_c

    bx = cam.mount_x + fwd
    bz = cam.mount_z + up
    by = left

    keep = (bz > z_band[0]) & (bz < z_band[1]) & (bx > 0.05)
    bx, by = bx[keep], by[keep]

    angle_min = -FOV_HALF
    angle_inc = (2 * FOV_HALF) / n_bins
    ranges = np.full(n_bins, np.inf)
    if bx.size:
        az = np.arctan2(by, bx)
        rng = np.hypot(bx, by)
        bins = ((az - angle_min) / angle_inc).astype(np.int32)
        ok = (bins >= 0) & (bins < n_bins)
        np.minimum.at(ranges, bins[ok], rng[ok])
    return ranges, angle_min, angle_inc
