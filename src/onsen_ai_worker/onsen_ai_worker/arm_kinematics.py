"""Analytic forward + inverse kinematics for the onsen arm — pure numpy.

The arm is a planar 3R chain (shoulder/elbow/wrist pitch) on a pan yaw joint,
with a fixed servo->joint coupling `ratio` (firmware servo degrees map to
larger joint tilts). This makes closed-form IK exact, which is *more* rigorous
than the iterative KDL/TRAC-IK solvers a URDF would force on us (they cannot
express the coupling without a mimic-joint hack). See docs/research_notes.md.

FK convention mirrors frontend/src/robot/kinematics.js exactly (tilts from
vertical-up, positive pitching forward):
    pan = (d0 - 90)
    t1  = (d1 - 90) * ratio
    t2  = t1 + (90 - d2) * ratio
    t3  = t2 + (90 - d3) * ratio
fingertip = shoulder + L1·(sin t1) + L2·(sin t2) + L3·(sin t3)  (radial, along pan)
          z = shoulder_z + L1·cos t1 + L2·cos t2 + L3·cos t3

This module is the test oracle for any solver and the runtime solver itself.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass

import numpy as np

SPEC_PATH = os.environ.get("ROBOT_SPEC_PATH", "/ros2_ws/shared/robot_spec.json")

# Scoop tool-tilt band (fingertip pointing forward-down): t3 from vertical.
SCOOP_TILT_DEG = (95.0, 120.0)
SERVO_MIN, SERVO_MAX = 0.0, 180.0
GRASP_Z = 0.045   # m, fingertip height when scooping a flat towel


@dataclass
class ArmModel:
    ratio: float
    l1: float
    l2: float
    l3: float
    base: tuple[float, float, float]
    shoulder_z: float

    @classmethod
    def from_spec(cls, arm_spec: dict) -> ArmModel:
        links = arm_spec["links"]
        base = arm_spec["base_offset"]
        return cls(
            ratio=float(arm_spec["joint_ratio"]),
            l1=float(links["upper"]), l2=float(links["forearm"]), l3=float(links["wrist"]),
            base=(float(base[0]), float(base[1]), float(base[2])),
            shoulder_z=float(arm_spec["shoulder_z"]),
        )

    # ── Forward kinematics ──────────────────────────────────────────────────────

    def fk(self, degrees: list[float]) -> dict:
        d = degrees
        rad = math.radians
        pan = rad(d[0] - 90)
        t1 = rad((d[1] - 90) * self.ratio)
        t2 = t1 + rad((90 - d[2]) * self.ratio)
        t3 = t2 + rad((90 - d[3]) * self.ratio)
        dx, dy = math.cos(pan), math.sin(pan)
        shoulder = (self.base[0], self.base[1], self.shoulder_z)

        def seg(p, length, tilt):
            return (
                p[0] + length * math.sin(tilt) * dx,
                p[1] + length * math.sin(tilt) * dy,
                p[2] + length * math.cos(tilt),
            )

        elbow = seg(shoulder, self.l1, t1)
        wrist = seg(elbow, self.l2, t2)
        fingertip = seg(wrist, self.l3, t3)
        return {
            "shoulder": shoulder, "elbow": elbow, "wrist": wrist,
            "fingertip": fingertip, "tilts": (t1, t2, t3), "pan": pan,
        }

    def fingertip(self, degrees: list[float]) -> tuple[float, float, float]:
        return self.fk(degrees)["fingertip"]

    # ── Inverse kinematics ──────────────────────────────────────────────────────

    def ik(
        self, target: tuple[float, float, float],
        current: list[float] | None = None,
        tilt_band_deg: tuple[float, float] = SCOOP_TILT_DEG,
    ) -> list[float] | None:
        """Closed-form IK with tool-tilt search. Returns 6 servo degrees (J4
        roll neutral 90, J5 gripper carried from `current`) or None if the
        target is unreachable / violates servo limits across the whole band."""
        tx, ty, tz = target
        pan = math.atan2(ty - self.base[1], tx - self.base[0])
        rho = math.hypot(tx - self.base[0], ty - self.base[1])
        d0 = 90 + math.degrees(pan)
        if not SERVO_MIN <= d0 <= SERVO_MAX:
            return None

        best: list[float] | None = None
        best_cost = math.inf
        lo, hi = tilt_band_deg
        for t3_deg in np.arange(lo, hi + 1e-6, 2.5):
            sol = self._solve_for_tilt(rho, tz, math.radians(t3_deg), d0)
            if sol is None:
                continue
            cost = self._smoothness_cost(sol, current)
            if cost < best_cost:
                best, best_cost = sol, cost
        if best is None:
            return best
        return self._polish(best, target, current)

    def _solve_for_tilt(
        self, rho: float, tz: float, t3: float, d0: float,
    ) -> list[float] | None:
        # Wrist point in the (radial, z) plane, subtracting the last link.
        w_rho = rho - self.l3 * math.sin(t3)
        w_z = tz - self.l3 * math.cos(t3)
        d_rho = w_rho                       # shoulder radial = 0
        d_z = w_z - self.shoulder_z
        dist = math.hypot(d_rho, d_z)
        if dist > self.l1 + self.l2 or dist < abs(self.l1 - self.l2) or dist < 1e-9:
            return None

        gamma = math.atan2(d_rho, d_z)      # angle of the wrist vector from vertical
        cos_alpha = (self.l1**2 + dist**2 - self.l2**2) / (2 * self.l1 * dist)
        alpha = math.acos(max(-1.0, min(1.0, cos_alpha)))
        t1 = gamma - alpha                  # elbow-up branch (elbow toward +radial)
        # elbow position -> t2 is the absolute tilt of the forearm segment
        e_rho = self.l1 * math.sin(t1)
        e_z = self.shoulder_z + self.l1 * math.cos(t1)
        t2 = math.atan2(w_rho - e_rho, w_z - e_z)

        d1 = 90 + math.degrees(t1) / self.ratio
        d2 = 90 - math.degrees(t2 - t1) / self.ratio
        d3 = 90 - math.degrees(t3 - t2) / self.ratio
        servos = [d0, d1, d2, d3, 90.0, 90.0]
        if any(not SERVO_MIN <= s <= SERVO_MAX for s in servos[:4]):
            return None
        return servos

    def _smoothness_cost(self, sol: list[float], current: list[float] | None) -> float:
        if current is None:
            return abs(sol[1] - 90) + abs(sol[2] - 90) + abs(sol[3] - 90)
        return max(abs(sol[i] - current[i]) for i in range(4))

    def _polish(
        self, seed: list[float], target: tuple[float, float, float],
        current: list[float] | None, iters: int = 6, lam: float = 0.05,
    ) -> list[float]:
        """Damped least-squares refinement on [pan, d1, d2, d3] — absorbs the
        degree-rounding from the tilt-band scan. Finite-difference Jacobian of
        the fingertip position; converges in a handful of steps at this scale."""
        q = list(seed)
        tgt = np.array(target)
        for _ in range(iters):
            err = tgt - np.array(self.fingertip(q))
            if np.linalg.norm(err) < 1e-6:
                break
            jac = np.zeros((3, 4))
            for j in range(4):
                dq = list(q)
                dq[j] += 0.5
                jac[:, j] = (np.array(self.fingertip(dq)) - np.array(self.fingertip(q))) / 0.5
            jt = jac.T
            step = jt @ np.linalg.solve(jac @ jt + lam * np.eye(3), err)
            for j in range(4):
                q[j] = float(np.clip(q[j] + step[j], SERVO_MIN, SERVO_MAX))
        if current is not None:
            q[5] = current[5]
        return q


def load_arm_model() -> ArmModel:
    with open(SPEC_PATH) as f:
        return ArmModel.from_spec(json.load(f)["arm"])


def reachable_annulus(model: ArmModel, z: float = 0.03) -> tuple[float, float]:
    """Min/max floor radius (from base center) the scoop band can reach at
    height z — the mission uses this to decide approach vs. reach."""
    radii = []
    rho = 0.20
    while rho < 0.75:
        x = model.base[0] + rho  # bearing 0; arm is radially symmetric in pan
        if model.ik((x, model.base[1], z)) is not None:
            radii.append(rho)
        rho += 0.01
    if not radii:
        return (0.0, 0.0)
    return (min(radii), max(radii))


def grasp_command(degrees: list[float], ms: int = 900) -> str:
    """Firmware J line for a 6-servo absolute move."""
    vals = " ".join(str(round(d)) for d in degrees)
    return f"J {vals} {ms}"
