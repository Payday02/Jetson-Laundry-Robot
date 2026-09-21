#!/usr/bin/env python3
"""
trajectory.py — pure waypoint-trajectory logic (no ROS, unit-testable).

    python3 trajectory.py    # self-test
"""
from __future__ import annotations

import numpy as np

def base_to_quest(pos_map: np.ndarray, p_base: np.ndarray) -> np.ndarray:
    """Inverse of teleop's pos_map (must be orthogonal; we verify once)."""
    return pos_map.T @ p_base


def quat_yaw(z: float):
    """Unit quaternion (w,x,y,z) for yaw about Z."""
    return np.array([np.cos(z / 2), 0.0, 0.0, np.sin(z / 2)])


class TrajectoryPlayer:
    """
    Waypoint player: linear interpolation between waypoints, each with an
    optional dwell (s) at the point. Loops forever.

    waypoints: list of dicts  {x, y, z, dwell=0.0, yaw=None}
    """

    def __init__(self, waypoints, rate_hz=30.0):
        self._wps = list(waypoints)
        if len(self._wps) < 2:
            raise ValueError("need at least 2 waypoints")
        self._rate = rate_hz
        # precompute segments (dt per 1 Hz of simulation)
        self._segs = []
        for a, b in zip(self._wps, self._wps[1:]):
            pa = np.array([a['x'], a['y'], a['z']])
            pb = np.array([b['x'], b['y'], b['z']])
            dist = float(np.linalg.norm(pb - pa))
            vmax = 0.25  # m/s cap for the fake trajectory (gentle)
            seg_s = max(dist / vmax, 0.2)
            self._segs.append((pa, pb, seg_s))
        self._idx = 0
        self._seg_t = 0.0
        self._dwell_left = float(self._wps[0].get('dwell', 0.0))
        self.pos = self._segs[0][0].copy()
        self.yaw = float(self._wps[0].get('yaw', 0.0) or 0.0)

    @property
    def done(self):
        return False  # loops forever

    def step(self):
        """Advance one control tick; returns (position_base, yaw)."""
        dt = 1.0 / self._rate
        if self._dwell_left > 0:
            self._dwell_left -= dt
            return self.pos.copy(), self.yaw

        pa, pb, seg_s = self._segs[self._idx]
        self._seg_t += dt
        u = min(1.0, self._seg_t / seg_s)
        self.pos = pa + (pb - pa) * u
        yaw_to = float(self._wps[self._idx + 1].get('yaw', self.yaw) or self.yaw)
        self.yaw = self.yaw + (yaw_to - self.yaw) * min(1.0, 2.0 * dt)
        if u >= 1.0:
            self._idx += 1
            self._seg_t = 0.0
            if self._idx >= len(self._segs):
                self._idx = 0
                self._dwell_left = float(self._wps[0].get('dwell', 0.0))
            else:
                self._dwell_left = float(self._wps[self._idx].get('dwell', 0.0))
        return self.pos.copy(), self.yaw


def default_waypoints():
    """
    One simulated laundry micro-cycle in base frame (m), tuned for the
    placeholder UR7e-like table (home TCP ~ (0.22, -0.14, 0.86)):
      1. home (start)
      2. reach over the incoming basket (left-front of the table)
      3. descend into the basket
      4. lift the "item"
      5. carry to the center of the table
      6. lower onto the table
      7. flatten: small circle over the item
      8. return home
    Replace with real station coordinates once the physical layout is built.
    """
    return [
        {'x': 0.22, 'y': -0.14, 'z': 0.86, 'dwell': 1.0},   # 1 home
        {'x': 0.10, 'y': -0.35, 'z': 0.70, 'dwell': 0.3},   # 2 over basket
        {'x': 0.10, 'y': -0.35, 'z': 0.45, 'dwell': 0.5},   # 3 in basket
        {'x': 0.10, 'y': -0.35, 'z': 0.75, 'dwell': 0.3},   # 4 lift item
        {'x': 0.30, 'y': -0.10, 'z': 0.80, 'dwell': 0.3},   # 5 carry to table
        {'x': 0.30, 'y': -0.10, 'z': 0.55, 'dwell': 0.4},   # 6 lower on table
        {'x': 0.36, 'y': -0.10, 'z': 0.56, 'dwell': 0.0},   # 7 flatten circle
        {'x': 0.30, 'y': -0.16, 'z': 0.56, 'dwell': 0.0},
        {'x': 0.24, 'y': -0.10, 'z': 0.56, 'dwell': 0.0},
        {'x': 0.30, 'y': -0.04, 'z': 0.56, 'dwell': 0.3},
        {'x': 0.22, 'y': -0.14, 'z': 0.86, 'dwell': 1.0},   # 8 home
    ]




def self_test():
    rate = 30.0
    pm = np.array([[0.0, 1.0, 0.0],
                   [-1.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0]])
    # round trip
    for p in [(0.22, -0.14, 0.86), (0.1, -0.35, 0.45)]:
        p = np.array(p)
        assert np.linalg.norm(pm @ base_to_quest(pm, p) - p) < 1e-9
    # continuity over 60 s
    pl = TrajectoryPlayer(default_waypoints(), rate)
    prev, max_step = None, 0.0
    for _ in range(int(60 * rate)):
        p, _ = pl.step()
        if prev is not None:
            max_step = max(max_step, float(np.linalg.norm(p - prev)))
        prev = p
    assert max_step < 0.012, max_step
    print(f"trajectory self-test: PASS  (60 s, max step {max_step*1000:.2f} mm, "
          f"{len(default_waypoints())} waypoints, loops)")


if __name__ == '__main__':
    self_test()
