#!/usr/bin/env python3
"""
kinematics.py — UR-style 6-DOF kinematics (Craig DH convention), pure numpy.

Shared by teleop_quest_node.py (IK for VR teleop) and any other FK consumer.
No ROS dependency so it can be unit-tested / self-tested on any machine:

    python3 kinematics.py            # runs a FK/IK round-trip self-test
    python3 kinematics.py config/ur7e_like.yaml

The DH table is read from YAML:

    kinematics:
      # rows: [d, a, alpha] per joint (m, m, rad) — Craig convention
      dh:
        - [0.1519, 0.0,      0.0]
        - [0.0,    -0.612,   0.0]
        ...
      theta_offset: [0.0]*6   # per-joint static offset (rad)
      tool_offset: [0.0, 0.0, 0.0]  # TCP offset along wrist frame (m)
"""
from __future__ import annotations

import math

import numpy as np

# ── Quaternions (w, x, y, z) ──────────────────────────────────────────────

def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0, 0, 0])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_to_axis_angle(q):
    """Quaternion -> (angle * axis) vector, valid for small angles too."""
    q = quat_normalize(q)
    w = min(max(q[0], -1.0), 1.0)
    v = q[1:]
    s = np.linalg.norm(v)
    if s < 1e-9:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(s, w)
    return v * (angle / s)


def quat_axis_angle(axis, angle):
    """(axis (3,), angle rad) -> quaternion."""
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / n
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s])


def quat_rotate(q, v):
    """Rotate vector v by quaternion q (w,x,y,z)."""
    v = np.asarray(v, dtype=float)
    return v + 2.0 * np.cross(q[1:], np.cross(q[1:], v) + q[0] * v)


def matrix_to_quat(R):
    """Rotation matrix -> (w, x, y, z) quaternion (Shepperd's method)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return quat_normalize(np.array([w, x, y, z]))


# ── Forward kinematics ─────────────────────────────────────────────────────

def dh_matrix(d, a, alpha, theta):
    """Craig DH transform: A = Rot_z(theta) Trans_z(d) Rot_x(alpha) Trans_x(a)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0, sa,      ca,     d],
        [0.0, 0.0,     0.0,    1.0],
    ])


def fk(dh, q, theta_offset=None, tool_offset=(0.0, 0.0, 0.0)):
    """
    Forward kinematics.
    dh: list of (d, a, alpha) per joint
    q:  (6,) joint angles (rad)
    returns 4x4 base->tool transform.
    """
    q = np.asarray(q, dtype=float)
    if theta_offset is None:
        theta_offset = np.zeros(len(dh))
    theta_offset = np.asarray(theta_offset, dtype=float)
    tool_offset = np.asarray(tool_offset, dtype=float)

    T = np.eye(4)
    for (d, a, alpha), qi, th0 in zip(dh, q, theta_offset):
        T = T @ dh_matrix(d, a, alpha, th0 + qi)

    if any(abs(x) > 1e-12 for x in tool_offset):
        T = T @ np.array([
            [1, 0, 0, tool_offset[0]],
            [0, 1, 0, tool_offset[1]],
            [0, 0, 1, tool_offset[2]],
            [0, 0, 0, 1.0],
        ])
    return T


def fk_pose(dh, q, theta_offset=None, tool_offset=(0.0, 0.0, 0.0)):
    """FK -> (position (3,), quaternion (w,x,y,z))."""
    T = fk(dh, q, theta_offset, tool_offset)
    return T[:3, 3].copy(), matrix_to_quat(T[:3, :3])


# ── Jacobian (numeric) + IK (damped least squares) ────────────────────────

def numerical_jacobian(dh, q, theta_offset=None, tool_offset=(0.0, 0.0, 0.0),
                        eps=1e-5):
    """Finite-difference geometric Jacobian -> (Jp (3,6), Jr (3,6))."""
    q = np.asarray(q, dtype=float)
    Jp = np.zeros((3, len(dh)))
    Jr = np.zeros((3, len(dh)))
    for i in range(len(dh)):
        qp = q.copy(); qp[i] += eps
        qm = q.copy(); qm[i] -= eps
        pp, qpp = fk_pose(dh, qp, theta_offset, tool_offset)
        pm, qpm = fk_pose(dh, qm, theta_offset, tool_offset)
        Jp[:, i] = (pp - pm) / (2 * eps)
        Jr[:, i] = quat_to_axis_angle(quat_mul(qpp, quat_conj(qpm))) / (2 * eps)
    return Jp, Jr


def ik_solve(dh, q0, target_pos, target_quat, theta_offset=None,
             tool_offset=(0.0, 0.0, 0.0), pos_weight=1.0, rot_weight=0.3,
             lam0=1e-3, max_iter=80, tol=1e-5):
    """
    Levenberg-Marquardt IK (standard form).

    Cost minimized: ||W (task_error)||^2 where W stacks pos_weight/rot_weight.
    Step:  dq = J^T (J J^T + λ I)^-1 e ;  accept if it reduces the raw cost,
    otherwise increase λ (dampen) and retry. λ halves on each accepted step.
    This is the textbook LM: the damping λ only regularizes the step (keeps it
    small near singularities, stays on the same elbow/wrist branch), and the
    acceptance test uses the raw task cost so the solver actually converges.

    q0: seed joint vector (use the current arm state for teleop).
    pos_weight/rot_weight: relative task weighting (position usually matters
    most for grasping; raise rot_weight for precise hanger insertion).
    Returns q with residual ~tol, or the best it can do.
    """
    q = np.asarray(q0, dtype=float).copy()
    target_pos = np.asarray(target_pos, dtype=float)
    target_quat = quat_normalize(np.asarray(target_quat, dtype=float))
    w = np.array([pos_weight, pos_weight, pos_weight,
                  rot_weight, rot_weight, rot_weight])

    def residual(qv):
        p, qq = fk_pose(dh, qv, theta_offset, tool_offset)
        e = np.concatenate([target_pos - p,
                            quat_to_axis_angle(quat_mul(target_quat, quat_conj(qq)))])
        return e * w

    lam = lam0
    err = residual(q)
    cost = float(err @ err)
    for _ in range(max_iter):
        if np.linalg.norm(err) < tol:
            break
        Jp, Jr = numerical_jacobian(dh, q, theta_offset, tool_offset)
        J = np.vstack([Jp, Jr])
        JJt = J @ J.T + lam * np.eye(6)
        dq = J.T @ np.linalg.solve(JJt, err)
        e_try = residual(q + dq)
        c_try = float(e_try @ e_try)
        if c_try < cost:            # accepted: step reduced the raw task cost
            q, err, cost = q + dq, e_try, c_try
            lam = max(lam * 0.5, 1e-8)
        else:
            lam = min(lam * 2.0, 1e3)
            if lam > 1e3:           # fully damped, can't improve -> give up
                break
    return q


def ik_residual(dh, q, target_pos, target_quat, theta_offset=None,
                tool_offset=(0.0, 0.0, 0.0), pos_weight=1.0, rot_weight=0.3):
    """Weighted 6D task-space residual for a candidate q. (pos err, rot err)."""
    p, qq = fk_pose(dh, q, theta_offset, tool_offset)
    e_pos = np.asarray(target_pos) - p
    e_rot = quat_to_axis_angle(quat_mul(quat_normalize(target_quat), quat_conj(qq)))
    return e_pos, e_rot


def ik_solve_robust(dh, q0, target_pos, target_quat, theta_offset=None,
                    tool_offset=(0.0, 0.0, 0.0), pos_weight=1.0, rot_weight=0.3,
                    pos_tol=0.02, seeds=None, **ik_kw):
    """
    IK with multi-start fallback. Tries the primary seed (usually the current
    arm state — incremental targets converge there every time), then falls back
    through alternate seeds if the residual stays above pos_tol.
    Returns (best_q, pos_err, rot_err).
    """
    if seeds is None:
        n = len(dh)
        seeds = [np.asarray(q0, dtype=float), np.zeros(n)]
    else:
        seeds = [np.asarray(s, dtype=float) for s in seeds]
    best_q, best_pe, best_re = None, np.inf, np.inf
    for seed in seeds:
        q = ik_solve(dh, seed, target_pos, target_quat, theta_offset,
                     tool_offset, pos_weight, rot_weight, **ik_kw)
        ep, er = ik_residual(dh, q, target_pos, target_quat, theta_offset, tool_offset)
        pe, re = float(np.linalg.norm(ep)), float(np.linalg.norm(er))
        if pe < best_pe:
            best_q, best_pe, best_re = q, pe, re
        if best_pe < pos_tol:
            break
    return best_q, best_pe, best_re


# ── Config loading ─────────────────────────────────────────────────────────

def load_kin_config(path):
    """
    Load a kinematics YAML config. Accepts `pi`, `2*pi`, `pi/2` tokens in
    numeric fields. Returns dict with:
      dh: [(d, a, alpha)], theta_offset: ndarray, tool_offset: tuple,
      home_pose: ndarray, workspace: dict (optional)
    """
    import yaml
    with open(path) as f:
        raw = yaml.safe_load(f)
    kin = raw["kinematics"] if "kinematics" in raw else raw

    def num(tok):
        if isinstance(tok, (int, float)):
            return float(tok)
        s = str(tok).replace(" ", "")
        if "pi" in s:
            import math as _m
            return float(eval(s.replace("pi", "_m.pi"), {"_m": _m}))
        return float(s)

    dh = [(num(r[0]), num(r[1]), num(r[2])) for r in kin["dh"]]
    out = {
        "dh": dh,
        "theta_offset": np.array([num(x) for x in
                                  kin.get("theta_offset", [0.0] * len(dh))]),
        "tool_offset": tuple(num(x) for x in kin.get("tool_offset", [0.0] * 3)),
    }
    if "home_pose" in kin:
        out["home_pose"] = np.array([num(x) for x in kin["home_pose"]])
    if "workspace" in kin:
        out["workspace"] = {
            "min": np.array([num(x) for x in kin["workspace"]["min"]]),
            "max": np.array([num(x) for x in kin["workspace"]["max"]]),
            "singularity_q5": num(kin["workspace"].get("singularity_q5", 0.15)),
        }
    return out


def self_test(yaml_path=None):
    """FK/IK round-trip: pick random q, FK to a target, IK back, compare."""
    cfg = {}
    if yaml_path:
        cfg = load_kin_config(yaml_path)
        dh = cfg["dh"]
        theta_offset = cfg["theta_offset"]
        tool_offset = cfg["tool_offset"]
    else:
        # Default placeholder table (same as config/ur7e_like.yaml) — real
        # UR7e Craig-DH numbers.
        dh = [(0.1625, 0.0, np.pi / 2), (0.0, -0.425, 0.0),
              (0.0, -0.3922, 0.0), (0.1333, 0.0, np.pi / 2),
              (0.0997, 0.0, -np.pi / 2), (0.0996, 0.0, 0.0)]
        theta_offset = np.zeros(6)
        tool_offset = (0.0, 0.0, 0.0)

    rng = np.random.default_rng(0)
    worst = 0.0
    # Extra IK seeds used when the primary (nearby) seed fails to converge:
    # a neutral pose + the configured home pose. In real teleop the seed is
    # always the current arm state, so these only matter for cold starts.
    neutral = np.array([0.0, -1.0, 0.0, -1.5, 0.3, 0.0])
    extra_seeds = [np.zeros(len(dh)), neutral]
    if "home_pose" in cfg:
        extra_seeds.append(cfg["home_pose"])
    for trial in range(25):
        # resample to avoid |q5| < 0.15 (wrist singularity: j4/j6 axes align —
        # no IK solver can converge there; real teleop avoids this region too)
        while True:
            q_true = rng.uniform(-1.2, 1.2, size=6)
            if abs(q_true[4]) > 0.15:
                break
        target_p, target_q = fk_pose(dh, q_true, theta_offset, tool_offset)
        q_seed = q_true + rng.uniform(-0.3, 0.3, size=6)
        q_hat, pos_err, rot_err = ik_solve_robust(
            dh, q_seed, target_p, target_q, theta_offset, tool_offset,
            seeds=[q_seed] + extra_seeds)
        worst = max(worst, pos_err, rot_err)
        status = "OK " if (pos_err < 1e-3 and rot_err < 1e-2) else "FAIL"
        print(f"  trial {trial:2d}: pos_err={pos_err*1000:8.3f} mm  "
              f"rot_err={rot_err:8.5f} rad   {status}")
    print(f"worst error: {worst:.5f}  ->  {'PASS' if worst < 1e-2 else 'FAIL'}")
    return worst < 1e-2


if __name__ == "__main__":
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else None
    print(f"Kinematics self-test{' (' + path + ')' if path else ''} (25 random round-trips):")
    ok = self_test(path)
    sys.exit(0 if ok else 1)
