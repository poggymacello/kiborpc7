#!/usr/bin/python3
# -*- coding:utf-8 -*-
"""
Kibo-RPC 7th (Int-Ball2) mission program -- team "Ganesha Orbit".

Layout of this single file (the official package rules forbid renaming the
package / launch file / CMakeLists, so everything lives in the launched script):

  1. CONFIG ............ every tunable number in one place
  2. Math .............. quaternion / vector helpers (x, y, z, w convention)
  3. Judge model ....... a line-by-line equivalent of rootfs/krpc/judge_module/judge.py
  4. Ctl model ......... timing model that reuses the formulas of the onboard
                         ctl_only profiler (pos_att_profiler.cpp, thrust_allocator.cpp)
  5. Planner ........... exact Held-Karp route search over every subset of checkpoints
  6. Mission ........... executor: move, gate the report on a judge-equivalent check,
                         guard the 5-minute limit, skip intelligently when late;
                         safe_stop() + run_mission_guarded(): every exit path (normal,
                         crash, deadline watchdog, /ib2_user/stop) ends with FAST off,
                         zero wrench and ctl KEEP_POSE (v3, 23 Oct 2026)
                         v4 (24 Oct 2026): per-leg split of the unchanged fan budget between
                         translation and rotation (FAST_ETA_ADAPT), a reference governor that
                         slows the profile clock when tracking lags (FAST_GOV), and an optional
                         camera (ArUco) final alignment for the ISS run (ARUCO_ALIGN, off)
  7. ROS adapter ....... rospy/actionlib bindings + the mandatory template contract
                         (/ib2_user/start, /ib2_user/stop, /ib2_user/ready)

Sections 1-6 import nothing from ROS, so tools/offline_sim.py can load this file
and run thousands of missions against an emulated ctl + judge on a laptop.

Compatibility: Python 3.6 (ROS Melodic python3 overlay). No f-string formatting
specifiers that need 3.8+, no dataclasses, no walrus operator. Standard library only.
"""

import json
import math
import threading
import time
import traceback

try:  # ROS is only present inside the simulator / on Int-Ball2.
    import rospy
    import actionlib
    from actionlib_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, WrenchStamped
    from platform_msgs.msg import UserLogic
    from platform_msgs.srv import StopProcessingUserNode, StopProcessingUserNodeResponse
    from krpc_ib2_msgs.srv import MissionStart, MissionFinished, CheckPoints
    from krpc_ib2_msgs.msg import CheckpointPassedAction, CheckpointPassedGoal
    from ib2_msgs.msg import CtlCommandAction, CtlCommandGoal, CtlCommandResult, CtlStatusType
    from ib2_msgs.msg import Navigation
    ROS_AVAILABLE = True
except ImportError:  # offline tools / unit tests
    ROS_AVAILABLE = False


# =============================================================================
# 1. CONFIG
# =============================================================================
CONFIG = {
    # --- mission timing ------------------------------------------------------
    "T_LIMIT_S": 300.0,          # Rulebook: 5 minutes from mission start
    "SAFETY_MARGIN_S": 15.0,     # planned finish must leave this much spare time
    "FINISH_GUARD_S": 3.0,       # mission_finished is forced before T_LIMIT - guard
    # --- checkpoint report gating ----------------------------------------------
    "HOLD_S": 1.0,               # Rulebook 2.3.1: stay inside the checkpoint for 1 s
    "POS_MARGIN": 0.60,          # report only when distance <= 0.60 x Lv3 radius
    "ANG_MARGIN": 0.60,          # ... and centering / roll <= 0.60 x Lv3 angle
    "FALLBACK_MARGIN": 0.85,     # margin used for the Lv2/Lv1 fallback report
    "WAIT_TIGHT_AFTER_SUCCESS_S": 2.5,
    "WAIT_SUCCESS_AFTER_REPORT_S": 4.0,
    "LOOP_DT": 0.05,
    # --- robustness ------------------------------------------------------------
    "MAX_GOAL_RETRY": 1,         # resend a goal the ctl aborted (collision/disturbance)
    "MAX_REFINE": 1,             # resend the same goal once if Lv3 is not reached
    "LEG_TIMEOUT_FACTOR": 1.6,
    "LEG_TIMEOUT_EXTRA_S": 12.0,
    "SERVICE_TIMEOUT_S": 5.0,
    "REPORT_TIMEOUT_S": 3.0,
    # --- planning model ----------------------------------------------------------
    "LEG_OVERHEAD_S": 1.0,       # communication + tracking lag per leg (calibrate from logs)
    "TRACK_LAG_S": 0.5,
    "SETTLE_S": 0.5,
    "ROTATE_FIRST_ENABLED": True,
    "ROTFIRST_MIN_GAIN_S": 1.5,
    "LEVEL_POINTS": {3: 3.0, 2: 2.0, 1: 1.0},   # assumption, only used to rank skip options
    "ALL_CHECKPOINTS_BONUS": 3.0,               # assumption for Pt_goal
    "MAG_MAX": 2.0,                              # Rulebook: time multiplier up to 2x
    # --- motion mode ---------------------------------------------------------------
    # "fast": our own parallel translation+rotation profile on /ctl/wrench (official API,
    #         see Programming Manual 6.2.2). Falls back to "safe" automatically on any fault.
    # "safe": onboard ctl via /ctl/command MOVE_TO_ABSOLUTE_TARGET only (JAXA's controller).
    "MOTION_MODE": "fast",
    "FAST_ETA_T": 0.55,          # share of fan capacity for translation
    "FAST_ETA_R": 0.40,          # share of fan capacity for rotation (runs in parallel)
    "FAST_VMAX": 0.20,           # same speed limit as the onboard profiler
    "FAST_WMAX": 0.436332312998582,
    "FAST_SAT": 0.95,            # scale wrench so no fan exceeds 95% of Fmax
    "FAST_RATE_HZ": 100.0,       # max controller update rate
    "FAST_FAULT_POS": 0.25,      # [m] tracking error that counts as a fault
    "FAST_FAULT_ATT_DEG": 25.0,
    "FAST_FAULT_TIME": 1.5,      # fault must persist this long
    "FAST_NAV_STALE_S": 0.5,
    "FAST_OVERHEAD_S": 0.4,
    "FAST_SETTLE_WAIT_S": 6.0,   # wait after profile end before refine / fallback report
    "FAST_MAX_REFINE": 1,        # Lv3 not held after settle wait -> re-run a short profile from here
    "FAST_MAX_DEGRADE": 2,       # on a fault: retry FAST with eta x FAST_DEGRADE_FACTOR (twice) before SAFE
    "FAST_DEGRADE_FACTOR": 0.7,
    "FAST_SOFT_POS": 0.18,       # leg finished but error exceeded these -> soften future profiles
    "FAST_SOFT_ATT_DEG": 18.0,
    "FAST_SOFT_FACTOR": 0.85,
    "FAST_ETA_T_MIN": 0.30,
    "FAST_ETA_R_MIN": 0.22,
    # v4: per-leg split of the same fan budget E = FAST_ETA_T + FAST_ETA_R (0.95, unchanged from
    # v1-v3). Most legs are translation-bound (rotation ends early), so each leg gives rotation only
    # the share it needs to finish by FAST_ROT_LEAD x the translation time and gives the rest to
    # translation. Peak fan demand is never above v3 (eta_t + eta_r <= E); speed limit unchanged.
    "FAST_ETA_ADAPT": True,
    "FAST_ETA_T_MAX_FRAC": 0.79,  # eta_t <= 0.79 E (0.75 at E = 0.95)
    "FAST_ETA_R_MIN_FRAC": 0.21,  # eta_r >= 0.21 E (0.20 at E = 0.95); eta_r never above FAST_ETA_R
    "FAST_ROT_LEAD": 0.9,         # rotation should end by 90% of the translation time
    # v4: reference governor. If the robot falls behind the profile (weaker fans, blocked flow) the
    # profile clock is slowed (speed x s, acceleration x s^2, s >= FAST_GOV_SMIN) until tracking is
    # back inside the LO band, instead of letting the error grow into a fault. Never active in
    # nominal runs (max tracking error there ~0.02 m / 1 deg).
    "FAST_GOV": True,
    "FAST_GOV_POS_HI": 0.10, "FAST_GOV_POS_LO": 0.06,       # [m]
    "FAST_GOV_ATT_HI": 8.0, "FAST_GOV_ATT_LO": 4.0,         # [deg]
    "FAST_GOV_RATE": 0.5,        # [1/s] how fast s moves
    "FAST_GOV_SMIN": 0.5,
    "FAST_GOV_SOFT_BELOW": 0.9,  # leg needed s below this -> soften later profiles (FAST_SOFT_FACTOR)
    "FAST_NO_RESPONSE_POS": 0.03,       # fault while the robot barely moved -> wrench not reaching
    "FAST_NO_RESPONSE_ATT_DEG": 3.0,    # the fans: go to SAFE at once instead of softening
    # v4: FAST starts only from (near) rest. If the robot is moving/turning at mission start, the
    # onboard ctl holds the pose (KEEP_POSE) first, for at most START_SETTLE_MAX_S.
    "START_REST_V": 0.03,        # [m/s]
    "START_REST_W_DEG": 5.0,     # [deg/s]
    "START_SETTLE_MAX_S": 12.0,
    # v2: attitude limit-cycle guard. If, after a profile ends, the Lv3 window is entered and
    # lost again >= FAST_OSC_ENTRIES times without a 1 s hold, the attitude loop is oscillating
    # (typically navigation delay). Then the attitude rate gain is scaled down for the rest of
    # the mission and one extra refine is allowed. Nominal runs never trigger it.
    "FAST_OSC_ENTRIES": 2,
    "FAST_OSC_GAIN_SCALE": 0.5,
    "FAST_OSC_GAIN_MIN": 0.25,
    # --- v3: protective stop (shared by normal finish, crash, deadline watchdog, /ib2_user/stop) --
    "SAFE_STOP_ZERO_N": 3,       # zero wrenches published after FAST is switched off ...
    "SAFE_STOP_ZERO_DT": 0.02,   # ... spaced so a wrench computed concurrently in the nav callback is overwritten
    "SAFE_STOP_CTL_WAIT_S": 1.0, # max wait for the ctl action server / KEEP_POSE result on the emergency path
    "DEADLINE_WATCHDOG_S": 2.0,  # independent watchdog forces safe stop + mission_finished at T_LIMIT - this
    # --- options -----------------------------------------------------------------
    "ENABLE_CAMERA_CHECK": False,  # log ArUco visibility at report time (diagnostic only)
    # v4: camera-based final alignment for the ISS run (OFF by default). The preliminary judge scores
    # the /sensor_fusion/navigation pose, so a camera correction cannot raise the simulator score.
    #   "off"     : no camera code runs (default; identical behaviour to v3 apart from the eta split)
    #   "log"     : at the end of each FAST leg, estimate the marker position from the camera and log
    #               the navigation-vs-marker offset; motion and reports unchanged
    #   "correct" : additionally shift the checkpoint (and marker) by that offset, re-run a short FAST
    #               profile, and gate the report on the shifted checkpoint (for the ISS, if JAXA
    #               judges by marker recognition; enable only after JAXA confirms)
    "ARUCO_ALIGN": "off",
    "ARUCO_MARKER_LEN_M": 0.02,          # Programming Manual 5.13.1: 2 cm, DICT_4X4, IDs 1-12
    "ARUCO_CAM_OFFSET_B": [0.14, 0.0, 0.0],  # Manual 5.13.2: camera 0.14 m ahead of the body origin, axis = +x
    "ARUCO_MIN_FRAMES": 3,               # consistent detections needed before an offset is trusted
    "ARUCO_MAX_SPREAD_M": 0.01,          # max spread of those detections
    "ARUCO_MAX_SHIFT_M": 0.05,           # offsets above this are treated as a bad detection and ignored
    "ARUCO_MIN_SHIFT_M": 0.01,           # below this no correction move is made
    "ARUCO_WAIT_S": 3.0,                 # max time spent collecting detections after the profile ends
    "VERBOSE": True,
}

# ctl_only parameters (Int-Ball2_platform_simulator/.../ctl_only/ctl.yaml, melodic branch)
CTL_PARAMS = {
    "mass": 3.216,
    "inertia": 0.0136,           # Is = diag(0.0136, 0.0136, 0.0136)
    "pos_fmax": 181.0032e-3,     # pos_profile/f_max
    "vmax": 0.20,                # pos_profile/v_max
    "x_thr": 0.05,               # pos_profile/x_threshold
    "eta": 0.4,                  # pos_profile/eta (also used for attitude)
    "eps_rm": 0.001,
    "att_tmax": 8.1904e-3,       # att_profile/t_max
    "wmax": 0.436332312998582,   # att_profile/w_max
    "q_thr": 0.017453292519943,  # att_profile/theta_threshold
    "eps_qm": 1.0e-9,
    "fan_fmax": 60.0e-3,         # fan/Fmax
    "tol_pos": 0.10,             # ctl/tolerance_pos
    "tol_att": 0.5,              # ctl/tolerance_att [rad]
    "duration_goal": 3.0,        # ctl/duration_goal
    # allocation matrices fan/Wp and fan/Wm: rows fan01..fan08, cols Fx Fy Fz Tx Ty Tz
    "Wp": [
        [0.0, 0.0, 0.0, 0.0, 0.0, 7.3257],
        [0.0, 0.6027, 0.0, 19.8198, 0.0, 0.0],
        [0.0, 0.6027, 0.4911, 0.0, 13.1935, 0.0],
        [0.0, 0.0, 0.4911, 19.8198, 13.1935, 7.3257],
        [0.3315, 0.0, 0.4911, 19.8198, 0.0, 0.0],
        [0.3315, 0.0, 0.0, 0.0, 13.1935, 0.0],
        [0.3315, 0.6027, 0.0, 19.8198, 13.1935, 7.3257],
        [0.3315, 0.6027, 0.4911, 0.0, 0.0, 7.3257],
    ],
    "Wm": [
        [0.3315, 0.6027, 0.4911, 19.8198, 13.1935, 0.0],
        [0.3315, 0.0, 0.4911, 0.0, 13.1935, 7.3257],
        [0.3315, 0.0, 0.0, 19.8198, 0.0, 7.3257],
        [0.3315, 0.6027, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.6027, 0.0, 0.0, 13.1935, 7.3257],
        [0.0, 0.6027, 0.4911, 19.8198, 0.0, 7.3257],
        [0.0, 0.0, 0.4911, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 19.8198, 13.1935, 0.0],
    ],
}


# =============================================================================
# 2. Math helpers (quaternions are [x, y, z, w], Hamilton product)
# =============================================================================
def v_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def v_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def v_scale(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


def v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def v_norm(a):
    return math.sqrt(v_dot(a, a))


def v_unit(a, fallback=None):
    n = v_norm(a)
    if n <= 1e-12:
        return list(fallback) if fallback is not None else [0.0, 0.0, 0.0]
    return [a[0] / n, a[1] / n, a[2] / n]


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def q_normalize(q):
    if q is None:
        return [0.0, 0.0, 0.0, 1.0]
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n <= 0.0:
        return [0.0, 0.0, 0.0, 1.0]
    return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]


def q_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def q_conj(q):
    return [-q[0], -q[1], -q[2], q[3]]


def q_rotate(q, v):
    """Rotate vector v by unit quaternion q (v' = q v q*)."""
    q = q_normalize(q)
    r = q_mul(q_mul(q, [v[0], v[1], v[2], 0.0]), q_conj(q))
    return [r[0], r[1], r[2]]


def q_angle(qa, qb):
    """Smallest rotation angle [rad] between two orientations."""
    qa = q_normalize(qa)
    qb = q_normalize(qb)
    d = abs(qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3])
    return 2.0 * math.acos(clamp(d, -1.0, 1.0))


def q_from_axis_angle(axis, ang):
    a = v_unit(axis, [1.0, 0.0, 0.0])
    s = math.sin(0.5 * ang)
    return [a[0] * s, a[1] * s, a[2] * s, math.cos(0.5 * ang)]


def q_integrate_body(q, w_body, dt):
    """q(t+dt) for body angular velocity w_body (rad/s)."""
    ang = v_norm(w_body) * dt
    if ang < 1e-12:
        return q_normalize(q)
    return q_normalize(q_mul(q, q_from_axis_angle(w_body, ang)))


def q_from_rotmat(m):
    """Rotation matrix (3x3, rows) -> quaternion [x,y,z,w] (Shepperd)."""
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return q_normalize([(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s])
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return q_normalize([0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s])
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return q_normalize([(m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s])
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return q_normalize([(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s])


def q_slerp(qa, qb, s):
    qa = q_normalize(qa)
    qb = q_normalize(qb)
    d = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3]
    if d < 0.0:
        qb = [-c for c in qb]
        d = -d
    if d > 0.9995:
        return q_normalize([qa[i] + s * (qb[i] - qa[i]) for i in range(4)])
    th = math.acos(clamp(d, -1.0, 1.0))
    sa = math.sin((1.0 - s) * th) / math.sin(th)
    sb = math.sin(s * th) / math.sin(th)
    return q_normalize([sa * qa[i] + sb * qb[i] for i in range(4)])


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


# =============================================================================
# 3. Judge model -- equivalent to rootfs/krpc/judge_module/judge.py
# =============================================================================
DEFAULT_POS_BANDS = [(0.20, 3), (0.40, 2), (0.60, 1)]
DEFAULT_ANG_BANDS = [(10.0, 3), (20.0, 2), (30.0, 1)]


def _bands(objs, key_names, default):
    out = []
    if isinstance(objs, list):
        for b in objs:
            if not isinstance(b, dict):
                continue
            thr = None
            for k in key_names:
                if k in b:
                    thr = b[k]
                    break
            lvl = b.get("level", b.get("score", b.get("point", b.get("points"))))
            try:
                out.append((float(thr), int(lvl)))
            except (TypeError, ValueError):
                continue
    out.sort(key=lambda x: x[0])
    return out if out else list(default)


def classify(value, bands):
    for thr, lvl in bands:
        if value <= thr:
            return lvl
    return 0


def band_threshold(bands, level):
    """Largest threshold that still yields `level` (bands sorted ascending)."""
    for thr, lvl in bands:
        if lvl == level:
            return thr
    return bands[0][0] if bands else 0.0


class CheckpointSpec(object):
    """One checkpoint exactly as delivered by /mission/get_checkpoints."""

    def __init__(self, item, goal_id=None):
        self.cid = int(item.get("checkpoint_id", item.get("checkpointid", item.get("id"))))
        self.position = [float(x) for x in item["position"][:3]]
        mp = item.get("marker_pose") or {}
        self.marker_position = [float(x) for x in (mp.get("position") or self.position)[:3]]
        self.marker_quaternion = q_normalize([float(x) for x in (mp.get("quaternion") or [0, 0, 0, 1])[:4]])
        self.pos_bands = _bands(item.get("score_bands") or item.get("bands"), ("radius", "margin", "distance"), DEFAULT_POS_BANDS)
        att = item.get("attitude") if isinstance(item.get("attitude"), dict) else {}
        q = att.get("quaternion")
        self.has_attitude = isinstance(q, list) and len(q) >= 4
        self.target_q = q_normalize([float(x) for x in q[:4]]) if self.has_attitude else self.look_at_quaternion(self.position)
        self.cen_bands = _bands(att.get("centering_bands"), ("angle", "margin", "threshold"), DEFAULT_ANG_BANDS)
        self.roll_bands = _bands(att.get("roll_bands"), ("angle", "margin", "threshold"), DEFAULT_ANG_BANDS)
        self.is_goal = goal_id is not None and self.cid == int(goal_id)
        self.max_level = max([lvl for _, lvl in self.pos_bands] or [3])

    def look_at_quaternion(self, from_pos):
        """Orientation whose body +x points at the marker (used only if no attitude given)."""
        x_axis = v_unit(v_sub(self.marker_position, from_pos), [1.0, 0.0, 0.0])
        up = [0.0, 0.0, 1.0] if abs(x_axis[2]) <= 0.95 else [0.0, 1.0, 0.0]
        z_axis = v_unit(v_cross(x_axis, up), [0.0, 0.0, 1.0])
        y_axis = v_unit(v_cross(z_axis, x_axis), [0.0, 1.0, 0.0])
        m = [[x_axis[0], y_axis[0], z_axis[0]], [x_axis[1], y_axis[1], z_axis[1]], [x_axis[2], y_axis[2], z_axis[2]]]
        return q_from_rotmat(m)


def twist_angle_deg(q, axis):
    qx, qy, qz, qw = q_normalize(q)
    if qw < 0.0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    d = qx * axis[0] + qy * axis[1] + qz * axis[2]
    return math.degrees(2.0 * math.atan2(d, qw))


def evaluate(spec, pos, q):
    """Return the judge's view of pose (pos, q) w.r.t. checkpoint `spec`."""
    dist = v_norm(v_sub(pos, spec.position))
    dist_lvl = classify(dist, spec.pos_bands)
    view_axis = v_unit(v_sub(spec.marker_position, pos))
    cam_fwd = v_unit(q_rotate(q, [1.0, 0.0, 0.0]))
    cen = math.degrees(math.acos(clamp(v_dot(cam_fwd, view_axis), -1.0, 1.0)))
    roll = abs(wrap_deg(twist_angle_deg(q, view_axis) - twist_angle_deg(spec.target_q, view_axis)))
    cands = [dist_lvl]
    cen_lvl = roll_lvl = None
    if spec.has_attitude:
        cen_lvl = classify(cen, spec.cen_bands)
        roll_lvl = classify(roll, spec.roll_bands)
        cands += [cen_lvl, roll_lvl]
    level = 0 if any(c == 0 for c in cands) else min(cands)
    return {"level": level, "dist": dist, "cen": cen, "roll": roll,
            "dist_lvl": dist_lvl, "cen_lvl": cen_lvl, "roll_lvl": roll_lvl}


def level_with_margin(spec, ev, margin):
    """Highest level L such that every metric is within margin x threshold(L)."""
    for lvl in (3, 2, 1):
        if ev["dist"] > margin * band_threshold(spec.pos_bands, lvl):
            continue
        if spec.has_attitude:
            if ev["cen"] > margin * band_threshold(spec.cen_bands, lvl):
                continue
            if ev["roll"] > margin * band_threshold(spec.roll_bands, lvl):
                continue
        return lvl
    return 0


def tight_ok(spec, ev, pos_margin, ang_margin, level=3):
    """True when every metric sits within margin x threshold(level)."""
    if ev["dist"] > pos_margin * band_threshold(spec.pos_bands, level):
        return False
    if spec.has_attitude:
        if ev["cen"] > ang_margin * band_threshold(spec.cen_bands, level):
            return False
        if ev["roll"] > ang_margin * band_threshold(spec.roll_bands, level):
            return False
    return True


def parse_checkpoints(message):
    """JSON string from /mission/get_checkpoints -> (list of CheckpointSpec, goal_id)."""
    payload = json.loads(message) if isinstance(message, str) else message
    items = payload.get("checkpoints", payload.get("Checkpoints", []))
    goal_id = payload.get("goalpoint_id")
    specs = []
    for item in items:
        if isinstance(item, dict) and item.get("position") is not None:
            specs.append(CheckpointSpec(item, goal_id))
    if goal_id is None and specs:  # defensive: generator always uses marker 12
        goal_id = 12 if any(s.cid == 12 for s in specs) else specs[-1].cid
        for s in specs:
            s.is_goal = (s.cid == goal_id)
    return specs, goal_id


# -----------------------------------------------------------------------------
# 3b. ArUco final alignment math (v4, used only when CONFIG["ARUCO_ALIGN"] != "off")
# -----------------------------------------------------------------------------
def cam_to_body(v_cam):
    """OpenCV camera frame (x right, y down, z forward) -> Int-Ball2 body (x fwd, y left, z up)."""
    return [v_cam[2], -v_cam[0], -v_cam[1]]


def marker_world_from_tvec(pos, q, tvec_cam, cam_offset_b):
    """Marker origin in the navigation frame from one estimatePoseSingleMarkers tvec."""
    return v_add(pos, q_rotate(q, v_add(cam_offset_b, cam_to_body(tvec_cam))))


def aruco_offset(obs_world, marker_world, cfg):
    """Mean observed marker position minus the mapped one, or None when the detections are too
    few, inconsistent (spread) or implausibly far from the map (bad detection)."""
    if len(obs_world) < cfg["ARUCO_MIN_FRAMES"]:
        return None
    n = float(len(obs_world))
    mean = [sum(o[i] for o in obs_world) / n for i in range(3)]
    if max(v_norm(v_sub(o, mean)) for o in obs_world) > cfg["ARUCO_MAX_SPREAD_M"]:
        return None
    off = v_sub(mean, marker_world)
    if v_norm(off) > cfg["ARUCO_MAX_SHIFT_M"]:
        return None
    return off


def shifted_spec(spec, off):
    """Copy of `spec` with checkpoint and marker moved by `off` (geometry relative to marker kept)."""
    s = CheckpointSpec.__new__(CheckpointSpec)
    s.__dict__.update(spec.__dict__)
    s.position = v_add(spec.position, off)
    s.marker_position = v_add(spec.marker_position, off)
    return s


# =============================================================================
# 4. Ctl timing model (same formulas as the onboard ctl_only profiler)
# =============================================================================
def maneuver_timing(r, v, a):
    """pos_att_profiler.cpp::maneuverTiming -> (tcb, tce, te)."""
    rnc = v * v / a
    tcb = v / a if r > rnc else math.sqrt(r / a)
    tce = r / v if r > rnc else tcb
    return tcb, tce, tcb + tce


def time_until_remaining(total, v, a, thr_remaining):
    """Time since profile start when the remaining distance first drops to thr."""
    if thr_remaining >= total:
        return 0.0
    tcb, tce, te = maneuver_timing(total, v, a)
    d_dec = 0.5 * a * tcb * tcb            # distance covered while decelerating
    if thr_remaining <= d_dec:
        return te - math.sqrt(2.0 * thr_remaining / a)
    if tce > tcb:                          # in the cruise phase
        return (total - thr_remaining) / v + 0.5 * tcb
    return math.sqrt(2.0 * (total - thr_remaining) / a)


class CtlModel(object):
    def __init__(self, params=None, cfg=None):
        self.p = dict(CTL_PARAMS)
        if params:
            self.p.update(params)
        self.cfg = dict(CONFIG)
        if cfg:
            self.cfg.update(cfg)
        self.Wp = self.p["Wp"]
        self.Wm = self.p["Wm"]

    # ---- thrust_allocator.cpp ------------------------------------------------
    def alloc(self, y):
        pp = [yi if yi > 0.0 else 0.0 for yi in y]
        pm = [-yi if yi < 0.0 else 0.0 for yi in y]
        f = []
        for i in range(8):
            s = 0.0
            for j in range(6):
                s += self.Wp[i][j] * pp[j] + self.Wm[i][j] * pm[j]
            f.append(s)
        mn = min(f)
        return [fi - mn for fi in f]

    def _effective_rate(self, y, eta):
        # allocation is positively homogeneous -> closed form of the Brent search on [0, 2]
        mx = max(self.alloc(y))
        if mx <= 1e-15:
            return 2.0
        return min(2.0, eta * self.p["fan_fmax"] / mx)

    def eff_acc(self, dir_body, eta=None):
        eta = self.p["eta"] if eta is None else eta
        f = self.p["pos_fmax"]
        d = v_unit(dir_body, [1.0, 0.0, 0.0])
        rate = self._effective_rate([d[0] * f, d[1] * f, d[2] * f, 0.0, 0.0, 0.0], eta)
        return f * rate / self.p["mass"]

    def eff_alpha(self, axis_body, eta=None):
        eta = self.p["eta"] if eta is None else eta
        t = self.p["att_tmax"]
        a = v_unit(axis_body, [1.0, 0.0, 0.0])
        rate = self._effective_rate([0.0, 0.0, 0.0, a[0] * t, a[1] * t, a[2] * t], eta)
        return t * rate / self.p["inertia"]

    # ---- one MOVE_TO_ABSOLUTE_TARGET goal ------------------------------------------
    def move(self, r0, q0, r1, q1, need_report=True, spec=None):
        """Predicted timing of one goal from pose (r0,q0) to (r1,q1).

        ctl_only executes MOVE_TO_ABSOLUTE_TARGET as SEQUENCE::POS_ATT:
        translate first (attitude held at q0), then rotate."""
        p, cfg = self.p, self.cfg
        q0 = q_normalize(q0)
        q1 = q_normalize(q1)
        dr_body = q_rotate(q_conj(q0), v_sub(r1, r0))
        xm = v_norm(dr_body)
        a = self.eff_acc(dr_body if xm > p["eps_rm"] else [1.0, 0.0, 0.0])
        tx = 0.0 if xm < p["x_thr"] else maneuver_timing(xm, p["vmax"], a)[2]
        dq = q_mul(q_conj(q0), q1)
        if dq[3] < 0.0:
            dq = [-c for c in dq]
        qm = 2.0 * math.acos(clamp(dq[3], -1.0, 1.0))
        axis = v_unit(dq[:3], [1.0, 0.0, 0.0]) if qm > p["eps_qm"] else [1.0, 0.0, 0.0]
        alpha = self.eff_alpha(axis)
        tq = 0.0 if qm < p["q_thr"] else maneuver_timing(qm, p["wmax"], alpha)[2]
        profile = tx + tq

        # ctl reachGoal(): |r-r1| < tol_pos and angle < tol_att held for duration_goal
        t_pos_in = 0.0 if xm < p["x_thr"] else time_until_remaining(xm, p["vmax"], a, p["tol_pos"])
        t_att_in = 0.0 if qm <= p["tol_att"] or tq == 0.0 else tx + time_until_remaining(qm, p["wmax"], alpha, p["tol_att"])
        stay_start = max(t_pos_in, t_att_in) + cfg["TRACK_LAG_S"]
        success_t = max(stay_start + p["duration_goal"], profile + cfg["SETTLE_S"])

        report_t = 0.0
        if need_report:
            r3 = cfg["POS_MARGIN"] * (band_threshold(spec.pos_bands, 3) if spec else 0.1)
            a3 = math.radians(cfg["ANG_MARGIN"] * (band_threshold(spec.roll_bands, 3) if spec else 10.0))
            t_pi = 0.0 if xm < p["x_thr"] else time_until_remaining(xm, p["vmax"], a, r3)
            t_ai = 0.0 if tq == 0.0 or qm <= a3 else tx + time_until_remaining(qm, p["wmax"], alpha, a3)
            report_t = max(t_pi, t_ai) + cfg["TRACK_LAG_S"] + cfg["HOLD_S"]
        end = max(success_t, report_t) + cfg["LEG_OVERHEAD_S"]
        return {"xm": xm, "qm": qm, "tx": tx, "tq": tq, "acc": a, "alpha": alpha,
                "profile": profile, "success": success_t, "report": report_t, "end": end}

    # ---- FAST mode (parallel profile on /ctl/wrench) -----------------------------
    def fast_acc(self, q0, q1, dh_world):
        """Translation acceleration that every attitude on the slerp path can deliver."""
        eta = self.cfg["FAST_ETA_T"]
        worst = None
        for k in range(5):
            qs = q_slerp(q0, q1, k / 4.0)
            a = self.eff_acc(q_rotate(q_conj(qs), dh_world), eta)
            worst = a if worst is None else min(worst, a)
        return worst

    def _rate_mx(self, y):
        """max fan force of the allocation of wrench y (the divisor inside _effective_rate)."""
        return max(self.alloc(y))

    def fast_split(self, q0, q1, dh_world, xm, qm, axis):
        """(acc, alpha, eta_t, eta_r) of one FAST leg. Without FAST_ETA_ADAPT this is exactly the
        v1-v3 fixed split. With it (v4) the fan budget E = FAST_ETA_T + FAST_ETA_R is split per leg
        so that max(t_trans, t_rot / FAST_ROT_LEAD) is smallest, with eta_r in
        [FAST_ETA_R_MIN_FRAC E, FAST_ETA_R] and eta_t <= min(E - eta_r, FAST_ETA_T_MAX_FRAC E)."""
        cfg, p = self.cfg, self.p
        if not cfg.get("FAST_ETA_ADAPT", False):
            a = self.fast_acc(q0, q1, dh_world) if xm > 1e-4 else 1.0
            alpha = self.eff_alpha(axis, cfg["FAST_ETA_R"])
            return a, alpha, cfg["FAST_ETA_T"], cfg["FAST_ETA_R"]
        E = cfg["FAST_ETA_T"] + cfg["FAST_ETA_R"]
        fmax = p["fan_fmax"]
        # eff_acc / eff_alpha are min(2, eta * fmax / mx) * scale: compute mx once, scale by eta
        f = p["pos_fmax"]
        mxa = 0.0
        if xm > 1e-4:
            for k in range(5):
                qs = q_slerp(q0, q1, k / 4.0)
                d = v_unit(q_rotate(q_conj(qs), dh_world), [1.0, 0.0, 0.0])
                mxa = max(mxa, self._rate_mx([d[0] * f, d[1] * f, d[2] * f, 0.0, 0.0, 0.0]))
        tm = p["att_tmax"]
        ax = v_unit(axis, [1.0, 0.0, 0.0])
        mxr = self._rate_mx([0.0, 0.0, 0.0, ax[0] * tm, ax[1] * tm, ax[2] * tm])

        def acc(eta):
            return f * (2.0 if mxa <= 1e-15 else min(2.0, eta * fmax / mxa)) / p["mass"]

        def alp(eta):
            return tm * (2.0 if mxr <= 1e-15 else min(2.0, eta * fmax / mxr)) / p["inertia"]

        r_lo = cfg["FAST_ETA_R_MIN_FRAC"] * E
        r_hi = max(r_lo, cfg["FAST_ETA_R"])
        t_cap = cfg["FAST_ETA_T_MAX_FRAC"] * E
        best = None
        n = 20
        for i in range(n + 1):
            er = r_lo + (r_hi - r_lo) * i / float(n)
            et = min(t_cap, E - er)
            tx = maneuver_timing(xm, cfg["FAST_VMAX"], acc(et))[2] if xm > 1e-4 else 0.0
            tq = maneuver_timing(qm, cfg["FAST_WMAX"], alp(er))[2] if qm > 1e-6 else 0.0
            cost = max(tx, tq / cfg["FAST_ROT_LEAD"])
            if best is None or cost < best[0] - 1e-9:
                best = (cost, et, er)
        et, er = best[1], best[2]
        return (acc(et) if xm > 1e-4 else 1.0), alp(er), et, er

    def fast_move(self, r0, q0, r1, q1):
        cfg = self.cfg
        q0 = q_normalize(q0)
        q1 = q_normalize(q1)
        dr = v_sub(r1, r0)
        xm = v_norm(dr)
        dh = v_unit(dr, [1.0, 0.0, 0.0])
        dq = q_mul(q_conj(q0), q1)
        if dq[3] < 0.0:
            dq = [-c for c in dq]
        qm = 2.0 * math.acos(clamp(dq[3], -1.0, 1.0))
        axis = v_unit(dq[:3], [1.0, 0.0, 0.0]) if qm > 1e-6 else [1.0, 0.0, 0.0]
        a, alpha, et, er = self.fast_split(q0, q1, dh, xm, qm, axis)
        tx = maneuver_timing(xm, cfg["FAST_VMAX"], a)[2] if xm > 1e-4 else 0.0
        tq = maneuver_timing(qm, cfg["FAST_WMAX"], alpha)[2] if qm > 1e-6 else 0.0
        return {"xm": xm, "qm": qm, "tx": tx, "tq": tq, "acc": a, "alpha": alpha, "T": max(tx, tq),
                "eta_t": et, "eta_r": er}

    def fast_leg(self, r0, q0, spec):
        """Predicted time until the report (tight band entered, then held HOLD_S)."""
        mv = self.fast_move(r0, q0, spec.position, spec.target_q)
        cfg = self.cfg
        r3 = cfg["POS_MARGIN"] * band_threshold(spec.pos_bands, 3)
        a3 = math.radians(cfg["ANG_MARGIN"] * band_threshold(spec.roll_bands, 3))
        t_p = 0.0 if mv["tx"] == 0.0 else time_until_remaining(mv["xm"], cfg["FAST_VMAX"], mv["acc"], r3)
        t_a = 0.0 if mv["tq"] == 0.0 else time_until_remaining(mv["qm"], cfg["FAST_WMAX"], mv["alpha"], a3)
        end = max(t_p, t_a) + cfg["TRACK_LAG_S"] + cfg["HOLD_S"] + cfg["FAST_OVERHEAD_S"]
        return (end, "fast", mv)

    def leg(self, r0, q0, spec):
        """Best way to reach `spec` from (r0, q0): one combined goal or rotate-first."""
        comb = self.move(r0, q0, spec.position, spec.target_q, True, spec)
        best = (comb["end"], "combined", comb)
        if self.cfg["ROTATE_FIRST_ENABLED"]:
            rot = self.move(r0, q0, r0, spec.target_q, False, spec)
            tra = self.move(r0, spec.target_q, spec.position, spec.target_q, True, spec)
            t_rf = rot["success"] + self.cfg["LEG_OVERHEAD_S"] + tra["end"]
            if t_rf + self.cfg["ROTFIRST_MIN_GAIN_S"] < comb["end"]:
                best = (t_rf, "rotate_first", tra)
        return best


# =============================================================================
# 4b. FAST mode: parallel translation + rotation profile and wrench controller
# =============================================================================
def profile_target(t, tcb, tce, te, thr, r, v, a):
    """Bang-coast-bang reference (same as pos_att_profiler.cpp::target): (x, v, a)."""
    if r < thr:
        return r, 0.0, 0.0
    if t < 0.0:
        return 0.0, 0.0, 0.0
    if t < tcb:
        vp = a * t
        return vp * t * 0.5, vp, a
    if t < tce:
        return v * (t - 0.5 * tcb), v, 0.0
    if t < te:
        tg = te - t
        vp = a * tg
        return r - vp * tg * 0.5, vp, -a
    return r, 0.0, 0.0


class FastProfile(object):
    """Translation along a straight line and single-axis rotation, executed in parallel."""

    def __init__(self, model, t0, r0, q0, r1, q1):
        cfg = model.cfg
        self.t0 = t0
        self.r0 = list(r0)
        self.q0 = q_normalize(q0)
        self.r1 = list(r1)
        self.q1 = q_normalize(q1)
        dr = v_sub(self.r1, self.r0)
        self.xm = v_norm(dr)
        self.dh = v_unit(dr, [1.0, 0.0, 0.0])
        self.v = cfg["FAST_VMAX"]
        dq = q_mul(q_conj(self.q0), self.q1)
        if dq[3] < 0.0:
            dq = [-c for c in dq]
        self.qm = 2.0 * math.acos(clamp(dq[3], -1.0, 1.0))
        self.axis = v_unit(dq[:3], [1.0, 0.0, 0.0]) if self.qm > 1e-6 else [1.0, 0.0, 0.0]
        # same split as CtlModel.fast_move, so the planner's prediction matches the flown profile
        a, alpha, self.eta_t, self.eta_r = model.fast_split(self.q0, self.q1, self.dh, self.xm, self.qm, self.axis)
        if self.xm > 1e-4:
            self.a = a
            self.xcb, self.xce, self.xtt = maneuver_timing(self.xm, self.v, self.a)
        else:
            self.a, self.xcb, self.xce, self.xtt = 1.0, 0.0, 0.0, 0.0
        self.w = cfg["FAST_WMAX"]
        if self.qm > 1e-6:
            self.alpha = alpha
            self.qcb, self.qce, self.qtt = maneuver_timing(self.qm, self.w, self.alpha)
        else:
            self.alpha, self.qcb, self.qce, self.qtt = 1.0, 0.0, 0.0, 0.0
        self.duration = max(self.xtt, self.qtt)

    def ref(self, t_abs, lag=0.0, s=1.0):
        """Reference at profile time t_abs - t0 - lag; speeds x s, accelerations x s^2 (governor)."""
        t = t_abs - self.t0 - lag
        if self.xm > 1e-4:
            xp, vp, ap = profile_target(t, self.xcb, self.xce, self.xtt, 0.0, self.xm, self.v, self.a)
        else:
            xp, vp, ap = self.xm, 0.0, 0.0
        r = [self.r0[i] + self.dh[i] * xp for i in range(3)]
        v = v_scale(self.dh, vp)
        a = v_scale(self.dh, ap)
        if self.qm > 1e-6:
            qp, wp, alp = profile_target(t, self.qcb, self.qce, self.qtt, 0.0, self.qm, self.w, self.alpha)
        else:
            qp, wp, alp = 0.0, 0.0, 0.0
        q = q_normalize(q_mul(self.q0, q_from_axis_angle(self.axis, qp))) if qp > 0.0 else self.q0
        if t >= self.qtt and self.qm > 1e-6:
            q = self.q1
        if t >= self.xtt:
            r = list(self.r1)
        if s != 1.0:
            v, a, wp, alp = v_scale(v, s), v_scale(a, s * s), wp * s, alp * s * s
        return r, v, a, q, v_scale(self.axis, wp), v_scale(self.axis, alp)


class FastController(object):
    """Same PD law and gains as ctl_only (pos_controller.cpp / att_controller.cpp)
    plus angular-acceleration feed-forward, fan-saturation guard and fault monitor."""

    KP_POS, KD_POS = 0.6219, 1.1152
    KP_ATT, KD_ATT = 2.6181, 3.2361

    def __init__(self, model):
        self.model = model
        self.cfg = model.cfg
        self.lock = threading.Lock()
        self.profile = None
        self.enabled = False
        self.fault = False
        self.fault_reason = ""
        self._bad_since = None
        self.max_pos_err = 0.0
        self.max_att_err = 0.0
        self.att_gain_scale = 1.0   # lowered by the limit-cycle guard (Mission._leg_fast)
        self.lag, self.s, self.min_s, self._t_last = 0.0, 1.0, 1.0, None   # v4 reference governor
        self._gov_r0, self._gov_q0, self._gov_live = None, None, False

    def set_profile(self, prof):
        with self.lock:
            self.profile = prof
            self._bad_since = None
            self.lag, self.s, self.min_s, self._t_last = 0.0, 1.0, 1.0, None
            self._gov_r0, self._gov_q0, self._gov_live = None, None, False

    def enable(self, on=True):
        with self.lock:
            self.enabled = bool(on)

    def active(self):
        return self.enabled and not self.fault and self.profile is not None

    def wrench(self, t, r, v, q, w):
        with self.lock:
            prof = self.profile
            en = self.enabled
        if not en or prof is None or self.fault:
            return None
        p, cfg = self.model.p, self.cfg
        rr, vr, ar, qr, wr, alr = prof.ref(t, self.lag, self.s)
        acc = [ar[i] - self.KP_POS * (r[i] - rr[i]) - self.KD_POS * (v[i] - vr[i]) for i in range(3)]
        force = v_scale(q_rotate(q_conj(q), acc), p["mass"])
        qe = q_mul(q_conj(qr), q)
        sgn = 1.0 if qe[3] >= 0.0 else -1.0
        qe_inv = q_conj(qe)
        wc = v_add(v_scale(qe[:3], -2.0 * self.KP_ATT / self.KD_ATT * sgn), q_rotate(qe_inv, wr))
        inertia = p["inertia"]
        torque = v_add(v_scale(v_sub(wc, w), self.KD_ATT * self.att_gain_scale * inertia), v_cross(w, v_scale(w, inertia)))
        torque = v_add(torque, v_scale(q_rotate(qe_inv, alr), inertia))
        mx = max(self.model.alloc(force + torque))
        lim = cfg["FAST_SAT"] * p["fan_fmax"]
        if mx > lim:
            s = lim / mx
            force = v_scale(force, s)
            torque = v_scale(torque, s)
        pos_err = v_norm(v_sub(r, rr))
        att_err = math.degrees(q_angle(q, qr))
        if cfg.get("FAST_GOV", False):
            # the governor only slows a robot that is responding: a robot that has not moved
            # FAST_NO_RESPONSE_POS / _ATT_DEG since the profile began must reach the fault quickly
            if self._gov_r0 is None:
                self._gov_r0, self._gov_q0 = list(r), list(q)
            elif not self._gov_live:
                self._gov_live = (v_norm(v_sub(r, self._gov_r0)) >= cfg["FAST_NO_RESPONSE_POS"] or
                                  math.degrees(q_angle(q, self._gov_q0)) >= cfg["FAST_NO_RESPONSE_ATT_DEG"])
            if self._t_last is not None:
                dt = clamp(t - self._t_last, 0.0, 0.1)
                self.lag += (1.0 - self.s) * dt
                if self._gov_live and (pos_err > cfg["FAST_GOV_POS_HI"] or att_err > cfg["FAST_GOV_ATT_HI"]):
                    self.s = max(cfg["FAST_GOV_SMIN"], self.s - cfg["FAST_GOV_RATE"] * dt)
                elif pos_err < cfg["FAST_GOV_POS_LO"] and att_err < cfg["FAST_GOV_ATT_LO"]:
                    self.s = min(1.0, self.s + cfg["FAST_GOV_RATE"] * dt)
                self.min_s = min(self.min_s, self.s)
            self._t_last = t
        self.max_pos_err = max(self.max_pos_err, pos_err)
        self.max_att_err = max(self.max_att_err, att_err)
        if pos_err > cfg["FAST_FAULT_POS"] or att_err > cfg["FAST_FAULT_ATT_DEG"]:
            if self._bad_since is None:
                self._bad_since = t
            elif t - self._bad_since > cfg["FAST_FAULT_TIME"]:
                self.fault = True
                self.fault_reason = "tracking error pos=%.3fm att=%.1fdeg" % (pos_err, att_err)
        else:
            self._bad_since = None
        return force, torque


# =============================================================================
# 5. Planner -- exact Held-Karp over all subsets, start pose fixed, goal last
# =============================================================================
class Planner(object):
    def __init__(self, model, cfg=None, leg_fn=None):
        self.model = model
        self.cfg = dict(CONFIG)
        if cfg:
            self.cfg.update(cfg)
        self.leg_fn = leg_fn or model.leg

    def mag(self, t_total):
        c = self.cfg
        return min(c["MAG_MAX"], 1.0 + max(0.0, c["T_LIMIT_S"] - t_total) / c["T_LIMIT_S"])

    def plan(self, start_r, start_q, nodes, goal, elapsed):
        """nodes: CheckpointSpec list (goal excluded). Returns dict with order/modes/time."""
        m, c = self.model, self.cfg
        n = len(nodes)
        INF = float("inf")
        c_sg, mode_sg, _ = self.leg_fn(start_r, start_q, goal)
        if n == 0:
            return {"order": [], "modes": [], "t_route": c_sg, "goal_mode": mode_sg,
                    "feasible": elapsed + c_sg <= c["T_LIMIT_S"] - c["FINISH_GUARD_S"], "dropped": []}
        c_s = [self.leg_fn(start_r, start_q, nodes[j]) for j in range(n)]
        c_m = [[None] * n for _ in range(n)]
        for i in range(n):
            for j in range(n):
                if i != j:
                    c_m[i][j] = self.leg_fn(nodes[i].position, nodes[i].target_q, nodes[j])
        c_g = [self.leg_fn(nodes[i].position, nodes[i].target_q, goal) for i in range(n)]

        size = 1 << n
        dp = [[INF] * n for _ in range(size)]
        par = [[-1] * n for _ in range(size)]
        for j in range(n):
            dp[1 << j][j] = c_s[j][0]
        for mask in range(1, size):
            row = dp[mask]
            for j in range(n):
                dj = row[j]
                if dj == INF:
                    continue
                cj = c_m[j]
                for k in range(n):
                    if mask & (1 << k):
                        continue
                    nm = mask | (1 << k)
                    val = dj + cj[k][0]
                    if val < dp[nm][k]:
                        dp[nm][k] = val
                        par[nm][k] = j
        # total time for every subset (route ends at goal)
        best_end = [INF] * size
        best_last = [-1] * size
        best_end[0] = c_sg
        for mask in range(1, size):
            row = dp[mask]
            for j in range(n):
                if row[j] < INF:
                    t = row[j] + c_g[j][0]
                    if t < best_end[mask]:
                        best_end[mask] = t
                        best_last[mask] = j

        budget = c["T_LIMIT_S"] - c["SAFETY_MARGIN_S"] - elapsed
        full = size - 1
        if best_end[full] <= budget:
            chosen = full
        else:
            pts = [c["LEVEL_POINTS"].get(nd.max_level, 3.0) for nd in nodes]
            goal_pts = c["LEVEL_POINTS"].get(goal.max_level, 3.0)
            chosen, best_val = None, -1.0
            for mask in range(size):
                t = best_end[mask]
                if t > budget:
                    continue
                s = goal_pts + sum(pts[j] for j in range(n) if mask & (1 << j))
                if mask == full:
                    s += c["ALL_CHECKPOINTS_BONUS"]
                val = s * self.mag(elapsed + t)
                if val > best_val + 1e-9 or (abs(val - best_val) <= 1e-9 and bin(mask).count("1") > bin(chosen or 0).count("1")):
                    chosen, best_val = mask, val
            if chosen is None:
                chosen = 0  # nothing fits: go straight to the goal
        # reconstruct
        order, modes = [], []
        if chosen:
            j = best_last[chosen]
            mask = chosen
            seq = []
            while j != -1:
                seq.append(j)
                pj = par[mask][j]
                mask &= ~(1 << j)
                j = pj
            seq.reverse()
            order = [nodes[j].cid for j in seq]
            prev = None
            for j in seq:
                modes.append(c_s[j][1] if prev is None else c_m[prev][j][1])
                prev = j
            goal_mode = c_g[seq[-1]][1]
        else:
            goal_mode = mode_sg
        dropped = [nodes[j].cid for j in range(n) if not (chosen & (1 << j))]
        return {"order": order, "modes": modes, "t_route": best_end[chosen], "goal_mode": goal_mode,
                "feasible": best_end[chosen] <= budget, "dropped": dropped, "t_full": best_end[full]}


# =============================================================================
# 6. Mission executor
# =============================================================================
def _api_log(api, level, text):
    try:
        api.log(level, text)
    except Exception:
        pass


def safe_stop(api, fast=None, reason="", cfg=None, ctl_wait_s=None, keep_pose=True):
    """Protective stop, the ONE routine used by every way a run can end (v3).

    Order (each step guarded on its own, so one failure never skips the next):
      1. FAST controller off -> the nav callback stops publishing /ctl/wrench
      2. zero wrench published SAFE_STOP_ZERO_N times, SAFE_STOP_ZERO_DT apart, so a wrench the
         nav callback computed just before step 1 is overwritten (only when a FAST controller
         exists: in pure SAFE mode we never touch /ctl/wrench and must not disturb ctl)
      3. onboard ctl KEEP_POSE at the current pose via /ctl/command (the standard ctl path);
         if that fails, cancel the running ctl goal instead. keep_pose=False (mission never
         started, so we never commanded the robot) skips this step: a stop before the start
         must not wake up the fans of a robot we have not moved.
    Callers decide whether mission_finished comes before (healthy finish: time-critical) or
    after (crash / watchdog: stop the robot first). Returns a dict describing what was done.
    """
    cfg = cfg or CONFIG
    if fast is None:
        fast = getattr(api, "_fast", None)
    out = {"fast_off": False, "zero_wrench": 0, "keep_pose": False, "keep_pose_sent": False, "cancel": False}
    n = int(cfg.get("SAFE_STOP_ZERO_N", 3)) if fast is not None else 0
    _api_log(api, "warn", "SAFE STOP (%s): FAST off -> zero wrench x%d -> %s" % (
        reason, n, "ctl KEEP_POSE" if keep_pose else "no ctl command (mission not started)"))
    if fast is not None:
        try:
            fast.enable(False)
            out["fast_off"] = True
        except Exception as exc:
            _api_log(api, "error", "safe stop: FAST disable failed: %s" % exc)
    if n > 0 and hasattr(api, "publish_zero_wrench"):
        for i in range(n):
            try:
                api.publish_zero_wrench()
                out["zero_wrench"] += 1
            except Exception as exc:
                _api_log(api, "error", "safe stop: zero wrench failed: %s" % exc)
            if i < n - 1:
                try:
                    api.sleep(cfg.get("SAFE_STOP_ZERO_DT", 0.02))
                except Exception:
                    pass
    if not keep_pose:
        return out
    if hasattr(api, "ctl_keep_pose"):
        try:
            # ctl_keep_pose: True = confirmed, False = sent but not confirmed, None = not sent
            if ctl_wait_s is None:
                res = api.ctl_keep_pose()
            else:
                res = api.ctl_keep_pose(server_s=ctl_wait_s, result_s=ctl_wait_s)
            out["keep_pose"] = res is True
            out["keep_pose_sent"] = res is not None
        except Exception as exc:
            _api_log(api, "error", "safe stop: ctl KEEP_POSE failed: %s" % exc)
    # a KEEP_POSE goal that was sent must not be cancelled; cancel only when none reached ctl
    if not out["keep_pose_sent"] and hasattr(api, "cancel_goal"):
        try:
            api.cancel_goal()
            out["cancel"] = True
        except Exception as exc:
            _api_log(api, "error", "safe stop: cancel_goal failed: %s" % exc)
    if not out["keep_pose"]:
        _api_log(api, "warn", "safe stop: ctl did not confirm KEEP_POSE (cancel sent=%s)" % out["cancel"])
    return out


class Mission(object):
    """Runs one mission against an `api` object (ROS adapter or offline simulator).

    Required api methods: now(), sleep(dt), nav() -> (pos, quat) | None,
    send_goal(pos, quat), goal_state() -> str, cancel_goal(), report_checkpoint(cid) -> bool,
    mission_start() -> bool, mission_finished() -> bool, get_checkpoints() -> str|None,
    stop_requested() -> bool, log(level, text).
    """

    TERMINAL_FAIL = ("PREEMPTED", "ABORTED", "REJECTED", "LOST", "RECALLED", "FAILED")

    def __init__(self, api, cfg=None, params=None):
        self.api = api
        self.cfg = dict(CONFIG)
        if cfg:
            self.cfg.update(cfg)
        self.model = CtlModel(params, self.cfg)
        self.mode = self.cfg["MOTION_MODE"]
        self.fast = None
        self.fast_degrades = 0
        self.fast_leg_start = None
        if self.mode == "fast" and hasattr(api, "attach_fast_controller"):
            self.fast = FastController(self.model)
            api.attach_fast_controller(self.fast)
        else:
            self.mode = "safe"
        self.planner = Planner(self.model, self.cfg,
                               self.model.fast_leg if self.mode == "fast" else self.model.leg)
        self.t0 = None
        self.results = {}       # cid -> dict(level, t, ...)
        self.legs = []          # diagnostics: predicted vs actual
        self.finished = False   # mission_finished has been sent
        self.finishing = False  # a thread is sending mission_finished right now
        self._finish_lock = threading.Lock()

    # ---- helpers ------------------------------------------------------------
    def log(self, text, level="info"):
        self.api.log(level, text)

    def elapsed(self):
        if self.t0 is None:   # crash before mission_start returned
            return 0.0
        return self.api.now() - self.t0

    def pose(self):
        nv = self.api.nav()
        tries = 0
        while nv is None and tries < 40 and not self.api.stop_requested():
            self.api.sleep(0.05)
            nv = self.api.nav()
            tries += 1
        return nv

    def time_left(self):
        return self.cfg["T_LIMIT_S"] - self.elapsed()

    def goal_eta_from(self, pos, q, goal):
        return self.planner.leg_fn(pos, q, goal)[0]

    # ---- main -----------------------------------------------------------------
    def run(self):
        api, cfg = self.api, self.cfg
        if not api.mission_start():
            self.log("mission_start failed; continuing anyway", "warn")
        self.t0 = api.now()
        specs, goal_id = self._load_checkpoints()
        if not specs:
            self.log("no checkpoint data -> finishing immediately", "error")
            self._finish()
            return self.summary()
        by_id = dict((s.cid, s) for s in specs)
        goal = by_id.get(goal_id)
        remaining = [s.cid for s in specs if s.cid != goal_id]
        self.log("checkpoints: %s goal=%s mode=%s" % (sorted(by_id.keys()), goal_id, self.mode))
        if self.mode == "fast":
            # make sure the onboard ctl is silent (STAND_BY) before we publish /ctl/wrench
            if hasattr(api, "ctl_standby") and not api.ctl_standby():
                self.log("ctl did not confirm STAND_BY; continuing (fault monitor will catch conflicts)", "warn")
            nf = self.api.nav_full()
            if nf is not None and not self._at_rest(nf) and hasattr(api, "ctl_keep_pose"):
                nf = self._settle_before_fast(nf)
            if nf is None:
                self._switch_to_safe("no navigation for FAST start")
            else:  # hold the current pose, then take control
                self.fast.set_profile(FastProfile(self.model, api.now(), nf[0], nf[1], nf[0], nf[1]))
                self.fast.enable(True)

        while remaining and not api.stop_requested():
            nv = self.pose()
            if nv is None:
                break
            pos, q = nv
            plan = self.planner.plan(pos, q, [by_id[c] for c in remaining], goal, self.elapsed())
            if cfg["VERBOSE"]:
                self.log("plan t=%.1fs order=%s modes=%s route=%.1fs full=%.1fs dropped=%s" % (
                    self.elapsed(), plan["order"], plan["modes"], plan["t_route"], plan.get("t_full", plan["t_route"]), plan["dropped"]))
            if not plan["order"]:
                break
            cid = plan["order"][0]
            self._do_leg(by_id[cid], plan["modes"][0], goal)
            remaining.remove(cid)
            if self._must_go_to_goal(goal):
                self.log("deadline guard -> heading to goal", "warn")
                break

        if goal is not None and not api.stop_requested():
            nv = self.pose()
            mode = "combined"
            if nv is not None:
                mode = self.planner.leg_fn(nv[0], nv[1], goal)[1]
            self._do_leg(goal, mode, goal, is_goal=True)
        self._finish()
        return self.summary()

    def _at_rest(self, nf):
        return (nf is not None and v_norm(nf[2]) <= self.cfg["START_REST_V"] and
                math.degrees(v_norm(nf[3])) <= self.cfg["START_REST_W_DEG"])

    def _settle_before_fast(self, nf):
        """v4: never start the aggressive FAST profile on a drifting / spinning robot: the onboard
        ctl (KEEP_POSE) brings it to rest first (max START_SETTLE_MAX_S), then STAND_BY again.
        Called only when the robot is not at rest, so a normal start is unchanged."""
        api, cfg = self.api, self.cfg
        self.log("robot moving at start (v=%.3fm/s w=%.1fdeg/s) -> ctl KEEP_POSE until at rest" % (
            v_norm(nf[2]), math.degrees(v_norm(nf[3]))), "warn")
        api.ctl_keep_pose(None, 0.5)
        t_end = api.now() + cfg["START_SETTLE_MAX_S"]
        while api.now() < t_end and not api.stop_requested():
            api.sleep(cfg["LOOP_DT"])
            if self._at_rest(api.nav_full()):
                break
        self.log("start settle done after %.1fs" % self.elapsed())
        if hasattr(api, "ctl_standby") and not api.ctl_standby():
            self.log("ctl did not confirm STAND_BY after settle; continuing", "warn")
        return api.nav_full()

    def _switch_to_safe(self, why):
        self.log("switching to SAFE mode (onboard ctl): %s" % why, "warn")
        if self.fast is not None:
            self.fast.enable(False)
        if hasattr(self.api, "publish_zero_wrench"):
            self.api.publish_zero_wrench()
        self.mode = "safe"
        self.planner.leg_fn = self.model.leg

    def _soften_fast(self, factor, why):
        c = self.model.cfg
        c["FAST_ETA_T"] = max(c["FAST_ETA_T_MIN"], c["FAST_ETA_T"] * factor)
        c["FAST_ETA_R"] = max(c["FAST_ETA_R_MIN"], c["FAST_ETA_R"] * factor)
        self.log("FAST profile softened (%s): eta_t=%.2f eta_r=%.2f" % (why, c["FAST_ETA_T"], c["FAST_ETA_R"]), "warn")

    def _do_leg(self, spec, mode, goal, is_goal=False):
        if mode == "fast" and self.mode == "fast" and self.fast is not None:
            while True:
                self.fast.max_pos_err = 0.0
                self.fast.max_att_err = 0.0
                status = self._leg_fast(spec, goal, is_goal)
                if status != "FAULT":
                    if (self.fast.max_pos_err > self.cfg["FAST_SOFT_POS"] or self.fast.max_att_err > self.cfg["FAST_SOFT_ATT_DEG"]
                            or self.fast.min_s < self.cfg.get("FAST_GOV_SOFT_BELOW", 0.0)):
                        self._soften_fast(self.cfg["FAST_SOFT_FACTOR"], "leg error pos=%.2fm att=%.0fdeg governor s=%.2f" % (
                            self.fast.max_pos_err, self.fast.max_att_err, self.fast.min_s))
                    return
                if self.fast_degrades >= self.cfg["FAST_MAX_DEGRADE"] or spec.cid in self.results:
                    break
                if self._fast_no_response():
                    self.fast.fault_reason += " (robot did not respond to /ctl/wrench)"
                    break
                self.fast_degrades += 1
                self._soften_fast(self.cfg["FAST_DEGRADE_FACTOR"], "fault: " + self.fast.fault_reason)
                self.fast.fault = False
                self.fast.fault_reason = ""
                self.fast._bad_since = None
            if spec.cid in self.results:
                if self.fast.fault:
                    self._switch_to_safe(self.fast.fault_reason or "fast leg fault")
                return
            self._switch_to_safe(self.fast.fault_reason or "fast leg fault")
            if spec.cid in self.results:
                return
            nv = self.pose()
            mode = self.model.leg(nv[0], nv[1], spec)[1] if nv is not None else "combined"
        if mode == "fast":
            mode = "combined"
        self._leg(spec, mode, goal, is_goal)

    def _leg_fast(self, spec, goal, is_goal=False):
        """One leg in FAST mode: no ctl involvement, report on our judge-equivalent check."""
        api, cfg = self.api, self.cfg
        nf = api.nav_full()
        if nf is None:
            return "FAULT"
        r0, q0 = nf[0], nf[1]
        self.fast_leg_start = (list(r0), list(q0))
        prof = FastProfile(self.model, api.now(), r0, q0, spec.position, spec.target_q)
        self.fast.set_profile(prof)
        pred = self.model.fast_leg(r0, q0, spec)
        t_leg0 = api.now()
        self.log("leg -> cp %d (fast) profile %.1fs predicted %.1fs, elapsed %.1fs, eta_t=%.2f eta_r=%.2f" % (
            spec.cid, prof.duration, pred[0], self.elapsed(), prof.eta_t, prof.eta_r))
        hold_since = {3: None, 2: None, 1: None}
        reported = False
        status = "OK"
        refines, t_prof0 = 0, t_leg0
        max_refine = cfg["FAST_MAX_REFINE"]
        osc_entries, was_tight = 0, False
        nv = None
        # v4 optional camera alignment (CONFIG["ARUCO_ALIGN"]); "off" leaves this leg exactly as in v3
        align = cfg.get("ARUCO_ALIGN", "off") if hasattr(api, "aruco_observe") else "off"
        tgt = spec                      # checkpoint the report is gated on (shifted only in "correct")
        a_obs, a_t0, a_done = [], None, align == "off"
        while not api.stop_requested():
            now = api.now()
            if self.fast.fault:
                status = "FAULT"
                break
            if hasattr(api, "nav_age") and api.nav_age() > cfg["FAST_NAV_STALE_S"]:
                self.fast.fault = True
                self.fast.fault_reason = "navigation stale"
                status = "FAULT"
                break
            nv = api.nav()
            if nv is not None:
                ev = evaluate(tgt, nv[0], nv[1])
                tight = tight_ok(tgt, ev, cfg["POS_MARGIN"], cfg["ANG_MARGIN"], 3)
                if not a_done and (tight or now - t_prof0 - self.fast.lag > prof.duration):
                    a_t0 = a_t0 or now
                    tv = api.aruco_observe(spec.cid)
                    if tv is not None:
                        a_obs.append(marker_world_from_tvec(nv[0], nv[1], tv, cfg["ARUCO_CAM_OFFSET_B"]))
                    off = aruco_offset(a_obs, spec.marker_position, cfg)
                    if off is not None or now - a_t0 > cfg["ARUCO_WAIT_S"]:
                        a_done = True
                        self.log("cp %d: aruco offset %s from %d detections" % (
                            spec.cid, "none" if off is None else "[%.3f, %.3f, %.3f] m" % tuple(off), len(a_obs)))
                        if align == "correct" and off is not None and v_norm(off) >= cfg["ARUCO_MIN_SHIFT_M"]:
                            tgt = shifted_spec(spec, off)
                            prof = FastProfile(self.model, now, nv[0], nv[1], tgt.position, tgt.target_q)
                            self.fast.set_profile(prof)
                            t_prof0 = now
                            hold_since = {3: None, 2: None, 1: None}
                            api.sleep(cfg["LOOP_DT"])
                            continue
                loose = level_with_margin(tgt, ev, cfg["FALLBACK_MARGIN"])
                hold_since[3] = (hold_since[3] or now) if tight else None
                for lvl in (2, 1):
                    hold_since[lvl] = (hold_since[lvl] or now) if loose >= lvl else None
                if (hold_since[3] is not None and now - hold_since[3] >= cfg["HOLD_S"]
                        and (a_done or align != "correct")):
                    reported = self._report(tgt, ev, "Lv3 held (fast)")
                    break
                if tight and not was_tight and now - t_prof0 - self.fast.lag > prof.duration:
                    osc_entries += 1   # entered the Lv3 window again after the profile ended
                was_tight = tight
            if now - t_prof0 - self.fast.lag > prof.duration + cfg["FAST_SETTLE_WAIT_S"]:
                if (osc_entries >= cfg["FAST_OSC_ENTRIES"] and
                        self.fast.att_gain_scale * cfg["FAST_OSC_GAIN_SCALE"] >= cfg["FAST_OSC_GAIN_MIN"] - 1e-9):
                    self.fast.att_gain_scale *= cfg["FAST_OSC_GAIN_SCALE"]
                    max_refine += 1
                    self.log("cp %d: attitude limit cycle (%d Lv3 entries, no 1 s hold) -> attitude gain x%.2f" % (
                        spec.cid, osc_entries, self.fast.att_gain_scale), "warn")
                osc_entries = 0
                if refines < max_refine and nv is not None and self._fast_refine_affordable(nv, tgt, goal, is_goal):
                    refines += 1
                    prof = FastProfile(self.model, now, nv[0], nv[1], tgt.position, tgt.target_q)
                    self.fast.set_profile(prof)
                    t_prof0 = now
                    self.log("cp %d: Lv3 not held after profile end -> FAST refine (%d), profile %.1fs" % (
                        spec.cid, refines, prof.duration), "warn")
                    api.sleep(cfg["LOOP_DT"])
                    continue
                self.log("cp %d: Lv3 not held %.1fs after profile end" % (spec.cid, cfg["FAST_SETTLE_WAIT_S"]), "warn")
                reported = self._report_fallback(tgt, hold_since, nv)
                break
            if is_goal:
                if self.elapsed() > cfg["T_LIMIT_S"] - cfg["FINISH_GUARD_S"] - 1.0:
                    if not reported:
                        self._report_fallback(tgt, hold_since, nv, allow_no_hold=True)
                    break
            elif goal is not None and nv is not None:
                need = self.goal_eta_from(nv[0], nv[1], goal)
                if self.elapsed() + need + 2.0 > cfg["T_LIMIT_S"] - cfg["FINISH_GUARD_S"]:
                    self.log("abandoning cp %d to protect the goal" % spec.cid, "warn")
                    if not reported:
                        self._report_fallback(tgt, hold_since, nv, allow_no_hold=True)
                    break
            api.sleep(cfg["LOOP_DT"])
        self.legs.append({"cid": spec.cid, "mode": "fast", "pred": pred[0], "actual": api.now() - t_leg0,
                          "level": self.results.get(spec.cid, {}).get("level", 0)})
        return status

    def _fast_no_response(self):
        """True when a FAST leg faulted although the robot hardly moved or turned since it began."""
        nv = self.api.nav()
        if nv is None or self.fast_leg_start is None:
            return False
        r0, q0 = self.fast_leg_start
        moved = v_norm(v_sub(nv[0], r0))
        turned = math.degrees(q_angle(nv[1], q0))
        return moved < self.cfg["FAST_NO_RESPONSE_POS"] and turned < self.cfg["FAST_NO_RESPONSE_ATT_DEG"]

    def _fast_refine_affordable(self, nv, spec, goal, is_goal):
        """A refine costs a short profile + settle wait + hold; only if the goal stays safe."""
        cfg = self.cfg
        prof = FastProfile(self.model, 0.0, nv[0], nv[1], spec.position, spec.target_q)
        cost = prof.duration + cfg["FAST_SETTLE_WAIT_S"] + cfg["HOLD_S"]
        if is_goal or goal is None:
            return self.elapsed() + cost < cfg["T_LIMIT_S"] - cfg["FINISH_GUARD_S"] - cfg["SAFETY_MARGIN_S"]
        need = self.goal_eta_from(spec.position, spec.target_q, goal)
        return self.elapsed() + cost + need + cfg["SAFETY_MARGIN_S"] < cfg["T_LIMIT_S"]

    def _load_checkpoints(self):
        for attempt in range(3):
            msg = self.api.get_checkpoints()
            if msg:
                try:
                    specs, goal_id = parse_checkpoints(msg)
                    if specs:
                        return specs, goal_id
                except Exception as exc:  # malformed payload
                    self.log("checkpoint parse error: %s" % exc, "error")
            self.api.sleep(0.5)
        return [], None

    def _must_go_to_goal(self, goal):
        if goal is None:
            return False
        nv = self.api.nav()
        if nv is None:
            return False
        need = self.goal_eta_from(nv[0], nv[1], goal)
        return self.elapsed() + need + self.cfg["SAFETY_MARGIN_S"] * 0.5 > self.cfg["T_LIMIT_S"]

    def _claim_finish(self):
        """Exactly one caller (mission thread, crash handler or watchdog) may send mission_finished."""
        with self._finish_lock:
            if self.finished or self.finishing:
                return False
            self.finishing = True
            return True

    def _send_finished(self):
        ok = False
        try:
            ok = self.api.mission_finished()
        except Exception as exc:
            self.log("mission_finished failed: %s" % exc, "error")
        finally:
            self.finished = True
            self.finishing = False
        self.log("mission_finished sent at t=%.2fs (ok=%s)" % (self.elapsed(), ok))
        return ok

    def safe_stop(self, reason, ctl_wait_s=None):
        # KEEP_POSE only once mission_start went through (t0 set): before that we never moved the robot
        return safe_stop(self.api, self.fast, reason, self.cfg, ctl_wait_s, keep_pose=self.t0 is not None)

    def _finish(self):
        """Healthy end (all legs done or deadline guard): mission_finished FIRST because it is
        time-critical, then the shared protective stop hands the robot to the onboard ctl."""
        if not self._claim_finish():
            return
        try:
            self._send_finished()
        finally:
            self.safe_stop("mission finished")

    def emergency_finish(self, reason):
        """Crash / watchdog path: protective stop FIRST (robot must not keep flying on a
        stale wrench controller), then mission_finished. Never raises."""
        if self.finished:
            return False
        claimed = self._claim_finish()
        try:
            # also when another thread is still inside mission_finished (it may be hung there)
            self.safe_stop(reason, self.cfg["SAFE_STOP_CTL_WAIT_S"])
        except Exception as exc:
            self.log("safe stop raised: %s" % exc, "error")
        finally:
            if claimed:
                self._send_finished()
        return claimed

    # ---- one leg ----------------------------------------------------------------
    def _send(self, pos, q):
        if self.finished or self.finishing or self.api.stop_requested():
            return  # never command a new move after a stop / finish
        self.api.send_goal(list(pos), q_normalize(q))

    def _wait_success(self, timeout):
        t_end = self.api.now() + timeout
        while self.api.now() < t_end and not self.api.stop_requested():
            st = self.api.goal_state()
            if st == "SUCCEEDED":
                return True
            if st in self.TERMINAL_FAIL:
                return False
            self.api.sleep(self.cfg["LOOP_DT"])
        return False

    def _leg(self, spec, mode, goal, is_goal=False):
        api, cfg = self.api, self.cfg
        nv = self.pose()
        if nv is None:
            return False
        pos0, q0 = nv
        pred = self.model.leg(pos0, q0, spec)
        t_leg0 = api.now()
        self.log("leg -> cp %d (%s) predicted %.1fs, elapsed %.1fs" % (spec.cid, mode, pred[0], self.elapsed()))

        if mode == "rotate_first":
            rot = self.model.move(pos0, q0, pos0, spec.target_q, False, spec)
            self._send(pos0, spec.target_q)
            if not self._wait_success(rot["success"] * cfg["LEG_TIMEOUT_FACTOR"] + cfg["LEG_TIMEOUT_EXTRA_S"]):
                self.log("rotate-first step did not succeed; continuing with combined goal", "warn")
        self._send(spec.position, spec.target_q)
        timeout = pred[0] * cfg["LEG_TIMEOUT_FACTOR"] + cfg["LEG_TIMEOUT_EXTRA_S"]

        hold_since = {3: None, 2: None, 1: None}
        reported, success_at, report_at = False, None, None
        retries, refines = 0, 0
        best = {"level": 0}
        while not api.stop_requested():
            now = api.now()
            nv = api.nav()
            if nv is not None:
                ev = evaluate(spec, nv[0], nv[1])
                if ev["level"] > best["level"]:
                    best = ev
                tight = tight_ok(spec, ev, cfg["POS_MARGIN"], cfg["ANG_MARGIN"], 3)
                loose = level_with_margin(spec, ev, cfg["FALLBACK_MARGIN"])
                hold_since[3] = (hold_since[3] or now) if tight else None
                for lvl in (2, 1):
                    hold_since[lvl] = (hold_since[lvl] or now) if loose >= lvl else None
                if not reported and hold_since[3] is not None and now - hold_since[3] >= cfg["HOLD_S"]:
                    reported = self._report(spec, ev, "Lv3 held")
                    report_at = now
                    if is_goal:
                        break
            st = api.goal_state()
            if st == "SUCCEEDED" and success_at is None:
                success_at = now
            elif st in self.TERMINAL_FAIL and success_at is None:
                if retries < cfg["MAX_GOAL_RETRY"]:
                    retries += 1
                    self.log("ctl goal ended with %s -> resend (%d)" % (st, retries), "warn")
                    self._send(spec.position, spec.target_q)
                elif not reported:
                    self._report_fallback(spec, hold_since, nv)
                    reported = True
                    break
            if reported and success_at is not None:
                break
            if reported and report_at is not None and now - report_at > cfg["WAIT_SUCCESS_AFTER_REPORT_S"]:
                break  # next goal will preempt; ctl handles it
            if success_at is not None and not reported and now - success_at > cfg["WAIT_TIGHT_AFTER_SUCCESS_S"]:
                if refines < cfg["MAX_REFINE"] and self.time_left() > cfg["SAFETY_MARGIN_S"] + 20.0:
                    refines += 1
                    success_at = None
                    self.log("Lv3 not reached after ctl success -> refine (%d)" % refines, "warn")
                    self._send(spec.position, spec.target_q)
                else:
                    reported = self._report_fallback(spec, hold_since, nv) or True
                    break
            # time guards
            if now - t_leg0 > timeout:
                self.log("leg timeout for cp %d" % spec.cid, "warn")
                if not reported:
                    self._report_fallback(spec, hold_since, nv)
                break
            if is_goal:
                if self.elapsed() > cfg["T_LIMIT_S"] - cfg["FINISH_GUARD_S"] - 1.0:
                    if not reported:
                        self._report_fallback(spec, hold_since, nv, allow_no_hold=True)
                    break
            elif goal is not None and nv is not None:
                need = self.goal_eta_from(nv[0], nv[1], goal)
                if self.elapsed() + need + 2.0 > cfg["T_LIMIT_S"] - cfg["FINISH_GUARD_S"]:
                    self.log("abandoning cp %d to protect the goal" % spec.cid, "warn")
                    if not reported:
                        self._report_fallback(spec, hold_since, nv, allow_no_hold=True)
                    break
            api.sleep(cfg["LOOP_DT"])

        actual = api.now() - t_leg0
        self.legs.append({"cid": spec.cid, "mode": mode, "pred": pred[0], "actual": actual,
                          "level": self.results.get(spec.cid, {}).get("level", 0)})
        return True

    def _report(self, spec, ev, why):
        ok = self.api.report_checkpoint(spec.cid)
        self.results[spec.cid] = {"level": ev["level"], "dist": ev["dist"], "cen": ev["cen"],
                                  "roll": ev["roll"], "t": self.elapsed(), "ok": ok, "why": why}
        self.log("REPORT cp %d level=%d dist=%.3f cen=%.1f roll=%.1f t=%.1fs (%s)" % (
            spec.cid, ev["level"], ev["dist"], ev["cen"], ev["roll"], self.elapsed(), why))
        return True

    def _report_fallback(self, spec, hold_since, nv, allow_no_hold=False):
        """Report at the best level currently held (never report a certain FAIL)."""
        if nv is None:
            return False
        ev = evaluate(spec, nv[0], nv[1])
        now = self.api.now()
        for lvl in (3, 2, 1):
            hs = hold_since.get(lvl)
            held = hs is not None and now - hs >= self.cfg["HOLD_S"]
            if ev["level"] >= lvl and (held or allow_no_hold):
                return self._report(spec, ev, "fallback, held >= Lv%d" % lvl)
        self.log("cp %d not reported (level=%d) to avoid a FAIL" % (spec.cid, ev["level"]), "warn")
        return False

    # ---- summary ------------------------------------------------------------------
    def summary(self):
        t = self.elapsed() if self.t0 is not None else 0.0
        lv = [r["level"] for r in self.results.values()]
        s = {"mission_time": t, "reported": len(lv), "levels": lv, "legs": self.legs}
        self.log("SUMMARY time=%.1fs reported=%d levels=%s" % (t, len(lv), lv))
        for lg in self.legs:
            self.log("  leg cp%-3d %-12s pred=%6.1fs actual=%6.1fs level=%d" % (
                lg["cid"], lg["mode"], lg["pred"], lg["actual"], lg["level"]))
        return s

    def watchdog_due(self):
        """True when the hard deadline guard must take over (mission started, not finished,
        elapsed >= T_LIMIT - DEADLINE_WATCHDOG_S)."""
        if self.t0 is None or self.finished or self.finishing:
            return False
        return self.elapsed() >= self.cfg["T_LIMIT_S"] - self.cfg["DEADLINE_WATCHDOG_S"]


def run_mission_guarded(api, make_mission):
    """Run one mission so that EVERY exit path ends safely (v3 fix).

    Normal end: Mission.run() already sent mission_finished and ran the protective stop.
    Exception / BaseException / run() returning without a finish report: the protective stop
    runs first (FAST off, zero wrench, ctl KEEP_POSE), then exactly one mission_finished.
    Returns (mission or None, crashed: bool). Exceptions other than BaseException never escape.
    """
    mission, crashed = None, False
    try:
        mission = make_mission()
        mission.run()
    except Exception:
        crashed = True
        _api_log(api, "error", "mission crashed: %s" % traceback.format_exc())
    finally:
        try:
            if mission is None:
                # constructor failed: no mission_start was sent, but a FAST controller may be attached
                safe_stop(api, None, "mission setup crashed", CONFIG, CONFIG["SAFE_STOP_CTL_WAIT_S"], keep_pose=False)
            elif not mission.finished:
                why = "mission crashed" if crashed else "mission ended without finish report"
                mission.emergency_finish(why)
        except Exception:
            _api_log(api, "error", "crash handler failed: %s" % traceback.format_exc())
    return mission, crashed


# =============================================================================
# 7. ROS adapter
# =============================================================================
class RosApi(object):
    STATE_NAMES = {0: "PENDING", 1: "ACTIVE", 2: "PREEMPTED", 3: "SUCCEEDED", 4: "ABORTED",
                   5: "REJECTED", 6: "PREEMPTING", 7: "RECALLING", 8: "RECALLED", 9: "LOST"}

    def __init__(self, node):
        self.node = node
        self._nav_lock = threading.Lock()
        self._nav = None
        self._nav_full = None
        self._nav_rx = None
        self._goal_sent = False
        self._fast = None
        self._last_wrench_t = 0.0
        self._wrench_pub = rospy.Publisher("/ctl/wrench", WrenchStamped, queue_size=1)
        self._cmd = actionlib.SimpleActionClient("/ctl/command", CtlCommandAction)
        self._cp = actionlib.SimpleActionClient("/event/checkpoint_passed", CheckpointPassedAction)
        self._srv_start = rospy.ServiceProxy("/event/mission_start", MissionStart)
        self._srv_finish = rospy.ServiceProxy("/event/mission_finished", MissionFinished)
        self._srv_cps = rospy.ServiceProxy("/mission/get_checkpoints", CheckPoints)
        self._nav_sub = rospy.Subscriber("/sensor_fusion/navigation", Navigation, self._on_nav, queue_size=1)
        rospy.on_shutdown(self.publish_zero_wrench)

    def _on_nav(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        lv = msg.twist.linear
        av = msg.twist.angular
        r = [p.x, p.y, p.z]
        q = [o.x, o.y, o.z, o.w]
        v = [lv.x, lv.y, lv.z]      # world frame (ctl pos controller convention)
        w = [av.x, av.y, av.z]      # body frame (ctl att controller convention)
        now = rospy.get_time()
        with self._nav_lock:
            self._nav = (r, q)
            self._nav_full = (r, q, v, w)
            self._nav_rx = now
        fast = self._fast
        if fast is not None and fast.active():
            if now - self._last_wrench_t >= 1.0 / CONFIG["FAST_RATE_HZ"]:
                self._last_wrench_t = now
                out = fast.wrench(now, r, v, q, w)
                if out is not None:
                    self._publish_wrench(out[0], out[1], now)
                elif fast.fault:
                    self.publish_zero_wrench()

    def _publish_wrench(self, force, torque, stamp_s=None):
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.from_sec(stamp_s) if stamp_s is not None else rospy.Time.now()
        msg.header.frame_id = "body"
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = torque
        self._wrench_pub.publish(msg)

    # --- FAST mode hooks ---------------------------------------------------------
    def attach_fast_controller(self, ctrl):
        self._fast = ctrl

    def publish_zero_wrench(self):
        try:
            self._publish_wrench([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        except Exception:
            pass

    def _ctl_mode(self, type_value, server_s=None, result_s=3.0):
        """None = goal not sent (no server), False = sent but no result in time, True = confirmed."""
        if server_s is None:
            server_s = CONFIG["SERVICE_TIMEOUT_S"]
        if not self._cmd.wait_for_server(rospy.Duration(server_s)):
            return None
        target = PoseStamped()
        target.pose.orientation.w = 1.0
        self._cmd.send_goal(CtlCommandGoal(target=target, type=CtlStatusType(type=type_value)))
        self._goal_sent = True
        return bool(self._cmd.wait_for_result(rospy.Duration(result_s)))

    def ctl_standby(self):
        """ctl.cpp: type < STOP_MOVING -> STAND_BY, publishes one zero wrench, then stays silent."""
        return bool(self._ctl_mode(CtlStatusType.STAND_BY))

    def ctl_keep_pose(self, server_s=None, result_s=3.0):
        """Hand control back to the onboard ctl (KEEP_POSE at the current pose; ctl.cpp
        commandCallback: type < STOP_MOVING -> setKeepPose()). A new goal also preempts a
        running MOVE goal. Returns True / False / None like _ctl_mode."""
        return self._ctl_mode(CtlStatusType.KEEP_POSE, server_s, result_s)

    def nav_full(self):
        with self._nav_lock:
            return self._nav_full

    def nav_age(self):
        with self._nav_lock:
            rx = self._nav_rx
        return float("inf") if rx is None else rospy.get_time() - rx

    # --- api -----------------------------------------------------------------
    def now(self):
        return rospy.get_time()

    def sleep(self, dt):
        rospy.sleep(dt)

    def nav(self):
        with self._nav_lock:
            return self._nav

    def send_goal(self, pos, q):
        target = PoseStamped()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = pos
        target.pose.orientation.x, target.pose.orientation.y = q[0], q[1]
        target.pose.orientation.z, target.pose.orientation.w = q[2], q[3]
        goal = CtlCommandGoal(target=target, type=CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET))
        if not self._cmd.wait_for_server(rospy.Duration(CONFIG["SERVICE_TIMEOUT_S"])):
            rospy.logerr("/ctl/command server not available")
            return False
        self._cmd.send_goal(goal)
        self._goal_sent = True
        return True

    def goal_state(self):
        if not self._goal_sent:
            return "NONE"
        st = self._cmd.get_state()
        name = self.STATE_NAMES.get(st, "UNKNOWN")
        if name == "SUCCEEDED":
            res = self._cmd.get_result()
            if res is not None and res.type != CtlCommandResult.TERMINATE_SUCCESS:
                return "FAILED"
        return name

    def cancel_goal(self):
        if self._goal_sent:
            self._cmd.cancel_goal()

    def report_checkpoint(self, cid):
        if not self._cp.wait_for_server(rospy.Duration(CONFIG["SERVICE_TIMEOUT_S"])):
            rospy.logerr("/event/checkpoint_passed server not available")
            return False
        self._cp.send_goal(CheckpointPassedGoal(checkpoint_id=int(cid)))
        if self._cp.wait_for_result(rospy.Duration(CONFIG["REPORT_TIMEOUT_S"])):
            res = self._cp.get_result()
            return bool(res and res.success)
        return False

    def _call(self, proxy, name):
        try:
            rospy.wait_for_service(name, timeout=CONFIG["SERVICE_TIMEOUT_S"])
            return proxy()
        except Exception as exc:
            rospy.logerr("service %s failed: %s" % (name, exc))
            return None

    def mission_start(self):
        res = self._call(self._srv_start, "/event/mission_start")
        return bool(res and res.success)

    def mission_finished(self):
        res = self._call(self._srv_finish, "/event/mission_finished")
        return bool(res and res.success)

    def get_checkpoints(self):
        res = self._call(self._srv_cps, "/mission/get_checkpoints")
        if res is None or not res.success:
            return None
        return res.message

    def stop_requested(self):
        return self.node.stop_flag or rospy.is_shutdown()

    def aruco_observe(self, cid):
        """v4: marker tvec in the camera frame (only with CONFIG["ARUCO_ALIGN"] != "off")."""
        cam = getattr(self.node, "camera", None)
        if cam is None or CONFIG["ARUCO_ALIGN"] == "off":
            return None
        return cam.marker_tvec(cid)

    def log(self, level, text):
        fn = {"info": rospy.loginfo, "warn": rospy.logwarn, "error": rospy.logerr}.get(level, rospy.loginfo)
        fn("[GaneshaOrbit] " + text)


class CameraCheck(object):
    """Optional camera helper (never blocks, never raises): ArUco IDs visible at report time
    (ENABLE_CAMERA_CHECK) and, for CONFIG["ARUCO_ALIGN"] != "off", the marker position in the
    OpenCV camera frame (Programming Manual 5.13: DICT_4X4, 2 cm, /camera_main/camera_info K, D)."""

    def __init__(self):
        self.ok = False
        self._last = None
        self._info = None
        self._used_stamp = None
        try:
            import cv2  # noqa
            import numpy
            from cv_bridge import CvBridge
            from sensor_msgs.msg import Image, CameraInfo
            self.cv2 = cv2
            self.np = numpy
            self.bridge = CvBridge()
            self.aruco = cv2.aruco
            self.dictionary = self.aruco.getPredefinedDictionary(self.aruco.DICT_4X4_50)
            rospy.Subscriber("/camera_main/image_raw", Image, self._on_img, queue_size=1, buff_size=2 ** 22)
            rospy.Subscriber("/camera_main/camera_info", CameraInfo, self._on_info, queue_size=1)
            self.ok = True
        except Exception as exc:
            rospy.logwarn("camera check disabled: %s" % exc)

    def _on_img(self, msg):
        self._last = msg

    def _on_info(self, msg):
        self._info = msg

    def marker_tvec(self, cid):
        """tvec [m] of marker `cid` in the camera frame from a frame not used before, else None."""
        msg, info = self._last, self._info
        if not self.ok or msg is None or info is None:
            return None
        try:
            stamp = msg.header.stamp.to_sec()
            if stamp == self._used_stamp:
                return None
            self._used_stamp = stamp
            img = self.bridge.imgmsg_to_cv2(msg, "mono8")
            corners, ids, _ = self.aruco.detectMarkers(img, self.dictionary)
            if ids is None:
                return None
            flat = [int(i) for i in ids.flatten()]
            if int(cid) not in flat:
                return None
            k = flat.index(int(cid))
            K = self.np.array(info.K, dtype=float).reshape(3, 3)
            D = self.np.array(info.D, dtype=float)
            out = self.aruco.estimatePoseSingleMarkers([corners[k]], CONFIG["ARUCO_MARKER_LEN_M"], K, D)
            t = out[1].reshape(-1)
            return [float(t[0]), float(t[1]), float(t[2])]
        except Exception:
            return None

    def visible_ids(self):
        if not self.ok or self._last is None:
            return None
        try:
            img = self.bridge.imgmsg_to_cv2(self._last, "mono8")
            _, ids, _ = self.aruco.detectMarkers(img, self.dictionary)
            return [] if ids is None else sorted(int(i) for i in ids.flatten())
        except Exception:
            return None


class UserTemplate(object):
    """Keeps the official template contract; mission runs in a worker thread."""

    def __init__(self):
        rospy.init_node("krpc_user")
        self.stop_flag = False
        self.__started = False
        self.mission = None
        self.api = RosApi(self)
        self.camera = CameraCheck() if (CONFIG["ENABLE_CAMERA_CHECK"] or CONFIG["ARUCO_ALIGN"] != "off") else None
        self.__subscriber_start = rospy.Subscriber("/ib2_user/start", UserLogic, self.__callback_start)  # DO NOT REMOVE
        self.__stop_processing_server = rospy.Service("/ib2_user/stop", StopProcessingUserNode, self.__stop_processing)  # DO NOT REMOVE
        rospy.set_param("/ib2_user/ready", True)  # DO NOT REMOVE

    def __callback_start(self, msg):
        if self.__started:
            rospy.logwarn("start callback is already processed")
            return
        self.__started = True
        rospy.loginfo("start %s", msg)
        th = threading.Thread(target=self.__run_mission)
        th.daemon = True
        th.start()

    def __make_mission(self):
        mission = Mission(self.api)
        self.mission = mission
        if self.camera is not None and CONFIG["ENABLE_CAMERA_CHECK"]:
            orig = mission._report

            def report_with_camera(spec, ev, why):
                rospy.loginfo("[GaneshaOrbit] visible markers at report: %s" % self.camera.visible_ids())
                return orig(spec, ev, why)
            mission._report = report_with_camera
        wd = threading.Thread(target=self.__deadline_watchdog, args=(mission,))
        wd.daemon = True
        wd.start()
        return mission

    def __run_mission(self):
        # crash-safe: any exception -> FAST off, zero wrench, ctl KEEP_POSE, then mission_finished
        run_mission_guarded(self.api, self.__make_mission)

    def __deadline_watchdog(self, mission):
        """Hard 5-minute guard independent of the mission thread: if mission_finished has not
        been sent by T_LIMIT - DEADLINE_WATCHDOG_S (mission thread hung or too slow), stop the
        mission loops, run the shared protective stop and send mission_finished."""
        try:
            while not mission.finished and not rospy.is_shutdown():
                if mission.watchdog_due():
                    rospy.logerr("[GaneshaOrbit] deadline watchdog: no finish report at t=%.1fs -> safe stop + finish"
                                 % mission.elapsed())
                    self.stop_flag = True
                    mission.emergency_finish("deadline watchdog")
                    return
                self.api.sleep(0.05)
        except Exception:
            if not rospy.is_shutdown():  # rospy.sleep raises on node shutdown: not an error
                rospy.logerr("deadline watchdog failed: %s" % traceback.format_exc())

    def __stop_processing(self, request):
        try:
            self.stop_flag = True
            # same protective stop as the crash / deadline paths; the KEEP_POSE goal also
            # preempts a running MOVE goal (cancel_goal is used only if it cannot be sent)
            m = self.mission
            started = m is not None and m.t0 is not None
            safe_stop(self.api, self.api._fast, "/ib2_user/stop", CONFIG, CONFIG["SAFE_STOP_CTL_WAIT_S"],
                      keep_pose=started)
            if not started:
                self.api.cancel_goal()  # official template behaviour (no-op when no goal was sent)
            rospy.loginfo("finish processing")
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)
        except Exception as exc:
            rospy.logerr("stop processing failed: %s", exc)
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.ERROR)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    if not ROS_AVAILABLE:
        raise SystemExit("ROS is not available: run tools/offline_sim.py for offline testing")
    UserTemplate().run()
